#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "ros2_tf_project/action/simple_navigation.hpp"
#include "ros2_tf_project/srv/simple_trajectory.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>

using namespace std::chrono_literals;
using SimpleNavigation = ros2_tf_project::action::SimpleNavigation;
using GoalHandleSimpleNavigation =
    rclcpp_action::ServerGoalHandle<SimpleNavigation>;

class SimpleNavigationActionServer : public rclcpp::Node {
public:
  SimpleNavigationActionServer()
      : Node("simple_navigation_action_server"), tf_buffer_(this->get_clock()),
        tf_listener_(tf_buffer_) {

    action_server_ = rclcpp_action::create_server<SimpleNavigation>(
        this, "/simple_navigation",
        std::bind(&SimpleNavigationActionServer::handle_goal, this,
                  std::placeholders::_1, std::placeholders::_2),
        std::bind(&SimpleNavigationActionServer::handle_cancel, this,
                  std::placeholders::_1),
        std::bind(&SimpleNavigationActionServer::handle_accepted, this,
                  std::placeholders::_1));

    client_ = this->create_client<ros2_tf_project::srv::SimpleTrajectory>(
        "/simple_trajectory");
    cmd_vel_pub_ =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
                "Simple Navigation Action Server is ready.");
  }

private:
  rclcpp_action::Server<SimpleNavigation>::SharedPtr action_server_;
  rclcpp::Client<ros2_tf_project::srv::SimpleTrajectory>::SharedPtr client_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &,
              std::shared_ptr<const SimpleNavigation::Goal> goal) {
    RCLCPP_INFO(
        this->get_logger(), "Received goal request: start=%s, clockwise=%s",
        goal->start ? "true" : "false", goal->clockwise ? "true" : "false");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleSimpleNavigation>) {
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(
      const std::shared_ptr<GoalHandleSimpleNavigation> goal_handle) {
    std::thread{
        std::bind(&SimpleNavigationActionServer::execute, this, goal_handle)}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleSimpleNavigation> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal...");

    auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<SimpleNavigation::Feedback>();
    auto result = std::make_shared<SimpleNavigation::Result>();

    auto request =
        std::make_shared<ros2_tf_project::srv::SimpleTrajectory::Request>();
    request->start = true;
    request->clockwise = goal->clockwise;

    if (!client_->wait_for_service(5s)) {
      RCLCPP_ERROR(this->get_logger(),
                   "SimpleTrajectory service not available.");
      result->completed = false;
      result->message = "Service not available";
      goal_handle->abort(result);
      return;
    }

    auto future = client_->async_send_request(request);
    if (future.future.wait_for(5s) != std::future_status::ready) {
      RCLCPP_ERROR(this->get_logger(), "Service call timed out.");
      result->completed = false;
      result->message = "Service timeout";
      goal_handle->abort(result);
      return;
    }

    auto response = future.future.get();
    if (!response->completed) {
      RCLCPP_ERROR(this->get_logger(), "Trajectory service failed.");
      result->completed = false;
      result->message = response->message;
      goal_handle->abort(result);
      return;
    }

    rclcpp::Rate rate(10);
    double total_distance = 0.0;
    auto last_time = this->now();
    geometry_msgs::msg::TransformStamped start_tf;

    try {
      start_tf =
          tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Start TF lookup failed: %s", ex.what());
      goal_handle->abort(result);
      return;
    }

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled.");
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist());
        goal_handle->canceled(result);
        return;
      }

      geometry_msgs::msg::TransformStamped odom_to_robot, odom_to_waypoint;
      try {
        odom_to_robot =
            tf_buffer_.lookupTransform("odom", "base_link", tf2::TimePointZero);
        odom_to_waypoint = tf_buffer_.lookupTransform("odom", "moving_waypoint",
                                                      tf2::TimePointZero);
      } catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
        rate.sleep();
        continue;
      }

      double dx = odom_to_waypoint.transform.translation.x -
                  odom_to_robot.transform.translation.x;
      double dy = odom_to_waypoint.transform.translation.y -
                  odom_to_robot.transform.translation.y;
      double distance = std::sqrt(dx * dx + dy * dy);
      double angle_to_target = std::atan2(dy, dx);

      tf2::Quaternion q(odom_to_robot.transform.rotation.x,
                        odom_to_robot.transform.rotation.y,
                        odom_to_robot.transform.rotation.z,
                        odom_to_robot.transform.rotation.w);
      tf2::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);

      double angle_error = angle_to_target - yaw;

      // Normalize angle error to [-π, π]
      while (angle_error > M_PI)
        angle_error -= 2 * M_PI;
      while (angle_error < -M_PI)
        angle_error += 2 * M_PI;

      geometry_msgs::msg::Twist cmd;

      if (distance < 0.05) {
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
      } else {
        double K_linear = 0.5;
        double K_angular = 1.5;
        double max_linear = 0.3;
        double max_angular = 1.0;

        cmd.linear.x = std::clamp(K_linear * distance, -max_linear, max_linear);
        cmd.angular.z =
            std::clamp(K_angular * angle_error, -max_angular, max_angular);
      }

      cmd_vel_pub_->publish(cmd);

      auto now = this->now();
      total_distance += std::abs(cmd.linear.x) * (now - last_time).seconds();
      last_time = now;

      feedback->current_distance = static_cast<float>(total_distance);
      goal_handle->publish_feedback(feedback);

      double start_dx = start_tf.transform.translation.x -
                        odom_to_robot.transform.translation.x;
      double start_dy = start_tf.transform.translation.y -
                        odom_to_robot.transform.translation.y;
      double dist_to_start =
          std::sqrt(start_dx * start_dx + start_dy * start_dy);

      if (total_distance > 2.0 && dist_to_start < 0.2) {
        RCLCPP_INFO(this->get_logger(), "Completed full lap.");
        break;
      }

      rate.sleep();
    }

    cmd_vel_pub_->publish(geometry_msgs::msg::Twist());

    result->completed = true;
    result->message = "Navigation complete.";
    result->total_distance = static_cast<float>(total_distance);
    goal_handle->succeed(result);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SimpleNavigationActionServer>());
  rclcpp::shutdown();
  return 0;
}

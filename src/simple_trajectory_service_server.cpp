#include "ros2_tf_project/srv/simple_trajectory.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>
#include <filesystem>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <yaml-cpp/yaml.h>

using namespace std::chrono_literals;

class SimpleTrajectoryServer : public rclcpp::Node {
public:
  SimpleTrajectoryServer() : Node("simple_trajectory_service_server") {
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    tf_dynamic_broadcaster_ =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);

    loadStaticFrames();

    service_ = this->create_service<ros2_tf_project::srv::SimpleTrajectory>(
        "/simple_trajectory",
        std::bind(&SimpleTrajectoryServer::handleTrajectoryRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Trajectory Service Ready");
  }

private:
  struct ArenaCenter {
    double x;
    double y;
  };

  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_dynamic_broadcaster_;
  rclcpp::Service<ros2_tf_project::srv::SimpleTrajectory>::SharedPtr service_;
  rclcpp::TimerBase::SharedPtr update_timer_;

  ArenaCenter arena_center_;
  double trajectory_a_ = 1.0;
  double trajectory_b_ = 1.0;
  double safety_margin_ = 0.2;

  bool is_moving_ = false;
  double current_angle_ = 0.0;
  double initial_angle_ = 0.0;
  double angular_step_ = 0.01;

  void loadStaticFrames() {
    const std::string package_path =
        ament_index_cpp::get_package_share_directory("ros2_tf_project");
    const std::string filename =
        package_path + "/frames/static_frames_sim.yaml";

    try {
      YAML::Node config = YAML::LoadFile(filename);
      std::vector<std::string> corner_names = {"corner_1", "corner_2",
                                               "corner_3", "corner_4"};

      std::map<std::string, geometry_msgs::msg::TransformStamped>
          all_transforms;
      std::vector<geometry_msgs::msg::Vector3> corners;

      RCLCPP_INFO(this->get_logger(), "Loading static frames from: %s",
                  filename.c_str());

      for (const auto &frame : config["feature_frames"]) {
        const std::string name = frame["feature"].as<std::string>();

        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = rclcpp::Time(0);
        tf.header.frame_id = "odom";
        tf.child_frame_id = name;
        tf.transform.translation.x = frame["position"]["x"].as<double>();
        tf.transform.translation.y = frame["position"]["y"].as<double>();
        tf.transform.translation.z = frame["position"]["z"].as<double>();
        tf.transform.rotation.w = 1.0;

        tf_static_broadcaster_->sendTransform(tf);
        all_transforms[name] = tf;
        RCLCPP_DEBUG(this->get_logger(), "Published static frame: %s",
                     name.c_str());
      }

      for (const std::string &corner : corner_names) {
        if (all_transforms.count(corner)) {
          geometry_msgs::msg::Vector3 v =
              all_transforms[corner].transform.translation;
          corners.push_back(v);
        } else {
          RCLCPP_WARN(this->get_logger(),
                      "Corner frame '%s' not found in YAML!", corner.c_str());
        }
      }

      if (corners.size() == 4) {
        double sum_x = 0.0, sum_y = 0.0;
        double min_x = corners[0].x, max_x = corners[0].x;
        double min_y = corners[0].y, max_y = corners[0].y;

        for (const auto &pt : corners) {
          sum_x += pt.x;
          sum_y += pt.y;
          min_x = std::min(min_x, pt.x);
          max_x = std::max(max_x, pt.x);
          min_y = std::min(min_y, pt.y);
          max_y = std::max(max_y, pt.y);
        }

        arena_center_.x = sum_x / 4.0;
        arena_center_.y = sum_y / 4.0;

        double safe_x =
            std::min(arena_center_.x - min_x, max_x - arena_center_.x) -
            safety_margin_;
        double safe_y =
            std::min(arena_center_.y - min_y, max_y - arena_center_.y) -
            safety_margin_;

        trajectory_a_ = safe_x;
        trajectory_b_ = safe_y;

        RCLCPP_INFO(this->get_logger(), "Arena center: x=%.3f, y=%.3f",
                    arena_center_.x, arena_center_.y);
        RCLCPP_INFO(this->get_logger(),
                    "Trajectory dimensions: a=%.2fm, b=%.2fm", trajectory_a_,
                    trajectory_b_);
      } else {
        RCLCPP_ERROR(
            this->get_logger(),
            "Could not calculate arena center. Only %zu corners found.",
            corners.size());
      }

      geometry_msgs::msg::TransformStamped center_tf;
      center_tf.header.stamp = rclcpp::Time(0);
      center_tf.header.frame_id = "odom";
      center_tf.child_frame_id = "arena_center";
      center_tf.transform.translation.x = arena_center_.x;
      center_tf.transform.translation.y = arena_center_.y;
      center_tf.transform.translation.z = 0.0;
      center_tf.transform.rotation.w = 1.0;
      tf_static_broadcaster_->sendTransform(center_tf);

    } catch (const YAML::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s\nFile: %s",
                   e.what(), filename.c_str());
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load static frames: %s",
                   e.what());
    }
  }

  void handleTrajectoryRequest(
      const std::shared_ptr<ros2_tf_project::srv::SimpleTrajectory::Request>
          req,
      std::shared_ptr<ros2_tf_project::srv::SimpleTrajectory::Response> res) {

    if (!is_moving_) {
      RCLCPP_INFO(this->get_logger(), "Starting %s trajectory",
                  req->clockwise ? "clockwise" : "counter-clockwise");

      initial_angle_ = M_PI / 2;
      current_angle_ = initial_angle_;
      is_moving_ = true;

      bool clockwise = req->clockwise;

      update_timer_ = this->create_wall_timer(
          50ms, [this, clockwise]() { this->updateTrajectory(clockwise); });

      res->completed = true;
      res->message = "Trajectory started successfully";
    } else {
      res->completed = false;
      res->message = "Trajectory already in progress";
    }
  }

  void updateTrajectory(bool clockwise) {
    double direction = clockwise ? -1.0 : 1.0;
    current_angle_ += angular_step_ * direction;

    if (std::abs(current_angle_ - initial_angle_) >= 2 * M_PI) {
      RCLCPP_INFO(this->get_logger(), "Trajectory completed");
      is_moving_ = false;
      update_timer_->cancel();
      return;
    }

    const double x = trajectory_a_ * cos(current_angle_);
    const double y = trajectory_b_ * sin(current_angle_);

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = this->now();
    tf.header.frame_id = "arena_center";
    tf.child_frame_id = "moving_waypoint";

    tf.transform.translation.x = x;
    tf.transform.translation.y = y;
    tf.transform.translation.z = 0.1;
    tf.transform.rotation.w = 1.0;

    tf_dynamic_broadcaster_->sendTransform(tf);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleTrajectoryServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

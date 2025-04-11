#include "ros2_tf_project/srv/feature_detection.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class FeatureDetectionServer : public rclcpp::Node {
public:
  FeatureDetectionServer() : Node("feature_detection_service_server") {
    // Initialize TF components
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Create service
    service_ = this->create_service<ros2_tf_project::srv::FeatureDetection>(
        "/feature_detection",
        std::bind(&FeatureDetectionServer::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Subscribe to laser scan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&FeatureDetectionServer::scan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Feature detection service ready");
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  int angle_to_index(float angle_deg) const {
    if (!latest_scan_)
      return -1;
    float angle_rad = angle_deg * M_PI / 180.0;
    return static_cast<int>(std::round((angle_rad - latest_scan_->angle_min) /
                                       latest_scan_->angle_increment));
  }

  void handle_service_request(
      const std::shared_ptr<ros2_tf_project::srv::FeatureDetection::Request>
          request,
      std::shared_ptr<ros2_tf_project::srv::FeatureDetection::Response>
          response) {

    response->detected = false;
    response->message = "";

    if (!latest_scan_) {
      response->message = "No scan data available";
      return;
    }

    if (request->feature_id.empty()) {
      response->message = "Empty feature_id provided";
      return;
    }

    // Define frontal sector (60° front: -30° to +30°)
    const int front_start_deg = -30;
    const int front_end_deg = 30;
    int start_idx = angle_to_index(front_start_deg);
    int end_idx = angle_to_index(front_end_deg);

    // Validate indices
    start_idx = std::clamp(start_idx, 0,
                           static_cast<int>(latest_scan_->ranges.size()) - 1);
    end_idx = std::clamp(end_idx, 0,
                         static_cast<int>(latest_scan_->ranges.size()) - 1);
    if (start_idx > end_idx)
      std::swap(start_idx, end_idx);

    // Find minimum distance in frontal sector
    float min_distance = std::numeric_limits<float>::max();
    int min_idx = -1;

    for (int i = start_idx; i <= end_idx; ++i) {
      float range = latest_scan_->ranges[i];
      if (std::isfinite(range)) {
        range =
            std::clamp(range, latest_scan_->range_min, latest_scan_->range_max);
        if (range < min_distance) {
          min_distance = range;
          min_idx = i;
        }
      }
    }

    if (min_idx == -1 || min_distance > 3.5) { // 3.5m detection range limit
      response->message = "No valid feature detected in frontal sector";
      return;
    }

    // Calculate position in laser frame
    float angle =
        latest_scan_->angle_min + min_idx * latest_scan_->angle_increment;
    geometry_msgs::msg::Point point;
    point.x = min_distance * cos(angle);
    point.y = min_distance * sin(angle);
    point.z = 0.0;

    // Transform to odom frame
    geometry_msgs::msg::PointStamped point_laser, point_odom;
    point_laser.header.frame_id = "base_scan"; // Correct laser frame
    point_laser.header.stamp = this->now();
    point_laser.point = point;

    try {
      // Get transform with longer timeout
      point_odom =
          tf_buffer_->transform(point_laser, "odom", tf2::durationFromSec(1.0));

      // Broadcast static TF with proper timestamp
      geometry_msgs::msg::TransformStamped transform;
      transform.header.stamp = rclcpp::Time(0); // Static TF requirement
      transform.header.frame_id = "odom";
      transform.child_frame_id = request->feature_id;
      transform.transform.translation.x = point_odom.point.x;
      transform.transform.translation.y = point_odom.point.y;
      transform.transform.translation.z = point_odom.point.z;
      transform.transform.rotation.w = 1.0; // No rotation

      tf_static_broadcaster_->sendTransform(transform);

      // Set response
      response->detected = true;
      response->message =
          "Feature detected and TF broadcasted: " + request->feature_id;
      response->position = point_odom.point;

      RCLCPP_INFO(this->get_logger(), "Detected feature '%s' at (%.2f, %.2f)",
                  request->feature_id.c_str(), point_odom.point.x,
                  point_odom.point.y);

    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "TF exception: %s", ex.what());
      response->message = std::string("TF transform failed: ") + ex.what();
    }
  }

  // Member variables
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Service<ros2_tf_project::srv::FeatureDetection>::SharedPtr service_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<FeatureDetectionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
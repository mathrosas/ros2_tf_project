#include "ros2_tf_project/srv/feature_detection.hpp"
#include <Eigen/Dense>
#include <algorithm>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <filesystem>
#include <fstream>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <stack>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <vector>

using namespace std::chrono_literals;

class FeatureDetectionServer : public rclcpp::Node {
public:
  FeatureDetectionServer() : Node("feature_detection_service_server") {
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_static_broadcaster_ =
        std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    try {
      auto transform = tf_buffer_->lookupTransform("odom", "base_link",
                                                   tf2::TimePointZero, 1s);
      geometry_msgs::msg::TransformStamped start_tf;
      start_tf.header.stamp = this->now();
      start_tf.header.frame_id = "odom";
      start_tf.child_frame_id = "robot_start";
      start_tf.transform = transform.transform;
      tf_static_broadcaster_->sendTransform(start_tf);

      // Save start position to feature frames
      FeatureFrame start_frame;
      start_frame.id = "start_frame";
      start_frame.position.x = start_tf.transform.translation.x;
      start_frame.position.y = start_tf.transform.translation.y;
      start_frame.position.z = start_tf.transform.translation.z;
      feature_frames_.push_back(start_frame);

      RCLCPP_INFO(this->get_logger(), "Saved initial robot position");
    } catch (const tf2::TransformException &ex) {
      RCLCPP_ERROR(this->get_logger(), "Initial position error: %s", ex.what());
    }

    service_ = this->create_service<ros2_tf_project::srv::FeatureDetection>(
        "/feature_detection",
        std::bind(&FeatureDetectionServer::handle_service_request, this,
                  std::placeholders::_1, std::placeholders::_2));

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&FeatureDetectionServer::scan_callback, this,
                  std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Service ready");
  }

  ~FeatureDetectionServer() { saveFramesToYAML(); }

private:
  struct DetectionResult {
    geometry_msgs::msg::Point position;
    bool valid = false;
  };

  struct FeatureFrame {
    std::string id;
    geometry_msgs::msg::Point position;
  };

  std::vector<FeatureFrame> feature_frames_;

  void saveFramesToYAML() {
    // Use ament_index to locate the package share path
    const std::string package_path =
        ament_index_cpp::get_package_share_directory("ros2_tf_project");
    const std::string frames_dir = package_path + "/frames";
    const std::string filename = frames_dir + "/static_frames_sim.yaml";

    std::filesystem::path dir_path(frames_dir);
    if (!std::filesystem::exists(dir_path)) {
      std::filesystem::create_directories(dir_path);
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open YAML file for writing");
      return;
    }

    file << "feature_frames:\n";
    for (const auto &frame : feature_frames_) {
      file << "  - feature: " << frame.id << "\n";
      file << "    position:\n";
      file << "      x: " << frame.position.x << "\n";
      file << "      y: " << frame.position.y << "\n";
      file << "      z: " << frame.position.z << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "Saved %zu features to %s",
                feature_frames_.size(), filename.c_str());
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    latest_scan_ = msg;
  }

  int angle_to_index(float angle_deg) const {
    if (!latest_scan_)
      return -1;
    float angle_rad = angle_deg * M_PI / 180.0f;
    int index = static_cast<int>(std::round(
        (angle_rad - latest_scan_->angle_min) / latest_scan_->angle_increment));
    return (index + latest_scan_->ranges.size()) % latest_scan_->ranges.size();
  }

  struct Line {
    double a, b, c;
  };

  Line fitLine(const std::vector<geometry_msgs::msg::Point> &points) const {
    if (points.size() < 2)
      return {0, 0, 0};

    Eigen::Vector2d centroid(0, 0);
    for (const auto &p : points) {
      centroid[0] += p.x;
      centroid[1] += p.y;
    }
    centroid /= points.size();

    Eigen::Matrix2d covariance = Eigen::Matrix2d::Zero();
    for (const auto &p : points) {
      Eigen::Vector2d diff = Eigen::Vector2d(p.x, p.y) - centroid;
      covariance += diff * diff.transpose();
    }
    covariance /= points.size();

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> solver(covariance);
    Eigen::Vector2d normal = solver.eigenvectors().col(0);

    return {normal[0], normal[1],
            -(normal[0] * centroid[0] + normal[1] * centroid[1])};
  }

  std::vector<std::vector<geometry_msgs::msg::Point>>
  splitAndMerge(const std::vector<geometry_msgs::msg::Point> &points,
                double split_threshold = 0.05,
                double merge_threshold = 0.17) const {
    std::vector<std::vector<geometry_msgs::msg::Point>> segments;
    if (points.empty())
      return segments;

    std::stack<std::vector<geometry_msgs::msg::Point>> to_process;
    to_process.push(points);

    while (!to_process.empty()) {
      auto current = to_process.top();
      to_process.pop();

      if (current.size() < 2) {
        segments.push_back(current);
        continue;
      }

      Line line = fitLine(current);
      double max_dist = 0.0;
      size_t split_idx = 0;

      for (size_t i = 0; i < current.size(); ++i) {
        const auto &p = current[i];
        double dist = std::abs(line.a * p.x + line.b * p.y + line.c) /
                      std::sqrt(line.a * line.a + line.b * line.b);
        if (dist > max_dist) {
          max_dist = dist;
          split_idx = i;
        }
      }

      if (max_dist > split_threshold && split_idx != 0 &&
          split_idx != current.size() - 1) {
        std::vector<geometry_msgs::msg::Point> left(
            current.begin(), current.begin() + split_idx + 1);
        std::vector<geometry_msgs::msg::Point> right(
            current.begin() + split_idx, current.end());
        to_process.push(right);
        to_process.push(left);
      } else {
        segments.push_back(current);
      }
    }

    if (segments.empty())
      return segments;

    std::vector<std::vector<geometry_msgs::msg::Point>> merged_segments;
    merged_segments.push_back(segments[0]);

    for (size_t i = 1; i < segments.size(); ++i) {
      auto &last = merged_segments.back();
      const auto &current = segments[i];

      Line line1 = fitLine(last);
      Line line2 = fitLine(current);

      double dot = line1.a * line2.a + line1.b * line2.b;
      double mag1 = std::hypot(line1.a, line1.b);
      double mag2 = std::hypot(line2.a, line2.b);
      double angle = std::acos(dot / (mag1 * mag2));

      if (angle < merge_threshold) {
        last.insert(last.end(), current.begin(), current.end());
      } else {
        merged_segments.push_back(current);
      }
    }

    return merged_segments;
  }

  geometry_msgs::msg::Point computeIntersection(const Line &line1,
                                                const Line &line2) const {
    geometry_msgs::msg::Point result;
    result.x = NAN;
    result.y = NAN;

    const double det = line1.a * line2.b - line2.a * line1.b;
    if (std::abs(det) < 1e-6)
      return result;

    result.x = (line1.b * line2.c - line2.b * line1.c) / det;
    result.y = (line2.a * line1.c - line1.a * line2.c) / det;
    return result;
  }

  DetectionResult find_closest_corner(int start_idx, int end_idx) const {
    DetectionResult result;

    std::vector<geometry_msgs::msg::Point> points;
    for (int i = start_idx; i <= end_idx; ++i) {
      if (std::isfinite(latest_scan_->ranges[i])) {
        const float angle =
            latest_scan_->angle_min + i * latest_scan_->angle_increment;
        geometry_msgs::msg::Point p;
        p.x = latest_scan_->ranges[i] * std::cos(angle);
        p.y = latest_scan_->ranges[i] * std::sin(angle);
        points.push_back(p);
      }
    }

    if (points.size() < 2)
      return result;

    auto segments = splitAndMerge(points, 0.05, 0.17);
    std::vector<geometry_msgs::msg::Point> corners;

    for (size_t i = 0; i < segments.size() - 1; ++i) {
      Line line1 = fitLine(segments[i]);
      Line line2 = fitLine(segments[i + 1]);
      auto intersection = computeIntersection(line1, line2);

      if (std::isfinite(intersection.x) && std::isfinite(intersection.y)) {
        corners.push_back(intersection);
      }
    }

    float min_dist_sq = std::numeric_limits<float>::max();
    for (const auto &corner : corners) {
      const float dist_sq = corner.x * corner.x + corner.y * corner.y;
      if (dist_sq < min_dist_sq) {
        min_dist_sq = dist_sq;
        result.position = corner;
        result.valid = true;
      }
    }

    return result;
  }

  DetectionResult find_closest_obstacle(int start_idx, int end_idx) const {
    DetectionResult result;
    float min_distance = std::numeric_limits<float>::max();
    int min_idx = -1;

    for (int i = start_idx; i <= end_idx; ++i) {
      if (std::isfinite(latest_scan_->ranges[i])) {
        float range =
            std::clamp(latest_scan_->ranges[i], latest_scan_->range_min,
                       latest_scan_->range_max);
        if (range < min_distance) {
          min_distance = range;
          min_idx = i;
        }
      }
    }

    if (min_idx != -1) {
      float angle =
          latest_scan_->angle_min + min_idx * latest_scan_->angle_increment;
      result.position.x = min_distance * cos(angle);
      result.position.y = min_distance * sin(angle);
      result.valid = true;
    }
    return result;
  }

  void handle_service_request(
      const std::shared_ptr<ros2_tf_project::srv::FeatureDetection::Request>
          req,
      std::shared_ptr<ros2_tf_project::srv::FeatureDetection::Response> res) {
    res->detected = false;
    res->message = "";

    if (!latest_scan_) {
      res->message = "No scan data";
      return;
    }

    const int start_deg = -60;
    const int end_deg = 60;

    int start_idx = angle_to_index(start_deg);
    int end_idx = angle_to_index(end_deg);

    start_idx = std::clamp(start_idx, 0,
                           static_cast<int>(latest_scan_->ranges.size()) - 1);
    end_idx = std::clamp(end_idx, 0,
                         static_cast<int>(latest_scan_->ranges.size()) - 1);

    DetectionResult detection;
    if (req->feature == "corner") {
      detection = find_closest_corner(start_idx, end_idx);
    } else if (req->feature == "obstacle") {
      detection = find_closest_obstacle(start_idx, end_idx);
    } else {
      res->message = "Unknown feature type";
      return;
    }

    if (!detection.valid) {
      res->message = "No " + req->feature + " detected";
      return;
    }

    geometry_msgs::msg::PointStamped point_laser, point_odom;
    point_laser.header.frame_id = "base_scan";
    point_laser.header.stamp = latest_scan_->header.stamp;
    point_laser.point = detection.position;

    try {
      auto transform = tf_buffer_->lookupTransform(
          "odom", point_laser.header.frame_id, point_laser.header.stamp, 100ms);

      tf2::doTransform(point_laser, point_odom, transform);

      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = rclcpp::Time(0);
      tf.header.frame_id = "odom";
      tf.child_frame_id = req->feature_id;
      tf.transform.translation.x = point_odom.point.x;
      tf.transform.translation.y = point_odom.point.y;
      tf.transform.translation.z = 0.0;
      tf.transform.rotation.w = 1.0;

      tf_static_broadcaster_->sendTransform(tf);

      // Save/update feature in YAML list
      FeatureFrame new_frame;
      new_frame.id = req->feature_id;
      new_frame.position = point_odom.point;

      auto it = std::find_if(
          feature_frames_.begin(), feature_frames_.end(),
          [&new_frame](const auto &f) { return f.id == new_frame.id; });
      if (it != feature_frames_.end()) {
        *it = new_frame;
      } else {
        feature_frames_.push_back(new_frame);
      }

      res->detected = true;
      res->message = "Detected " + req->feature;
      res->position = point_odom.point;

      RCLCPP_DEBUG(this->get_logger(), "Feature position in odom: (%.2f, %.2f)",
                   point_odom.point.x, point_odom.point.y);
    } catch (const tf2::TransformException &ex) {
      res->message = "TF error: " + std::string(ex.what());
      RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
    }
  }

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
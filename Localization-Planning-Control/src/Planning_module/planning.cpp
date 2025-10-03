#include <fstream>
#include <sstream>
#include <vector>
#include <cmath>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "std_msgs/msg/float32.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include <chrono>
#include <functional>
 
class PlanningNode : public rclcpp::Node
{
public:
  PlanningNode()
  : Node("planning_node")
  {
    // Declare track parameter (default to "lvms")
    std::string track = this->declare_parameter<std::string>("track", "lvms");
    
    // Set datum and map file based on track
    if (track == "lvms") {
      lat_datum_ = 36.27117541;
      lon_datum_ = -115.00764474;
      hgt_datum_ = 0.0;
      map_file_ = "map/lvms_center_min_curve.csv";
      default_velocity_ = 23.0; // Play around
 
      RCLCPP_INFO(this->get_logger(), "Using LVMS track: %s", map_file_.c_str());
    }
    else if (track == "monza") {
      lat_datum_ = 45.61898065946136;
      lon_datum_ = 9.28112698795041;
      hgt_datum_ = 0.0;
      map_file_ = "map/monza_mincurv.csv";
      default_velocity_ = 20.0;
      RCLCPP_INFO(this->get_logger(), "Using Monza track: %s", map_file_.c_str());
    }
    else {
      RCLCPP_ERROR(this->get_logger(), "Unknown track: %s. Valid options: lvms, monza", track.c_str());
      throw std::runtime_error("Invalid track parameter");
    }
 
    // Initialize LocalCartesian with the selected datum
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(
      lat_datum_, lon_datum_, hgt_datum_);
 
    // Publishers
    path_pub_ = create_publisher<nav_msgs::msg::Path>(
                  "/planning/front_path/offset_path", 10);
    speed_pub_ = create_publisher<std_msgs::msg::Float32>(
                   "/planning/desired_velocity", 10);
 
    // Load global path from CSV and transform to Local ENU
    load_and_transform_path(map_file_);
 
    // Setup TF2 buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
 
    // Timer to run planner loop at fixed rate (20 Hz)
    timer_ = create_wall_timer(
               std::chrono::milliseconds(50),
               std::bind(&PlanningNode::plan_timer_callback, this));
 
    RCLCPP_INFO(this->get_logger(),
                "Planning Node Initialized with datum: (%.6f, %.6f)",
                lat_datum_, lon_datum_);
  }
 
private:
  struct Waypoint {
    double lat, lon, hgt;
    double x, y, z;           // Local ENU coordinates
    double velocity;
  };
 
  void load_and_transform_path(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open map file: %s", filename.c_str());
      throw std::runtime_error("Failed to open map file");
    }
  
    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) continue;
  
      std::stringstream ss(line);
      std::string val;
      Waypoint wp;
  
      // Column 1: latitude
      if (!std::getline(ss, val, ',')) continue;
      wp.lat = std::stod(val);
  
      // Column 2: longitude
      if (!std::getline(ss, val, ',')) continue;
      wp.lon = std::stod(val);
  
      // Column 3: velocity (optional, use default if not present)
      if (std::getline(ss, val, ',')) {
        try {
          wp.velocity = std::stod(val);
        } catch (...) {
          wp.velocity = default_velocity_;
        }
      } else {
        wp.velocity = default_velocity_;
      }
  
      wp.hgt = 0.0;
  
      // Transform lat/lon to Local ENU coordinates
      local_cartesian_->Forward(wp.lat, wp.lon, wp.hgt, wp.x, wp.y, wp.z);
  
      global_path_.push_back(wp);
    }
  
    file.close();
    
    RCLCPP_INFO(this->get_logger(),
                "Loaded %zu waypoints from %s", global_path_.size(), filename.c_str());
    
    if (global_path_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No waypoints loaded from file!");
      throw std::runtime_error("Empty path");
    }
  }
  
  void plan_timer_callback() {
    // Step 1: Get car pose from TF (world -> base_local)
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_buffer_->lookupTransform("world", "base_local", tf2::TimePointZero);
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *get_clock(), 2000,
                           "Could not get transform world->base_local: %s", ex.what());
      return;
    }
 
    const rclcpp::Time tf_time = tf.header.stamp;
    const double car_x = tf.transform.translation.x;
    const double car_y = tf.transform.translation.y;
 
    if (global_path_.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Path is empty.");
      return;
    }
 
    // Step 2: Find closest point on global path (brute force O(n) search)
    int closest_idx = find_closest_waypoint(car_x, car_y);
    
    // Step 3: Transform the next X meters of path into the car's frame
    publish_local_path_and_speed(closest_idx, tf_time);
  }
 
  int find_closest_waypoint(double car_x, double car_y) {
    int closest_idx = 0;
    double min_distance = std::numeric_limits<double>::infinity();
    
    // Brute force search through entire path (O(n))
    for (size_t i = 0; i < global_path_.size(); ++i) {
      const double dx = global_path_[i].x - car_x;
      const double dy = global_path_[i].y - car_y;
      const double distance = std::sqrt(dx * dx + dy * dy);
      
      if (distance < min_distance) {
        min_distance = distance;
        closest_idx = static_cast<int>(i);
      }
    }
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                         "Closest waypoint: %d (distance: %.2f m)",
                         closest_idx, min_distance);
    
    return closest_idx;
  }
 
  void publish_local_path_and_speed(int closest_idx, const rclcpp::Time& tf_time) {
    // Build local path in base_local frame for the controller
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = tf_time;
    path_msg.header.frame_id = "base_local";
 
    // Publish next HORIZON_POINTS waypoints ahead
    const int last_idx = std::min<int>(
      closest_idx + HORIZON_POINTS,
      static_cast<int>(global_path_.size()) - 1
    );
 
    for (int i = closest_idx; i <= last_idx; ++i) {
      // Create point in world frame
      geometry_msgs::msg::PointStamped wp_world;
      wp_world.header.stamp = tf_time;
      wp_world.header.frame_id = "world";
      wp_world.point.x = global_path_[i].x;
      wp_world.point.y = global_path_[i].y;
      wp_world.point.z = 0.0;
 
      // Transform to car (base_local) frame using TF2
      geometry_msgs::msg::PointStamped wp_car;
      try {
        tf_buffer_->transform(wp_world, wp_car, "base_local", tf2::durationFromSec(0.05));
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                             "Transform waypoint failed: %s", ex.what());
        continue;
      }
 
      // Create pose for this waypoint
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path_msg.header;
      pose.pose.position = wp_car.point;
 
      // Compute orientation by looking at the next waypoint
      if (i + 1 <= last_idx) {
        geometry_msgs::msg::PointStamped wp_next_world;
        wp_next_world.header.stamp = tf_time;
        wp_next_world.header.frame_id = "world";
        wp_next_world.point.x = global_path_[i+1].x;
        wp_next_world.point.y = global_path_[i+1].y;
        wp_next_world.point.z = 0.0;
 
        geometry_msgs::msg::PointStamped wp_next_car;
        try {
          tf_buffer_->transform(wp_next_world, wp_next_car, "base_local", tf2::durationFromSec(0.05));
        } catch (const tf2::TransformException& ex) {
          // Use identity quaternion if transform fails
          pose.pose.orientation.w = 1.0;
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          path_msg.poses.push_back(pose);
          continue;
        }
 
        // Calculate yaw angle from current to next waypoint
        double dx = wp_next_car.point.x - wp_car.point.x;
        double dy = wp_next_car.point.y - wp_car.point.y;
        double yaw = std::atan2(dy, dx);
 
        // Convert yaw to quaternion
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        pose.pose.orientation = tf2::toMsg(q);
      } else {
        // Last point: use identity orientation
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
      }
 
      path_msg.poses.push_back(pose);
    }
 
    // Publish path message
    if (!path_msg.poses.empty()) {
      path_pub_->publish(path_msg);
 
      auto first = path_msg.poses.front().pose.position;
      auto last = path_msg.poses.back().pose.position;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                           "Published path: %zu points, First: (%.2f, %.2f), Last: (%.2f, %.2f)",
                           path_msg.poses.size(), first.x, first.y, last.x, last.y);
    } else {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "No poses transformed; path not published.");
      return;
    }
 
    // Publish desired velocity from closest waypoint
    if (closest_idx >= 0 && closest_idx < static_cast<int>(global_path_.size())) {
      std_msgs::msg::Float32 vel_msg;
      vel_msg.data = static_cast<float>(global_path_[closest_idx].velocity);
      speed_pub_->publish(vel_msg);
      
      RCLCPP_INFO_THROTTLE(this->get_logger(), *get_clock(), 1000,
                           "Desired velocity: %.2f m/s", vel_msg.data);
    }
  }
 
  // Member variables
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
 
  double lat_datum_, lon_datum_, hgt_datum_;
  std::string map_file_;
  double default_velocity_;
  std::vector<Waypoint> global_path_;
 
  std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
 
  // Configuration parameters
  const int HORIZON_POINTS = 30;  // Number of waypoints to publish ahead
};
 
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PlanningNode>());
  rclcpp::shutdown();
  return 0;
}
 
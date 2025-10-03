#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "novatel_oem7_msgs/msg/bestpos.hpp"
#include "novatel_oem7_msgs/msg/bestvel.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include "nav_msgs/msg/odometry.hpp"
 
using std::placeholders::_1;
std::string GPS_pos_topic = "/COM/bestpos";
std::string GPS_vel_topic = "/COM/bestvel";
 
class LocalizationNode : public rclcpp::Node
{
  public:
    LocalizationNode()
    : Node("localization_node")
    {
      // Declare track parameter (default to "lvms")
      std::string track = this->declare_parameter<std::string>("track", "lvms");
      
      // Set datum based on track
      if (track == "lvms") {
        lat_datum = 36.27117541;
        lon_datum = -115.00764474;
        hgt_datum = 0.0;
        RCLCPP_INFO(this->get_logger(), "Using LVMS track datum");
      }
      else if (track == "monza") {
        lat_datum = 45.61898065946136;
        lon_datum = 9.28112698795041;
        hgt_datum = 0.0;
        RCLCPP_INFO(this->get_logger(), "Using Monza track datum");
      }
      else {
        RCLCPP_ERROR(this->get_logger(), "Unknown track: %s. Valid options: lvms, monza", track.c_str());
        throw std::runtime_error("Invalid track parameter");
      }
 
      subscription_ = this->create_subscription<novatel_oem7_msgs::msg::BESTPOS>(
        GPS_pos_topic, 10, std::bind(&LocalizationNode::pos_callback, this, _1));
 
      vel_subscription_ = this->create_subscription<novatel_oem7_msgs::msg::BESTVEL>(
        GPS_vel_topic, 10, std::bind(&LocalizationNode::vel_callback, this, _1));
    
      local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(lat_datum, lon_datum, hgt_datum);
      
      tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
 
      world_frame = this->declare_parameter<std::string>("world_frame", "world");
      base_frame = this->declare_parameter<std::string>("base_frame", "base_local");
      yaw_offset_deg = this->declare_parameter<double>("yaw_offset_deg", 0.0);
 
      odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("/localization/odom", 10);
      
      RCLCPP_INFO(this->get_logger(), "Localization node initialized with datum: (%.6f, %.6f)",
                  lat_datum, lon_datum);
    }
 
  private:
    void pos_callback(const novatel_oem7_msgs::msg::BESTPOS::SharedPtr msg)
    {
        // Store position data
        gps_data_.lat = msg->lat;
        gps_data_.lon = msg->lon;
        gps_data_.pos_valid = true;
        gps_data_.pos_timestamp = this->get_clock()->now();
        
        // Publish if we have both position and velocity
        publish_if_ready();
    }
    
    void vel_callback(const novatel_oem7_msgs::msg::BESTVEL::SharedPtr msg)
    {
        // Store velocity data
        gps_data_.yaw_rad = msg->trk_gnd * M_PI / 180.0;  // Convert degrees to radians
        gps_data_.speed_mps = msg->hor_speed;
        gps_data_.vel_valid = true;
        gps_data_.vel_timestamp = this->get_clock()->now();
        
        RCLCPP_INFO(this->get_logger(), "GPS Velocity: %.3f m/s, Track: %.2f deg",
                    msg->hor_speed, msg->trk_gnd);
        
        // Publish if we have both position and velocity
        publish_if_ready();
    }
    
    void publish_if_ready()
    {
        // Check if we have both valid position and velocity data
        if (!gps_data_.pos_valid || !gps_data_.vel_valid) {
            return;
        }
        
        // Convert lat/lon to local ENU coordinates
        double x, y, z;
        local_cartesian_->Forward(gps_data_.lat, gps_data_.lon, 0.0, x, y, z);
        
        RCLCPP_INFO(this->get_logger(), "Local ENU X: %.3f, Y: %.3f | GPS: (%.6f, %.6f)",
                    x, y, gps_data_.lat, gps_data_.lon);
        
        // Prepare transform message
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->get_clock()->now();
        tf_msg.header.frame_id = world_frame;
        tf_msg.child_frame_id = base_frame;
 
        tf_msg.transform.translation.x = x;
        tf_msg.transform.translation.y = y;
        tf_msg.transform.translation.z = 0.0;
 
        // Convert GPS track angle (clockwise from North) to ROS ENU frame
        // GPS: 0° = North, 90° = East (clockwise)
        // ENU: 0° = East, 90° = North (counter-clockwise)
        // Conversion: yaw_enu = 90° - yaw_gps
        const double yaw = (M_PI_2 - gps_data_.yaw_rad);
        
        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        tf_msg.transform.rotation = tf2::toMsg(q);
 
        RCLCPP_INFO(this->get_logger(), "Publishing TF %s -> %s (%.2f, %.2f), yaw=%.2f deg",
                    world_frame.c_str(), base_frame.c_str(), x, y, yaw * 180.0 / M_PI);
        
        tf_broadcaster->sendTransform(tf_msg);
 
        // Publish odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = tf_msg.header.stamp;
        odom.header.frame_id = world_frame;
        odom.child_frame_id = base_frame;
 
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf_msg.transform.rotation;
 
        // Derive ENU velocity components from speed and yaw
        odom.twist.twist.linear.x = gps_data_.speed_mps * std::cos(yaw);
        odom.twist.twist.linear.y = gps_data_.speed_mps * std::sin(yaw);
        odom.twist.twist.linear.z = 0.0;
 
        odom_pub->publish(odom);
    }
    
    // GPS data structure to sync position and velocity
    struct GPSData {
        bool pos_valid = false;
        bool vel_valid = false;
        double lat, lon;
        double yaw_rad;
        double speed_mps;
        rclcpp::Time pos_timestamp;
        rclcpp::Time vel_timestamp;
    } gps_data_;
    
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTPOS>::SharedPtr subscription_;
    rclcpp::Subscription<novatel_oem7_msgs::msg::BESTVEL>::SharedPtr vel_subscription_;
 
    double lat_datum, lon_datum, hgt_datum;
    
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
    
    std::string world_frame;
    std::string base_frame;
    double yaw_offset_deg;
 
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
};
 
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}
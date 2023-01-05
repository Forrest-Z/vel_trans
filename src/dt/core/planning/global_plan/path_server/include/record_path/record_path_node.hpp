#ifndef PROJECT_RECORD_PATH_NODE_HPP_
#define PROJECT_RECORD_PATH_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/utils.h>

#include "path_server/srv/set_path_name.hpp"
#include <std_srvs/srv/empty.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class RecordPathNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit RecordPathNode(const rclcpp::NodeOptions & options);

  ~RecordPathNode();

private:
  void record_callback();

  void start_record(const std::shared_ptr<path_server::srv::SetPathName::Request> request,
        const std::shared_ptr<path_server::srv::SetPathName::Response> response);

  void stop_record(const std::shared_ptr<path_server::srv::SetPathName::Request> request,
        const std::shared_ptr<path_server::srv::SetPathName::Response> response);

  void record_data(double x, double y, double yaw);

  inline std::string get_time_str() {
    char ch[64];
    time_t t = time(nullptr);
    strftime(ch, sizeof(ch), "%Y_%m_%d_%H_%M_%S", localtime(&t));
    return std::string(ch);
  }

  inline double norm2(double x, double y, double yaw) {
    return x * x + y * y + yaw * yaw;
  }

private:
  /* tf */
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  bool record_flag_, first_record_;

  std::string map_frame_, base_link_frame_;
  std::string odom_topic_, path_file_;
  double distance_interval_;

  std::ofstream output_file_;

  double cache_x_, cache_y_, cache_yaw_;
  nav_msgs::msg::Path path_data_;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::TimerBase::SharedPtr record_timer_;
  rclcpp::Service<path_server::srv::SetPathName>::SharedPtr start_record_server_;
  rclcpp::Service<path_server::srv::SetPathName>::SharedPtr stop_record_server_;
};

#endif //PROJECT_RECORD_PATH_NODE_HPP_

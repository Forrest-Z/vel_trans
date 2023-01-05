#ifndef PATH_GENERATOR_HPP_
#define PATH_GENERATOR_HPP_

#include "can_bus/srv/control_service.hpp"

#include <rclcpp/rclcpp.hpp>

#include <autoware_planning_msgs/msg/path.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/float64.hpp>

#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <cmath>

class PathGenerator : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit PathGenerator(const rclcpp::NodeOptions & options);

private:
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_occupancy_grid_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr sub_engage_;
  rclcpp::Publisher<autoware_planning_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr path_index_pub_;
  rclcpp::Client<can_bus::srv::ControlService>::SharedPtr wash_brush_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr path_index_timer_;
  nav_msgs::msg::Path path_;
  nav_msgs::msg::OccupancyGrid::ConstSharedPtr drivable_area_;

  bool is_path_ready_;
  bool engage_;
  bool clean_start_;
  double max_velocity_;
  double path_rate_;
  double match_threshold_;
  int interpolate_;
  double forward_distance_;
  double backward_distance_;
  int last_closest_path_point_index_;
  std::string target_frame_;

  void onLoopPath(const nav_msgs::msg::Path::ConstSharedPtr msg);
  void onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg);
  void onEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);
  void onTimer();
  void PathIndexOnTimer();
  inline static double norm2(const geometry_msgs::msg::Pose & from, geometry_msgs::msg::Pose & to)
  {
    double delta_x = from.position.x - to.position.x;
    double delta_y = from.position.y - to.position.y;
    return delta_x * delta_x + delta_y * delta_y;
  }

  inline static int min_distance_index(
    const geometry_msgs::msg::Pose & from, nav_msgs::msg::Path & path, double threshold)
  {
    int index = -1;
    double min_value = std::numeric_limits<double>::max();
    double dis = 0;
    double yaw_diff = 0;
    const double max_dis = 3 * 3;
    const double max_yaw_diff = 1;
    double ego_yaw = tf2::getYaw(from.orientation);
    if (ego_yaw < 0) {
      ego_yaw = 2 * M_PI + ego_yaw;
    }
    for (size_t i = 0; i < path.poses.size(); i++) {
      dis = norm2(from, path.poses.at(i).pose);
      if (dis > max_dis) {
        // RCLCPP_INFO(rclcpp::get_logger("Path generator"), "continue i: %d\n", static_cast<int>(i));
        continue;
      }
      double path_yaw = tf2::getYaw(path.poses.at(i).pose.orientation);
      if (path_yaw < 0) {
        path_yaw = 2 * M_PI + path_yaw;
      }
      yaw_diff = std::fabs(ego_yaw - path_yaw);
      if (yaw_diff > M_PI) {
        yaw_diff = 2 * M_PI - yaw_diff;
      }
      if (dis < threshold && yaw_diff < max_yaw_diff) {
        return i;
      }
      if (dis < min_value && yaw_diff < max_yaw_diff) {
        // RCLCPP_INFO(rclcpp::get_logger("Path generator"), "update i: %d\n", static_cast<int>(i));
        min_value = dis;
        index = static_cast<int>(i);
      }
    }
    if (min_value > max_dis) {
      RCLCPP_INFO(rclcpp::get_logger("Path generator"), "can not find the closest path point");
      index = 0;
    }
    return index;
  }

  inline static int positive_modulo(int i, int n) { return (i % n + n) % n; }

  inline static int min_distance_index_inner(
    const geometry_msgs::msg::Pose & from, nav_msgs::msg::Path & path, int from_index, int to_index)
  {
    int index = from_index;
    double min_value = std::numeric_limits<double>::max();
    double dis = 0;
    int end_index = std::min(to_index, static_cast<int>(path.poses.size()));
    for (int i = from_index;
         positive_modulo(i, static_cast<int>(path.poses.size())) + 1 < end_index; i++) {
      auto tmp_index = static_cast<size_t>(positive_modulo(i, static_cast<int>(path.poses.size())));
      dis = norm2(from, path.poses.at(tmp_index).pose);
      if (dis < min_value) {
        min_value = dis;
        index = static_cast<int>(tmp_index);
      }
    }
    return index;
  }
};

#endif

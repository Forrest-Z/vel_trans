#ifndef PROJECT_LOAD_PATH_NODE_HPP_
#define PROJECT_LOAD_PATH_NODE_HPP_

#include "path_server/srv/global_path.hpp"
#include "path_server/srv/set_path_name.hpp"

#include <interpolation/spline_interpolation.hpp>
#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/path.hpp>

#include <tf2/utils.h>

#include <fstream>
#include <string>

class LoadPathNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit LoadPathNode(const rclcpp::NodeOptions & options);

private:
  bool load_path(std::string path);

  void load_path_callback();

  rclcpp::Service<path_server::srv::GlobalPath>::SharedPtr frontend_path_service_;

  void GlobalPathCallback(
    const std::shared_ptr<path_server::srv::GlobalPath::Request>,
    const std::shared_ptr<path_server::srv::GlobalPath::Response> response);

  nav_msgs::msg::Path getInterpolatedPath(
    const nav_msgs::msg::Path path, const double delta_arc_length)
  {
    std::vector<double> tmp_x;
    std::vector<double> tmp_y;
    for (std::size_t i = 0; i < path.poses.size(); i++) {
      if (i > 0) {
        if (
          std::fabs(path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x) < 1e-6 &&
          std::fabs(path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y) < 1e-6) {
          continue;
        }
      }
      tmp_x.push_back(path.poses[i].pose.position.x);
      tmp_y.push_back(path.poses[i].pose.position.y);
    }
    nav_msgs::msg::Path interpolated_path;
    interpolated_path.header.frame_id = map_frame_;
    interpolated_path.header.stamp = get_clock()->now();
    LoadPathNode::interpolate2DPoints(tmp_x, tmp_y, delta_arc_length, &interpolated_path);
    return interpolated_path;
  }

  bool interpolate2DPoints(
    const std::vector<double> & base_x, const std::vector<double> & base_y, const double resolution,
    nav_msgs::msg::Path * interpolated_path)
  {
    if (base_x.empty() || base_y.empty()) {
      return false;
    }
    std::vector<double> base_s = calcEuclidDist(base_x, base_y);
    if (base_s.empty() || base_s.size() == 1) {
      return false;
    }
    std::vector<double> new_s;
    for (double i = 0.0; i < base_s.back() - 1e-6; i += resolution) {
      new_s.push_back(i);
    }

    // spline interpolation
    const std::vector<double> interpolated_x = interpolation::slerp(base_s, base_x, new_s);
    const std::vector<double> interpolated_y = interpolation::slerp(base_s, base_y, new_s);

    for (size_t i = 0; i < interpolated_x.size(); i++) {
      if (std::isnan(interpolated_x[i]) || std::isnan(interpolated_y[i])) {
        return false;
      }
    }
    auto time = get_clock()->now();
    std::vector<geometry_msgs::msg::PoseStamped> new_poses;
    for (size_t i = 0; i < interpolated_x.size(); i++) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = interpolated_x[i];
      pose.pose.position.y = interpolated_y[i];
      pose.header.frame_id = map_frame_;
      pose.header.stamp = time;
      double yaw = 0.0;
      if (i < interpolated_x.size() - 1) {
        yaw = std::atan2(
          (interpolated_y[i + 1] - interpolated_y[i]), (interpolated_x[i + 1] - interpolated_x[i]));
      } else {
        yaw = std::atan2(
          (interpolated_y[i] - interpolated_y[i - 1]), (interpolated_x[i] - interpolated_x[i - 1]));
      }
      tf2::Quaternion myQuaternion;
      myQuaternion.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(myQuaternion);
      new_poses.push_back(pose);
    }
    interpolated_path->poses = new_poses;
    return true;
  }

  /*
   * calculate distance in x-y 2D space
   */
  std::vector<double> calcEuclidDist(const std::vector<double> & x, const std::vector<double> & y)
  {
    if (x.size() != y.size()) {
      std::cerr << "x y vector size should be the same." << std::endl;
    }

    std::vector<double> dist_v;
    dist_v.push_back(0.0);
    for (unsigned int i = 0; i < x.size() - 1; ++i) {
      const double dx = x.at(i + 1) - x.at(i);
      const double dy = y.at(i + 1) - y.at(i);
      dist_v.push_back(dist_v.at(i) + std::hypot(dx, dy));
    }

    return dist_v;
  }

private:
  std::string map_frame_, path_file_;
  double arc_length_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr full_path_pub_;
  nav_msgs::msg::Path record_path_;
};

#endif  // PROJECT_LOAD_PATH_NODE_HPP_

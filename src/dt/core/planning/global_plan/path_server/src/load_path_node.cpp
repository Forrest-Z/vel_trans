
#include <load_path/load_path_node.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

LoadPathNode::LoadPathNode(const rclcpp::NodeOptions & options) : Node("load_path", options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  map_frame_ = declare_parameter("map_frame", "map");
  path_file_ = declare_parameter("path_file", "");
  arc_length_ = declare_parameter("arc_length", 0.2);

  full_path_pub_ =
    this->create_publisher<nav_msgs::msg::Path>("/recorded_path", rclcpp::QoS{1}.transient_local());

  frontend_path_service_ = create_service<path_server::srv::GlobalPath>(
    "frontend_path_service", std::bind(&LoadPathNode::GlobalPathCallback, this, _1, _2));
  load_path(path_file_);
  full_path_pub_->publish(record_path_);
}

void LoadPathNode::GlobalPathCallback(
  const std::shared_ptr<path_server::srv::GlobalPath::Request>,
  const std::shared_ptr<path_server::srv::GlobalPath::Response> response)
{
  // response->lanes = this->lanes;
  response->path = this->record_path_;
}
void LoadPathNode::load_path_callback()
{
  if (record_path_.poses.size() == 0) {
    if (!path_file_.empty()) {
      load_path(path_file_);
    }
  } else {
    full_path_pub_->publish(record_path_);
  }
}

bool LoadPathNode::load_path(std::string path)
{
  if (path.empty()) {
    RCLCPP_WARN(get_logger(), "path_file cannot be empty!");
    return false;
  }
  std::ifstream is(path);
  if (!is) {
    RCLCPP_WARN(get_logger(), "file : %s not exist!", path.c_str());
    return false;
  } else {
    RCLCPP_WARN(get_logger(), "loaded path");
    record_path_.poses.clear();
    record_path_.header.frame_id = map_frame_;
    record_path_.header.stamp = get_clock()->now();
    while (!is.eof()) {
      double x, y, yaw;
      is >> x >> y >> yaw;
      geometry_msgs::msg::PoseStamped poseStamped;
      poseStamped.pose.position.x = x;
      poseStamped.pose.position.y = y;
      poseStamped.pose.position.z = 0;
      record_path_.poses.push_back(poseStamped);
    }
    auto interpolated = LoadPathNode::getInterpolatedPath(record_path_, arc_length_);
    record_path_ = interpolated;
    full_path_pub_->publish(record_path_);
    return true;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(LoadPathNode)

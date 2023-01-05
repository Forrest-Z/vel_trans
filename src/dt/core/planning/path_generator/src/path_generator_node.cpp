#include "path_generator/path_generator_node.hpp"

PathGenerator::PathGenerator(const rclcpp::NodeOptions & node_options)
: Node("path_generator", node_options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  is_path_ready_(false),
  engage_(false),
  clean_start_(false),
  last_closest_path_point_index_(-1)
{
  using std::placeholders::_1;
  interpolate_ = declare_parameter("interpolate_step", 1);
  forward_distance_ = declare_parameter("forward_distance", 50.);
  backward_distance_ = declare_parameter("backward_distance", 1.);
  max_velocity_ = declare_parameter("max_velocity", 5.0);
  path_rate_ = declare_parameter("path_rate", 2.0);
  match_threshold_ = declare_parameter("match_threshold", 0.2);
  target_frame_ = declare_parameter("target_frame", "map");

  sub_path_ = this->create_subscription<nav_msgs::msg::Path>(
    "input/recorded_path", rclcpp::QoS{1}.transient_local(),
    std::bind(&PathGenerator::onLoopPath, this, _1));
  sub_occupancy_grid_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    "input/occupancy_grid", 1, std::bind(&PathGenerator::onOccupancyGrid, this, _1));
  sub_engage_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "input/engage", rclcpp::QoS{1}, std::bind(&PathGenerator::onEngage, this, _1));

  path_pub_ = create_publisher<autoware_planning_msgs::msg::Path>("output/path", rclcpp::QoS(1));
  path_index_pub_ = create_publisher<std_msgs::msg::Float64>("output/path_index", rclcpp::QoS(1));
  wash_brush_client_ = create_client<can_bus::srv::ControlService>("input/wash_brush_server");
  const double dt = 1.0 / path_rate_;
  /* Timer */
  {
    auto timer_callback = std::bind(&PathGenerator::onTimer, this);
    auto period =
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
    timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
      this->get_clock(), period, std::move(timer_callback),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(timer_, nullptr);
  }
  {
    const double path_index_dt = 15;
    auto timer_callback_path_index = std::bind(&PathGenerator::PathIndexOnTimer, this);
    auto period_path_index = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(path_index_dt));
    path_index_timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback_path_index)>>(
      this->get_clock(), period_path_index, std::move(timer_callback_path_index),
      this->get_node_base_interface()->get_context());
    this->get_node_timers_interface()->add_timer(path_index_timer_, nullptr);
  }
}

void PathGenerator::onEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  engage_ = msg->engage;
  clean_start_ = msg->engage;
}

void PathGenerator::onLoopPath(const nav_msgs::msg::Path::ConstSharedPtr msg)
{
  path_ = *msg;
  is_path_ready_ = true;
}

void PathGenerator::onOccupancyGrid(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr msg)
{
  drivable_area_ = msg;
  // drivable_area_.header.frame_id = target_frame_;
  // drivable_area_.info.map_load_time = msg.header.stamp;
  // geometry_msgs::Pose ps;
  // geometry_msgs::TransformStamped transform;
  // try {
  //     transform =
  //     tf_buffer_.lookupTransform("map", "base_link", ros::Time(0),
  //     ros::Duration(5));
  // } catch (tf2::TransformException & ex) {
  //     ROS_WARN_DELAYED_THROTTLE(3.0, "cannot get map to base_link transform.
  //     %s", ex.what()); return ;
  // }
  // tf2::doTransform(msg.info.origin, ps, transform);
  // drivable_area_.info.origin = ps;
}
void PathGenerator::PathIndexOnTimer()
{
  std_msgs::msg::Float64 msg;
  msg.data = last_closest_path_point_index_;
  path_index_pub_->publish(msg);
}

void PathGenerator::onTimer()
{
  if (!is_path_ready_ || !drivable_area_ || !engage_) {
    last_closest_path_point_index_ = -1;
    return;
  }
  // when vehicle to end, then stop compute , the index gap is 4(mean 0.4m), beacause the long of
  // vehicle is about 1m
  if (
    last_closest_path_point_index_ != -1 &&
    last_closest_path_point_index_ + 4 >= static_cast<int>(path_.poses.size())) {
    return;
  }

  geometry_msgs::msg::TransformStamped map_to_baselink;
  try {
    map_to_baselink = tf_buffer_.lookupTransform("map", "base_link", tf2::TimePointZero);
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    return;
  }
  geometry_msgs::msg::Pose pose;
  pose.position.x = map_to_baselink.transform.translation.x;
  pose.position.y = map_to_baselink.transform.translation.y;
  double yaw = tf2::getYaw(map_to_baselink.transform.rotation);
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, yaw);
  pose.orientation = tf2::toMsg(myQuaternion);
  if (last_closest_path_point_index_ == -1) {
    last_closest_path_point_index_ = min_distance_index(pose, path_, match_threshold_);
  } else {
    last_closest_path_point_index_ = min_distance_index_inner(
      pose, path_, last_closest_path_point_index_,
      last_closest_path_point_index_ + interpolate_ * 10);
  }

  // judge is the vehicle at path ending
  if (clean_start_ && ((long)path_.poses.size() - (long)last_closest_path_point_index_ < 12)) {
    clean_start_ = false;
    auto request_brush = std::make_shared<can_bus::srv::ControlService::Request>();
    request_brush->type = "brush";
    request_brush->start = false;
    wash_brush_client_->async_send_request(request_brush);

    auto request_wash = std::make_shared<can_bus::srv::ControlService::Request>();
    request_wash->type = "wash";
    request_wash->start = false;
    wash_brush_client_->async_send_request(request_wash);
    // RCLCPP_INFO(
    //   this->get_logger(), "gonging to shut down wash and brush//////!!!!!!!!!!!! %ld %ld",
    //   (long)last_closest_path_point_index_, (long)path_.poses.size());
  }

  autoware_planning_msgs::msg::Path path;

  auto path_begin_index = static_cast<size_t>(last_closest_path_point_index_);
  auto path_end_index = static_cast<size_t>(last_closest_path_point_index_);

  {
    double acc_dis = 0;
    for (size_t i = last_closest_path_point_index_; i > 0 && acc_dis < backward_distance_; i--) {
      double delta_s = std::sqrt(norm2(path_.poses.at(i).pose, path_.poses.at(i + 1).pose));
      acc_dis += delta_s;
      if (acc_dis < backward_distance_) {
        path_begin_index = i;
      }
    }
  }

  {
    double acc_dis = 0;
    for (size_t i = last_closest_path_point_index_ + 1;
         i < path_.poses.size() && acc_dis < forward_distance_; i++) {
      double delta_s = std::sqrt(norm2(path_.poses.at(i - 1).pose, path_.poses.at(i).pose));
      acc_dis += delta_s;
      if (acc_dis < forward_distance_) {
        path_end_index = i;
      }
    }
  }

  for (size_t i = path_begin_index; i < path_.poses.size() && i < path_end_index;
       i += interpolate_) {
    autoware_planning_msgs::msg::PathPoint point;
    point.pose = path_.poses.at(i).pose;
    point.twist.linear.x = max_velocity_;
    path.points.push_back(point);
  }
  path.header.frame_id = path_.header.frame_id;
  path.header.stamp = get_clock()->now();
  path.drivable_area = *drivable_area_;
  path_pub_->publish(path);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(PathGenerator)

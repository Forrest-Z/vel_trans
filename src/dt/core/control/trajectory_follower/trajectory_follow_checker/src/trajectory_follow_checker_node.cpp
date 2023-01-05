#include "trajectory_follow_checker/trajectory_follow_checker_node.hpp"

using autoware_utils::rad2deg;

double calculateSquaredDistance(
  const geometry_msgs::msg::Pose & a, const geometry_msgs::msg::Pose & b)
{
  const double dx = a.position.x - b.position.x;
  const double dy = a.position.y - b.position.y;
  return dx * dx + dy * dy;
}


size_t findNearestIndexWithIn(
  const std::vector<autoware_planning_msgs::msg::TrajectoryPoint> & points, const geometry_msgs::msg::Pose & pose, size_t inner)
{
  int nearest_idx = 0;
  double min_dist = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < inner && i < points.size(); i++) {
    const double dist = calculateSquaredDistance(points.at(i).pose, pose);
    if (dist < min_dist) {
      min_dist = dist;
      nearest_idx = i;
    }
  }
  return nearest_idx;
}

TrajectoryFollowCheckerNode::TrajectoryFollowCheckerNode(const rclcpp::NodeOptions & options)
: Node("trajectory_follow_checker_node", options)
{
  using std::placeholders::_1;

  // Node Parameter
  node_param_.update_rate = declare_parameter("update_rate", 10.0);

  // Vehicle Info
  const auto vehicle_info = vehicle_info_util::VehicleInfoUtil(*this).getVehicleInfo();
  vehicle_length_m_ = vehicle_info.vehicle_length_m;

  param_.max_lateral_deviation = declare_parameter("max_lateral_deviation", 1.0);
  param_.max_longitudinal_deviation = declare_parameter("max_longitudinal_deviation", 1.0);
  param_.max_yaw_deviation_deg = declare_parameter("max_yaw_deviation_deg", 30.0);

  // Subscriber
  sub_twist_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
    "~/input/twist", 1, std::bind(&TrajectoryFollowCheckerNode::onTwist, this, _1));
  sub_reference_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/reference_trajectory", 1,
    std::bind(&TrajectoryFollowCheckerNode::onReferenceTrajectory, this, _1));
  sub_predicted_trajectory_ = this->create_subscription<autoware_planning_msgs::msg::Trajectory>(
    "~/input/predicted_trajectory", 1,
    std::bind(&TrajectoryFollowCheckerNode::onPredictedTrajectory, this, _1));
  // Publisher
  engage_pub_ = this->create_publisher<autoware_vehicle_msgs::msg::Engage>("/vehicle/engage", rclcpp::QoS(1));

  // Diagnostic Updater
  updater_.setHardwareID("trajectory_follow_checker");

  updater_.add("trajectory_deviation", this, &TrajectoryFollowCheckerNode::checkTrajectoryDeviation);

  // Wait for first self pose
  self_pose_listener_.waitForFirstPose();

  // Timer
  double delta_time = 1.0 / static_cast<double>(node_param_.update_rate);
  auto timer_callback_ = std::bind(&TrajectoryFollowCheckerNode::onTimer, this);
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(delta_time));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback_)>>(
    this->get_clock(), period_ns, std::move(timer_callback_),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
}

void TrajectoryFollowCheckerNode::onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg)
{
  current_twist_ = msg;
}

void TrajectoryFollowCheckerNode::onReferenceTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  reference_trajectory_ = msg;
}

void TrajectoryFollowCheckerNode::onPredictedTrajectory(
  const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  predicted_trajectory_ = msg;
}


void TrajectoryFollowCheckerNode::onTimer()
{
  current_pose_ = self_pose_listener_.getCurrentPose();

  if (!isDataReady()) {
    return;
  }

  if (isDataTimeout()) {
    return;
  }
  output_.trajectory_deviation = 
    calcTrajectoryDeviation(*reference_trajectory_, current_pose_->pose);

  updater_.force_update();

  {
    const auto & deviation = output_.trajectory_deviation;
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/lateral", deviation.lateral);
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw", deviation.yaw);
    debug_publisher_.publish<autoware_debug_msgs::msg::Float64Stamped>(
      "deviation/yaw_deg", rad2deg(deviation.yaw));
  }
}

bool TrajectoryFollowCheckerNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_pose...");
    return false;
  }

  if (!current_twist_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "waiting for current_twist msg...");
    return false;
  }

  if (!reference_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for reference_trajectory msg...");
    return false;
  }

  if (!predicted_trajectory_) {
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000, "waiting for predicted_trajectory msg...");
    return false;
  }

  return true;
}

bool TrajectoryFollowCheckerNode::isDataTimeout()
{
  const auto now = this->now();

  constexpr double th_pose_timeout = 1.0;
  const auto pose_time_diff = rclcpp::Time(current_pose_->header.stamp) - now;
  if (pose_time_diff.seconds() > th_pose_timeout) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "pose is timeout...");
    return true;
  }

  return false;
}

PoseDeviation TrajectoryFollowCheckerNode::calcTrajectoryDeviation(
  const autoware_planning_msgs::msg::Trajectory & trajectory, const geometry_msgs::msg::Pose & pose)
{
  // TODO(jiheng): find better method to replace hardcode number, this distace should be same as fixed distance for mpt
  const auto nearest_idx = findNearestIndexWithIn(trajectory.points, pose, 50);
  return autoware_utils::calcPoseDeviation(trajectory.points.at(nearest_idx).pose, pose);
}

void TrajectoryFollowCheckerNode::checkTrajectoryDeviation(
  diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  int8_t level = DiagStatus::OK;

  if (std::abs(output_.trajectory_deviation.lateral) >= param_.max_lateral_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(output_.trajectory_deviation.longitudinal) >= param_.max_longitudinal_deviation) {
    level = DiagStatus::ERROR;
  }

  if (std::abs(rad2deg(output_.trajectory_deviation.yaw)) >= param_.max_yaw_deviation_deg) {
    level = DiagStatus::ERROR;
  }

  std::string msg = "OK";
  if (level == DiagStatus::ERROR) {
    msg = "trajectory deviation is too large";
    // TODO(jiheng): remove publisher when we can 
    // use system monitor to detect ERROR state
    // RCLCPP_INFO(get_logger(), "Trajectory deviation too big, disengage");
    // autoware_vehicle_msgs::msg::Engage engage;
    // engage.stamp = this->get_clock()->now();
    // engage.engage = false;
    // engage_pub_->publish(engage);
  }

  stat.addf("max lateral deviation", "%.3f", param_.max_lateral_deviation);
  stat.addf("lateral deviation", "%.3f", output_.trajectory_deviation.lateral);

  stat.addf("max longitudinal deviation", "%.3f", param_.max_longitudinal_deviation);
  stat.addf("longitudinal deviation", "%.3f", output_.trajectory_deviation.longitudinal);

  stat.addf("max yaw deviation", "%.3f", param_.max_yaw_deviation_deg);
  stat.addf("yaw deviation", "%.3f", rad2deg(output_.trajectory_deviation.yaw));

  stat.summary(level, msg);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TrajectoryFollowCheckerNode)

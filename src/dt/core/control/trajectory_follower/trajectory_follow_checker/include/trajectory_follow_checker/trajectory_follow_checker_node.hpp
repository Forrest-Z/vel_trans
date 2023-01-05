#include <rclcpp/rclcpp.hpp>

#include <autoware_utils/ros/self_pose_listener.hpp>
#include <autoware_utils/geometry/pose_deviation.hpp>
#include <autoware_utils/ros/debug_publisher.hpp>
#include <autoware_utils/math/unit_conversion.hpp>

#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <vehicle_info_util/vehicle_info_util.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

using autoware_utils::PoseDeviation;

struct Output
{
  PoseDeviation trajectory_deviation{};
};

struct Param
{
  double footprint_margin_scale;
  double resample_interval;
  double max_deceleration;
  double delay_time;
  double max_lateral_deviation;
  double max_longitudinal_deviation;
  double max_yaw_deviation_deg;
};

struct NodeParam
{
  double update_rate;
  bool visualize_lanelet;
};

class TrajectoryFollowCheckerNode : public rclcpp::Node
{
public:
  explicit TrajectoryFollowCheckerNode(const rclcpp::NodeOptions & options);

private:
  autoware_utils::SelfPoseListener self_pose_listener_{this};
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr sub_twist_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_reference_trajectory_;
  rclcpp::Subscription<autoware_planning_msgs::msg::Trajectory>::SharedPtr
    sub_predicted_trajectory_;

  geometry_msgs::msg::PoseStamped::ConstSharedPtr current_pose_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr current_twist_; 

  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr reference_trajectory_;
  autoware_planning_msgs::msg::Trajectory::ConstSharedPtr predicted_trajectory_;

  void onTwist(const geometry_msgs::msg::TwistStamped::ConstSharedPtr msg);
  void onReferenceTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  void onPredictedTrajectory(const autoware_planning_msgs::msg::Trajectory::ConstSharedPtr msg);
  
  Output output_{};

  static PoseDeviation calcTrajectoryDeviation(
    const autoware_planning_msgs::msg::Trajectory & trajectory,
    const geometry_msgs::msg::Pose & pose);

  // Publisher
  autoware_utils::DebugPublisher debug_publisher_{this, "~/debug"};
  rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer();
  bool isDataReady();
  bool isDataTimeout();

  NodeParam node_param_;
  Param param_;
  double vehicle_length_m_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_{this};

  void checkTrajectoryDeviation(diagnostic_updater::DiagnosticStatusWrapper & stat);
};

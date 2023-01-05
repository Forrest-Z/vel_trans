#ifndef DATA_TRANSFORMER_NODE_HPP_
#define DATA_TRANSFORMER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <autoware_vehicle_msgs/msg/control_mode.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>
#include <autoware_vehicle_msgs/msg/steering.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64.hpp>

class DataTransformerNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit DataTransformerNode(const rclcpp::NodeOptions & options);

private:
  int rate_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string simulation_frame_id_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr sub_engage_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_steer_;

  rclcpp::Publisher<autoware_vehicle_msgs::msg::Steering>::SharedPtr pub_steer_;
  rclcpp::Publisher<autoware_vehicle_msgs::msg::ControlMode>::SharedPtr pub_control_mode_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr
    pub_twist_;  //!< @brief topic ros publisher for current twist

  geometry_msgs::msg::TwistStamped::SharedPtr twist_;
  autoware_vehicle_msgs::msg::ControlMode::SharedPtr control_mode_;
  autoware_vehicle_msgs::msg::Steering::SharedPtr steer_;

  void callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg);

  void callbackEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);

  void callbackSteer(const std_msgs::msg::Float64::ConstSharedPtr msg);

  void onTimer();
};

#endif

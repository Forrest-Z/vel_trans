#include "cmdvel_convertor/cmdvel_convertor_node.hpp"


CmdvelConvertorNode::CmdvelConvertorNode(const rclcpp::NodeOptions & node_options)
: Node("cmdvel_convertor_node", node_options)
{
  using std::placeholders::_1;

  vehicle_cmd_sub_ = create_subscription<autoware_vehicle_msgs::msg::VehicleCommand>(
    "input/vehicle_cmd", rclcpp::QoS{1},
    std::bind(&CmdvelConvertorNode::callbackVehicleCmd, this, std::placeholders::_1));

  cmd_vel_pub_ =
    create_publisher<geometry_msgs::msg::Twist>("output/cmd_vel", rclcpp::QoS{1});
}

void CmdvelConvertorNode::callbackVehicleCmd(
  const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg)
{
  geometry_msgs::msg::Twist twist;
  twist.linear.x = msg->control.velocity;
  twist.angular.z = msg->control.steering_angle;

  cmd_vel_pub_->publish(twist);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CmdvelConvertorNode)

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <autoware_vehicle_msgs/msg/vehicle_command.hpp>

class CmdvelConvertorNode : public rclcpp::Node
{

public:

  explicit CmdvelConvertorNode(const rclcpp::NodeOptions & options);

private:

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr
    cmd_vel_pub_; 

  rclcpp::Subscription<autoware_vehicle_msgs::msg::VehicleCommand>::SharedPtr vehicle_cmd_sub_;

  /**
   * @brief set current_vehicle_cmd_ptr_ with received message
   */
  void callbackVehicleCmd(const autoware_vehicle_msgs::msg::VehicleCommand::ConstSharedPtr msg);

};


#include "vel_trans/vel_trans.hpp"

//namespace dt_tool {

VelTrans::VelTrans(const rclcpp::NodeOptions & node_options)
: Node("vel_trans_node", node_options)
{
  RCLCPP_INFO(this->get_logger(),"vel_trans start !!!!!!!!!!!!!!!!!!!!!");

  using std::placeholders::_1;
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/autocar/cmd_vel", 10);
  twist_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("/autocar_ros/cmd_vel", 10, std::bind(&VelTrans::handle_twist_msg, this, _1));

}

VelTrans::~VelTrans(){
}

void VelTrans::handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg){
  geometry_msgs::msg::Twist twist;
  twist = *msg;
  if(twist.linear.x < 0){
    twist.angular.z = -1 * twist.angular.z;
  }
  twist_pub_->publish(twist); 
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(VelTrans)

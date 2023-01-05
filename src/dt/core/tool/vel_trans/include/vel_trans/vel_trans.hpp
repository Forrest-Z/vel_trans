#ifndef VEL_TRANS_NODE_H
#define VEL_TRANS_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>


class VelTrans : public rclcpp::Node {

public:
   
  explicit VelTrans(const rclcpp::NodeOptions & options);

  ~VelTrans();
private:
  void handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
};


#endif // VEL_TRANS_NODE_H

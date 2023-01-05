#ifndef CAN_BUS_PMDRIVER_HPP_
#define CAN_BUS_PMDRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include "chasu/srv/control_service.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <climits>
#include <cmath>

//#include <std_msgs/Float64.h>
#include "chasu/msg/can_bus_message.hpp"
#include "chasu/msg/cmd.hpp"
#include "chasu/srv/can_bus_service.hpp"

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include <autoware_vehicle_msgs/msg/engage.hpp>
static double speedtable[11] = {
  0, 0.06, 0.149, 0.237, 0.324, 0.410, 0.499, 0.585, 0.673, 0.760, 0.842};
static double pushtable[11] = {6.0,6.0,7.45,7.90,8.1,8.2,8.32,8.36,8.41,8.44,8.42};
const int CANBUS_NODETYPE_ECU = 1;
union HEX {
    float num;
    unsigned char hex_num[4];
};

class CanBusPmdriver : public rclcpp::Node
{
public:
  explicit CanBusPmdriver(const rclcpp::NodeOptions & options);

private:
  void handle_canbus_msg(const chasu::msg::CanBusMessage::ConstSharedPtr msg);

  void handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void parse_msg();


private:
  geometry_msgs::msg::Twist twist_cache_;
  double twist_timeout_;
  std::string odom_frame_, base_frame_;
  int rate_;
  double sync_timeout_;
  double max_speed_;
  double current_vel_l, current_vel_r, last_a, last_v;
  float current_angle;
  double wheelbase, wheeltread;
  double speed_cor, angle_cor;
  bool publish_tf_;
  bool first_send_odom_flag_;
  double accumulation_x_, accumulation_y_, accumulation_yaw_;
  rclcpp::TimerBase::SharedPtr odom_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_send_odom_time_;

  rclcpp::Subscription<chasu::msg::CanBusMessage>::SharedPtr canbus_msg_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Client<chasu::srv::CanBusService>::SharedPtr canbus_client_;
  rclcpp::Publisher<chasu::msg::Cmd>::SharedPtr v_receive_pub_, a_receive_pub_, v_send_pub_,
    a_send_pub_, vel_send_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;

};

#endif  // CAN_BUS_PMDRIVER_HPP_
#ifndef CAN_BUS_PMDRIVER_HPP_
#define CAN_BUS_PMDRIVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include "can_bus/srv/control_service.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <climits>
#include <cmath>

//#include <std_msgs/Float64.h>
#include "can_bus/msg/can_bus_message.hpp"
#include "can_bus/msg/cmd.hpp"
#include "can_bus/srv/can_bus_service.hpp"

#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/header.hpp>
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include <autoware_vehicle_msgs/msg/engage.hpp>
using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
const int CANBUS_NODETYPE_ECU = 1;
const int CANBUS_NODETYPE_TCU = 2;
const int CANBUS_NODETYPE_ST = 3;
const int CANBUS_NODETYPE_BR = 4;
static double speedtable[23] = {
  0,
  17.473391812865497,
  51.55913978494624,
  95.21657894736842,
  121.53465346534654,
  160.8355614973262,
  186.2227722772277,
  226.36643664366437,
  250.82322607260727,
  288.8070317235805,
  313.86564675885063,
  351.9254901960784,
  377.75856653340946,
  430.7647058823529,
  497.38613861386136,
  597.1518637157833,
  664.9620297029703,
  765.5500915750915,
  832.2838604248774,
  932.7054015636105,
  998.8645021645021,
  1099.225,
  1165.9901515151514};
static double pushtable[23] = {0,    3.0,  5.0,  8.0,  10.0, 13.0, 15.0, 18.0,
                               20.0, 23.0, 25.0, 28.0, 30.0, 33.0, 35.0, 38.0,
                               40.0, 43.0, 45.0, 48.0, 50.0, 53.0, 55.0};
static double KA = 33.4374293;
static double KB = -672.7682996;

class CanBusPmdriver : public rclcpp::Node
{
public:
  explicit CanBusPmdriver(const rclcpp::NodeOptions & options);

private:
  void handle_canbus_msg(const can_bus::msg::CanBusMessage::ConstSharedPtr msg);

  void handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg);

  void onEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg);

  void parse_msg();

  void brush_msg();

  void wash_msg();

  void brake_msg();

  void guard_msg();

  void diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg);

  bool findDiagStatus(const std::string & name, DiagStatus & status);

private:
  geometry_msgs::msg::Twist twist_cache_;
  double twist_timeout_;
  std::string odom_frame_, base_frame_;
  int rate_, test_model, angle_acel_, max_angle_, v_flag, brake_value_;
  double sync_timeout_;
  float max_speed_;
  float current_vel, current_angle, last_a, last_v;
  float wheelbase;
  float speed_cor, angle_cor;
  bool publish_tf_, model_, check_model_, brake_model_;
  bool first_send_odom_flag_;
  double accumulation_x_, accumulation_y_, accumulation_yaw_;
  int brush_start_, wash_start_, brake_start_, guard_start_;
  diagnostic_msgs::msg::DiagnosticArray array_;
  autoware_vehicle_msgs::msg::Engage engage_msg;
  DiagStatus status_sonar, status_laser;
  rclcpp::TimerBase::SharedPtr odom_timer_, brush_timer_, wash_timer_, brake_timer_, guard_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_send_odom_time_;
  rclcpp::Time current_vel_time, current_angle_time;

  rclcpp::Subscription<can_bus::msg::CanBusMessage>::SharedPtr canbus_msg_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
  rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr sonar_sub_;
  rclcpp::Subscription<autoware_vehicle_msgs::msg::Engage>::SharedPtr sub_engage_;
  rclcpp::Client<can_bus::srv::CanBusService>::SharedPtr canbus_client_;
  rclcpp::Publisher<can_bus::msg::Cmd>::SharedPtr v_receive_pub_, a_receive_pub_, v_send_pub_,
    a_send_pub_, vel_send_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr bms_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr steer_pub_;
  rclcpp::Service<can_bus::srv::ControlService>::SharedPtr control_service_;

  void controlService(
  const std::shared_ptr<can_bus::srv::ControlService::Request> req,
  const std::shared_ptr<can_bus::srv::ControlService::Response> );
};

#endif  // CAN_BUS_PMDRIVER_HPP_
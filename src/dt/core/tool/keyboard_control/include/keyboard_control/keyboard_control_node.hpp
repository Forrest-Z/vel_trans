#ifndef KEYBOARD_CONTROL_NODE_H
#define KEYBOARD_CONTROL_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <fcntl.h>
#include <dirent.h>
#include <linux/input.h>
#include <sys/stat.h>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
//#include "std_msgs/Bool.h"
#define KEYBOARD_UP     103
#define KEYBOARD_DOWN   108
#define KEYBOARD_LEFT   105
#define KEYBOARD_RIGHT  106
#define KEYBOARD_1      2
#define KEYBOARD_2      3
#define KEYBOARD_3      4
#define KEYBOARD_4      5
#define KEYBOARD_7      8
#define KEYBOARD_8      9
#define KEYBOARD_9      10
#define KEYBOARD_0      11
#define BRUSH_START     18  //brush_start_button “E”
#define BRUSH_STOP      19  //brush_stop_button “R”
#define WASH_START     20  //brush_start_button “E”
#define WASH_STOP      21  //brush_stop_button “R”
#define EMERGENCY_STOP           33  //brush_stop_button “F”
#define EMERGENCY_GO           34  //brush_stop_button “G”


//namespace dt_tool {

class KeyboardControl : public rclcpp::Node {

public:
   
  explicit KeyboardControl(const rclcpp::NodeOptions & options);

  ~KeyboardControl();
  void brushstart();
  void brushstop();
  void washstart();
  void washstop();
  void emgstop();
  void emgcontinue();

private:

  std::shared_ptr<rclcpp::AsyncParametersClient> parameters_client;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  rclcpp::TimerBase::SharedPtr twist_pub_timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr brush_pub_,stop_pub_;

  int fd_;
  int linear_state_, angular_state_;
  struct input_event ev_;
  std::string port_name_;
  double rate_;
  double linear_scale_, angular_scale_;
  double linear_min_, linear_max_, linear_step_;
  double angular_min_, angular_max_, angular_step_;
  double smooth_step_, vel_now_, vel_last_; //平滑每档速度
  bool send_flag_;
  bool smooth_switch_; //平滑开关
  bool brush_switch_; //清扫刷控制开关
  bool wash_switch_;

  bool init();
  void parseKeyboard();
  void twistCallback();
  void buttonTwistCheck(int value, int& state, int down, int up);
  void buttonScaleCheck(int value, double& scale, double step, double limit);


};


#endif // KEYBOARD_CONTROL_NODE_H

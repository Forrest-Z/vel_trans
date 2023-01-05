#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "brake_server/srv/control_service.hpp"
using namespace std;



class BrakeInterfaceNode : public rclcpp::Node
{
public:



  /**
   * @brief constructor
   */
  explicit BrakeInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // Set up ROS.
  string port_;
  int baud_, num, brake_value_, brake_start_, init_value_, retrace_value_;
  serial::Serial serial_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<brake_server::srv::ControlService>::SharedPtr control_service_;
  void controlService(
  const std::shared_ptr<brake_server::srv::ControlService::Request> req,
  const std::shared_ptr<brake_server::srv::ControlService::Response> );
  void setmode();
  void setpose(int value_);
  int getpose();
  void init();
  void on_timer();
};








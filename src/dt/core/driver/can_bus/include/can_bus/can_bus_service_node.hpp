#ifndef CAN_BUS_SERVICE_NODE_HPP_
#define CAN_BUS_SERVICE_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include "can_bus/ECanVci.h"
#include "can_bus/srv/control_service.hpp"
#include "can_bus/msg/can_bus_message.hpp"
#include "can_bus/msg/cmd.hpp"
#include "can_bus/srv/can_bus_service.hpp"

#include <geometry_msgs/msg/twist.hpp>

#include <boost/asio.hpp>
#include <boost/asio/buffer.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/bind.hpp>
#include <boost/system/error_code.hpp>
#include <boost/system/system_error.hpp>
#include <boost/thread.hpp>
#include <vector>
// #include <stdio.h>
// #include <string.h>
// #include <unistd.h>
// #include <sys/types.h>
// #include <sys/stat.h>
// #include <fcntl.h>
// #include <pthread.h>
// #include <stdlib.h>

#define RX_WAIT_TIME 0
#define RX_BUFF_SIZE 1
using namespace std;

class CanBusServiceNode : public rclcpp::Node
{
public:
  explicit CanBusServiceNode(const rclcpp::NodeOptions & options);

private:
  bool init();
  void parse_msg1();
  void parse_msg2();
  bool trans_msg(int dev, int num, int ID, int externflag, vector <unsigned char> & data);
  bool opendev();
  void closedev();
  void startdev();
  bool canbus_service(
    const std::shared_ptr<can_bus::srv::CanBusService::Request> req,
    std::shared_ptr<can_bus::srv::CanBusService::Response>);
  void controlService(
    const std::shared_ptr<can_bus::srv::ControlService::Request> req,
    const std::shared_ptr<can_bus::srv::ControlService::Response> );
  bool verify_frame(CAN_OBJ * can, can_bus::msg::CanBusMessage * msg);


private:
  int gDevType1_;
  int gDevIdx1_;
  int gChMask1_;
  int gBaud1_;
  int gTxType1_;
  int gDevType2_;
  int gDevIdx2_;
  int gChMask2_;
  int gBaud2_;
  int gTxType2_;
  int parse_rate_;
  int brush_state;
  int light_limit;
  int light_state;
  int wash_bit;
  int guard_bit;
  bool enable_debug_;

  rclcpp::Publisher<can_bus::msg::CanBusMessage>::SharedPtr canbus_msg_pub1_, canbus_msg_pub2_;
  rclcpp::Publisher<can_bus::msg::Cmd>::SharedPtr qianjin_pub, qianjin_rec, jiaodu_pub, jiaodu_rec,
    brush_pub;

  rclcpp::Service<can_bus::srv::CanBusService>::SharedPtr canbus_msg_service_;
  rclcpp::Service<can_bus::srv::ControlService>::SharedPtr control_service_;
};

#endif

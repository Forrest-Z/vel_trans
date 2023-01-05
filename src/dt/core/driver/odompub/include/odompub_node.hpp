#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <vector>
#include <stdint.h>
using namespace std;
union HEX {
    uint32_t num;
    unsigned char hex_num[4];
};
union HEX2 {
    int32_t num;
    unsigned char hex_num[4];
};
class OdomPubNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit OdomPubNode(const rclcpp::NodeOptions & options);


  ~OdomPubNode();



private:
  // Set up ROS.
  string port_;
  uint32_t odom_cache_;
  int baud_, rate;
  serial::Serial serial_;
  uint8_t payload[36] = {0xaa, 0x44, 0x13, 0x14, 0xdd, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
                         0x00, 0x00, 0x00, 0x00};
  rclcpp::TimerBase::SharedPtr pub_timer_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  void pubTimer();
  void handle_odom_msg(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  uint32_t crc32(const uint8_t *data, const int size);
};

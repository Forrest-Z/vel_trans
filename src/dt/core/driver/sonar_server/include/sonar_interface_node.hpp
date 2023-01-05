#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>

#include <algorithm>
#include <vector>
using namespace std;

class SonarInterfaceNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit SonarInterfaceNode(const rclcpp::NodeOptions & options);

  ~SonarInterfaceNode();


private:
  // Set up ROS.
  string port_;
  int baud_, x_num, y_num, z_num,smooth_, rate;
  float front_back_length_, left_right_length_, skew_length_;
  serial::Serial serial_;
  vector<vector<float>> v_list;
  vector<float> v;
  uint8_t payload[3];
  float max_sonar_dist_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_x, publisher_y, publisher_z;
  rclcpp::TimerBase::SharedPtr timer_, pub_timer_;
  
  void readTimer();
  void pubTimer();
};

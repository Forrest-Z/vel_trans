#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include "test_server/srv/control_service.hpp"
using namespace std;



class TestInterfaceNode : public rclcpp::Node
{
public:



  /**
   * @brief constructor
   */
  explicit TestInterfaceNode(const rclcpp::NodeOptions & options);
  ~TestInterfaceNode();
private:
  // Set up ROS.
  rclcpp::TimerBase::SharedPtr timer_;
  void readTimer();

};








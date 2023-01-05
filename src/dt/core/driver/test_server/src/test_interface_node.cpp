#include <test_interface_node.hpp>

TestInterfaceNode::TestInterfaceNode(const rclcpp::NodeOptions & node_options) 
: Node("brake_interface", node_options)
{
    typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  timer_ =
    create_wall_timer(second_type(1.0 / 50), std::bind(&TestInterfaceNode::readTimer, this));
}
TestInterfaceNode::~TestInterfaceNode() { }
void TestInterfaceNode::readTimer()
{
  RCLCPP_INFO(rclcpp::get_logger("!!"), "!!!!!!!!!");
//   rclcpp::Rate r(1);
//   r.sleep();
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(TestInterfaceNode)

#include <fixtrans_node.hpp>

FixTransNode::FixTransNode(const rclcpp::NodeOptions & node_options)
: Node("fixtrans_node", node_options)
{
  using std::placeholders::_1;
  odom_cache_ = 0;
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odom", 10);
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("/fixposition/odometry", 10, std::bind(&FixTransNode::handle_odom_msg, this, _1));
}

FixTransNode::~FixTransNode() { this->serial_.close(); }


void FixTransNode::handle_odom_msg(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  odom_cache_ = *msg;
	double L = 34.33651212;
	double B = 108.780483213;
	double H = 347.564;   
  Eigen::Vector3d topocentricOrigin(L, B, H);
	Eigen::Matrix4d wolrd2localMatrix;
	FixTransNode::CalEcef2Enu(topocentricOrigin, wolrd2localMatrix);
  Eigen::Vector4d xyz(odom_cache_.pose.pose.position.x, odom_cache_.pose.pose.position.y, odom_cache_.pose.pose.position.z, 1);
  Eigen::Vector4d enu = wolrd2localMatrix * xyz;
  nav_msgs::msg::Odometry odom_msg;
  odom_msg.header.frame_id = 'odom';
  odom_msg.child_frame_id = 'base_link';
  odom_msg.header.stamp = rclcpp::Node::now();;
  odom_msg.pose.pose.position.x = enu[0];
  odom_msg.pose.pose.position.y = enu[1];
  odom_msg.pose.pose.position.z = enu[2];
  odom_pub_->publish(odom_msg);
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(FixTransNode)

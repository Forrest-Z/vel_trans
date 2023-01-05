#include <odompub_node.hpp>

OdomPubNode::OdomPubNode(const rclcpp::NodeOptions & node_options)
: Node("odompub_node", node_options)
{
  odom_cache_ = 700;
  using std::placeholders::_1;
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  // read parameter for serial
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 115200);
  rate = declare_parameter("rate", 50);
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 5000, 0, 5000, 0);
  this->serial_.open();
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&OdomPubNode::handle_odom_msg, this, _1));
  pub_timer_ =
    create_wall_timer(second_type(1.0 / rate), std::bind(&OdomPubNode::pubTimer, this));
}

OdomPubNode::~OdomPubNode() { this->serial_.close(); }

void OdomPubNode::pubTimer()
{
  union HEX2 data_send;
  data_send.num = odom_cache_;
  payload[12] = data_send.hex_num[0];
  payload[13] = data_send.hex_num[1];
  payload[14] = data_send.hex_num[2];
  payload[15] = data_send.hex_num[3];
  union HEX data_check;
  data_check.num = crc32(payload, 32);
  payload[32] = data_check.hex_num[0];
  payload[33] = data_check.hex_num[1];
  payload[34] = data_check.hex_num[2];
  payload[35] = data_check.hex_num[3];
  this->serial_.flushInput();
  this->serial_.write(payload, sizeof(payload));
  // RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!\n");
}
void OdomPubNode::handle_odom_msg(const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
  odom_cache_ = int32_t(msg->twist.twist.linear.x * 1000);   
}
uint32_t OdomPubNode::crc32(const uint8_t *data, const int size) {
    uint32_t crc = 0;
    for (int i = 0; i < size; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 1) {
                crc = (crc >> 1) ^ 0xedb88320u;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(OdomPubNode)

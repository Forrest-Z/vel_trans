#include <sonar_interface_node.hpp>

SonarInterfaceNode::SonarInterfaceNode(const rclcpp::NodeOptions & node_options)
: Node("sonar_interface", node_options), max_sonar_dist_(5.675)
{
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  // read parameter for serial
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 115200);
  x_num = declare_parameter("x_num", 1);  // 前后方向的传感器数量
  y_num = declare_parameter("y_num", 0);  // 斜方向的传感器数量
  z_num = declare_parameter("z_num", 0);  // 左右方向的传感器数量
  rate = declare_parameter("rate", 10);
  smooth_ = declare_parameter(
    "smooth",
    4);  // 这款传感器的数值稳定性还不够，面对特征较弱的障碍物时，大部分时间会返回真实测值，但偶尔还串入一些最大值（因为没有接收到回波），为了保险起见，传感器的每次测值取前smooth_次的最小值
  // set parameter for serial
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 5000, 0, 5000, 0);
  this->serial_.open();

  // alarm data publisher
  publisher_x =
    create_publisher<std_msgs::msg::Float32>("~/output/fb_sonar_dist", 1);  // 前后方向的最小距离
  publisher_y =
    create_publisher<std_msgs::msg::Float32>("~/output/skew_sonar_dist", 1);  // skew方向的最小距离
  publisher_z =
    create_publisher<std_msgs::msg::Float32>("~/output/lr_sonar_dist", 1);  // 左右方向的最小距离

  // 在一个访问频次内，依次访问各个传感器的值，
  v.push_back(max_sonar_dist_);
  for (int i = 0; i < x_num + y_num + z_num; i++) {
    v_list.push_back(v);
  }
  front_back_length_ = max_sonar_dist_;
  left_right_length_ = max_sonar_dist_;
  skew_length_ = max_sonar_dist_;

  timer_ =
    create_wall_timer(second_type(1.0 / rate), std::bind(&SonarInterfaceNode::readTimer, this));

  pub_timer_ =
    create_wall_timer(second_type(1.0 / 1), std::bind(&SonarInterfaceNode::pubTimer, this));
}

SonarInterfaceNode::~SonarInterfaceNode() { this->serial_.close(); }

void SonarInterfaceNode::pubTimer()
{
  std_msgs::msg::Float32 warning_data1;
  warning_data1.data = front_back_length_;
  publisher_x->publish(warning_data1);

  std_msgs::msg::Float32 warning_data2;
  warning_data2.data = skew_length_;
  publisher_y->publish(warning_data2);

  std_msgs::msg::Float32 warning_data3;
  warning_data3.data = left_right_length_;
  publisher_z->publish(warning_data3);

  // RCLCPP_INFO(this->get_logger(), "!!!!!!!!!!!!!!!!!\n");
}

void SonarInterfaceNode::readTimer()
{
  front_back_length_ = max_sonar_dist_;
  for (int i = 0; i < x_num; i++) {
    uint8_t recbuff[2];
    try {
      payload[0] = 224 + i * 2;
      payload[1] = 0x02;
      payload[2] = 0xb0;
      this->serial_.write(payload, sizeof(payload));
      this->serial_.read(recbuff, sizeof(recbuff));
      float sonar_range = ((recbuff[0] << 8) | (recbuff[1] & 0xff));
      sonar_range = sonar_range / 1000;
      if ((int)v_list[i].size() == smooth_) {
        v_list[i].erase(v_list[i].begin());
      }
      v_list[i].push_back(sonar_range);
      float sonar_range_ = *min_element(v_list[i].begin(), v_list[i].end());
      front_back_length_ = std::min(front_back_length_, std::min(max_sonar_dist_, sonar_range_));
    } catch (serial::IOException & e) {
      // print error msg
    }
  }

    // 然后访问skew传感器
   skew_length_ = max_sonar_dist_;
  for (int i = x_num; i < x_num + y_num; i++) {
    uint8_t recbuff[2];
    try {
      payload[0] = 224 + i * 2;
      payload[1] = 0x02;
      payload[2] = 0xb0;
      this->serial_.write(payload, sizeof(payload));
      this->serial_.read(recbuff, sizeof(recbuff));
      float sonar_range = ((recbuff[0] << 8) | (recbuff[1] & 0xff));
      sonar_range = sonar_range / 1000;
      if ((int)v_list[i].size() == smooth_) {
        v_list[i].erase(v_list[i].begin());
      }
      v_list[i].push_back(sonar_range);
      float sonar_range_ = *min_element(v_list[i].begin(), v_list[i].end());
      skew_length_ = std::min(skew_length_, std::min(max_sonar_dist_, sonar_range_));
    } catch (serial::IOException & e) {
      // print error msg
    }
  }


    // 最后访问左右传感器
   left_right_length_ = max_sonar_dist_;
  for ( int i = x_num + y_num; i < x_num + y_num + z_num; i++) {
    uint8_t recbuff[2];
    try {
      payload[0] = 224 + i * 2;
      payload[1] = 0x02;
      payload[2] = 0xb0;
      this->serial_.write(payload, sizeof(payload));
      this->serial_.read(recbuff, sizeof(recbuff));
      float sonar_range = ((recbuff[0] << 8) | (recbuff[1] & 0xff));
      sonar_range = sonar_range / 1000;
      if ((int)v_list[i].size() == smooth_) {
        v_list[i].erase(v_list[i].begin());
      }
      v_list[i].push_back(sonar_range);
      float sonar_range_ = *min_element(v_list[i].begin(), v_list[i].end());
      left_right_length_ = std::min(left_right_length_, std::min(max_sonar_dist_, sonar_range_));
    } catch (serial::IOException & e) {
      // print error msg
    }
  }
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SonarInterfaceNode)

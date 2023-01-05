#include <data_transformer/data_transformer_node.hpp>

DataTransformerNode::DataTransformerNode(const rclcpp::NodeOptions & node_options)
: Node("data_tansformer", node_options)
{
  typedef std::chrono::duration<double, std::ratio<1, 1>> second_type;
  using std::placeholders::_1;

  rate_ = declare_parameter("rate", 20);
  simulation_frame_id_ = declare_parameter("simulation_frame_id", "base_link");

  sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
    "~/input/odom", 10, std::bind(&DataTransformerNode::callbackOdom, this, _1));
  sub_engage_ = create_subscription<autoware_vehicle_msgs::msg::Engage>(
    "~/input/engage", 10, std::bind(&DataTransformerNode::callbackEngage, this, _1));
  sub_steer_ = create_subscription<std_msgs::msg::Float64>(
    "~/input/steer", 10, std::bind(&DataTransformerNode::callbackSteer, this, _1));

  pub_steer_ = create_publisher<autoware_vehicle_msgs::msg::Steering>("~/output/steer", 1);
  pub_control_mode_ =
    create_publisher<autoware_vehicle_msgs::msg::ControlMode>("~/output/control_mode", 1);
  pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>("~/output/twist", 1);

  control_mode_ = std::make_shared<autoware_vehicle_msgs::msg::ControlMode>();
  control_mode_->data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;

  timer_ =
    create_wall_timer(second_type(1.0 / rate_), std::bind(&DataTransformerNode::onTimer, this));
}

void DataTransformerNode::callbackOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  if (!twist_) {
    twist_ = std::make_shared<geometry_msgs::msg::TwistStamped>();
  }
  twist_->twist = msg->twist.twist;
  twist_->header = msg->header;
}

void DataTransformerNode::callbackSteer(const std_msgs::msg::Float64::ConstSharedPtr msg)
{
  if (!steer_) {
    steer_ = std::make_shared<autoware_vehicle_msgs::msg::Steering>();
  }
  steer_->data = msg->data;
}

void DataTransformerNode::callbackEngage(
  const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg)
{
  if (msg->engage) {
    control_mode_->data = autoware_vehicle_msgs::msg::ControlMode::AUTO;
  } else {
    control_mode_->data = autoware_vehicle_msgs::msg::ControlMode::MANUAL;
  }
}

void DataTransformerNode::onTimer()
{
  if(!twist_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "twist_ not ready");
    return;
  }
  if(!steer_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "steer_ not ready");
    return;
  }
  if(!control_mode_) {
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000, "control_mode_ not ready");
    return;
  }
  pub_control_mode_->publish(*control_mode_);
  pub_twist_->publish(*twist_);
  rclcpp::Time current_time = get_clock()->now();
  steer_->header.frame_id = simulation_frame_id_;
  steer_->header.stamp = current_time;
  pub_steer_->publish(*steer_);
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(DataTransformerNode)

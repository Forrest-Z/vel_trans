#include <record_path/record_path_node.hpp>

RecordPathNode::RecordPathNode(const rclcpp::NodeOptions & options)
: Node("record_path", options),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  record_flag_(false),
  first_record_(true)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef std::chrono::duration<double,std::ratio<1,1>> second_type;
  map_frame_ = declare_parameter("map_frame", "map");
  base_link_frame_ = declare_parameter("base_link_frame", "base_link");
  odom_topic_ = declare_parameter("odom_topic", "odom");
  distance_interval_ = declare_parameter("distance_interval", 0.05);
  path_file_ = declare_parameter("path_file", ""); 
  RCLCPP_INFO(get_logger(), "start_record %s", path_file_.c_str()); 
  path_pub_ = create_publisher<nav_msgs::msg::Path>("/recorded_path", rclcpp::QoS(1));
  
  record_timer_ = create_wall_timer(second_type(0.5),
    std::bind(&RecordPathNode::record_callback, this));
  start_record_server_ = create_service<path_server::srv::SetPathName>(
        "start_record_path", std::bind(&RecordPathNode::start_record, this, _1, _2));
  stop_record_server_ = create_service<path_server::srv::SetPathName>(
        "stop_record_path", std::bind(&RecordPathNode::stop_record, this, _1, _2));
}

RecordPathNode::~RecordPathNode() {
  output_file_.close();
}


void RecordPathNode::record_callback() {
  if (record_flag_ && tf_buffer_.canTransform(map_frame_, base_link_frame_,
    tf2::TimePointZero)) {
    geometry_msgs::msg::TransformStamped map_to_baselink;
    try {
      map_to_baselink = tf_buffer_.lookupTransform(map_frame_, base_link_frame_,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      return;
    }
    double x = map_to_baselink.transform.translation.x;
    double y = map_to_baselink.transform.translation.y;
    double yaw = tf2::getYaw(map_to_baselink.transform.rotation);

    if (first_record_) {
        record_data(x, y, yaw);
        first_record_ = false;
    } else if (norm2(cache_x_ - x, cache_y_ - y, 0) > (distance_interval_ * distance_interval_)) {
        record_data(x, y, yaw);
    }
  }
}

void RecordPathNode::record_data(double x, double y, double yaw) {
  if (output_file_) {
    output_file_ << x << " " << y << " " << yaw << std::endl;
  }

  cache_x_ = x;
  cache_y_ = y;
  cache_yaw_ = yaw;
  tf2::Quaternion myQuaternion;
  myQuaternion.setRPY(0, 0, yaw);
  geometry_msgs::msg::PoseStamped poseStamped;
  poseStamped.header.stamp = get_clock()->now();
  poseStamped.header.frame_id = map_frame_;
  poseStamped.pose.position.x = x;
  poseStamped.pose.position.y = y;
  poseStamped.pose.orientation = tf2::toMsg(myQuaternion);
  path_data_.poses.push_back(poseStamped);
  path_pub_->publish(path_data_);
}


void RecordPathNode::start_record(const std::shared_ptr<path_server::srv::SetPathName::Request>,
        const std::shared_ptr<path_server::srv::SetPathName::Response>) {
  RCLCPP_INFO(get_logger(), "start_record at: %s", path_file_.c_str());  
  output_file_.close();
  output_file_ = std::ofstream(path_file_);

  path_data_.header.stamp = get_clock()->now();
  path_data_.header.frame_id = map_frame_;
  path_data_.poses.clear();

  record_flag_ = true;
  first_record_ = true;
  return;
}

void RecordPathNode::stop_record(const std::shared_ptr<path_server::srv::SetPathName::Request>,
        const std::shared_ptr<path_server::srv::SetPathName::Response>) {
  RCLCPP_INFO(get_logger(), "stop_record"); 
  geometry_msgs::msg::TransformStamped map_to_baselink;
  try {
    map_to_baselink = tf_buffer_.lookupTransform(map_frame_, base_link_frame_,
      tf2::TimePointZero);
    record_data(map_to_baselink.transform.translation.x,
      map_to_baselink.transform.translation.y,
      tf2::getYaw(map_to_baselink.transform.rotation)
    );
  } catch (tf2::TransformException & ex) {
    RCLCPP_WARN(get_logger(), "%s", ex.what());
    rclcpp::sleep_for(std::chrono::milliseconds(100));
  }
  output_file_.close();
  record_flag_ = false;
  return;
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(RecordPathNode)
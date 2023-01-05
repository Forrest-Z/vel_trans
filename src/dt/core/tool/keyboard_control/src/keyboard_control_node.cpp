
#include "keyboard_control/keyboard_control_node.hpp"

//namespace dt_tool {

KeyboardControl::KeyboardControl(const rclcpp::NodeOptions & node_options)
: Node("keyboard_control_node", node_options),
linear_state_(0),
angular_state_(0),
port_name_("")
{
  RCLCPP_INFO(this->get_logger(),"program start !!!!!!!!!!!!!!!!!!!!!");

  using std::placeholders::_1;
  using std::placeholders::_2;
  typedef std::chrono::duration<double,std::ratio<1,1>> second_type;

  linear_min_ = declare_parameter("linear_min", 0.2);
  linear_max_ = declare_parameter("linear_max", 2.0);
  linear_step_ = declare_parameter("linear_step", 0.2);
  angular_min_ = declare_parameter("angular_min", 0.5);
  angular_max_ = declare_parameter("angular_max", 4.0);
  angular_step_ = declare_parameter("angular_step", 0.2);
  smooth_step_ = declare_parameter("smooth_step", 0.1); //平滑每档速度
  smooth_switch_ = declare_parameter("smooth_switch", false); //平滑开关
  rate_ = declare_parameter("rate", 10.0);


  parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(this, "/driver/can_driver");

  linear_scale_ = linear_min_;
  angular_scale_ = angular_min_;
  send_flag_ = true;

  setlocale(LC_ALL,"");

  if (init()){
  twist_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  stop_pub_ = create_publisher<std_msgs::msg::Bool>("/emgency_stop", 100);
  twist_pub_timer_ = create_wall_timer( second_type(1.0/rate_),std::bind(&KeyboardControl::twistCallback, this));
  boost::thread parse_thread(boost::bind(&KeyboardControl::parseKeyboard, this));

  }
}

KeyboardControl::~KeyboardControl(){
 close(fd_);
}


//急停开
void KeyboardControl::emgstop(){
  std_msgs::msg::Bool data_push;
  data_push.data = true;
  stop_pub_->publish(data_push); 
  RCLCPP_INFO(this->get_logger(),"急停开");
}

//急停关
void KeyboardControl::emgcontinue(){
  std_msgs::msg::Bool data_push;
  data_push.data = false;
  stop_pub_->publish(data_push); 
  RCLCPP_INFO(this->get_logger(),"急停关");
}

void KeyboardControl::buttonTwistCheck(int value, int& state, int down, int up){
  if (value == 1){
    state += down;
  }else if (value == 0){
    state += up;
  }
}

void KeyboardControl::buttonScaleCheck(int value, double &scale, double step, double limit){
  if (value == 1){
    if (step > 0){
      scale = std::min(scale + step, limit);
    }else{
      scale = std::max(scale + step, limit);
    }
  }
}

void KeyboardControl::parseKeyboard(){
  while (true) {
    // TODO(yty): warning: ignoring return value of ‘ssize_t read(int, void*, size_t)’
    read(fd_, &ev_, sizeof(struct input_event));
    if (ev_.type == EV_KEY){
      RCLCPP_DEBUG(this->get_logger(),"INFO: [key]: %d ; [value]: %d ",ev_.code ,ev_.value);
      //ROS_DEBUG_STREAM("INFO: [key]: " << ev_.code << ", [value]: " << ev_.value);
      switch (ev_.code) {
      case KEYBOARD_UP:
        buttonTwistCheck(ev_.value, linear_state_, 1, -1);
        break;
      case KEYBOARD_DOWN:
        buttonTwistCheck(ev_.value, linear_state_, -1, 1);
        break;
      case KEYBOARD_LEFT:
        buttonTwistCheck(ev_.value, angular_state_, 1, -1);
        break;
      case KEYBOARD_RIGHT:
        buttonTwistCheck(ev_.value, angular_state_, -1, 1);
        break;

      case EMERGENCY_STOP:      //急停开
        emgstop();
        break;
      case EMERGENCY_GO:      //急停关
        emgcontinue();
        break;
      case KEYBOARD_1:
        buttonScaleCheck(ev_.value, linear_scale_, linear_step_, linear_max_);
        break;
      case KEYBOARD_2:
        buttonScaleCheck(ev_.value, linear_scale_, -linear_step_, linear_min_);
        break;
      case KEYBOARD_3:
        buttonScaleCheck(ev_.value, angular_scale_, angular_step_, angular_max_);
        break;
      case KEYBOARD_4:
        buttonScaleCheck(ev_.value, angular_scale_, -angular_step_, angular_min_);
        break;

      case KEYBOARD_9:
        if (ev_.value == 1){
          send_flag_ = true;
        }
        break;
      case KEYBOARD_0:
        if (ev_.value == 1){
          send_flag_ = false;
        }
        break;
      default:
        break;
      }
    }
  }
  
}

void KeyboardControl::twistCallback(){
  if (send_flag_){
    geometry_msgs::msg::Twist twist;
    if (!smooth_switch_)
    {
    twist.linear.x = linear_state_ * linear_scale_;
    twist.angular.z = angular_state_ * angular_scale_;
    twist_pub_->publish(twist);
    RCLCPP_DEBUG(this->get_logger(),"linear: %f ; angular: %f ",twist.linear.x , twist.angular.z);
    }
    else
    {
    twist.linear.x = linear_state_ * linear_scale_;
    twist.angular.z = angular_state_ * angular_scale_;
    vel_now_ = twist.linear.x;
    vel_last_ = twist.linear.x;
    twist_pub_->publish(twist);
    RCLCPP_DEBUG(this->get_logger(),"linear: %f ; angular: %f ",twist.linear.x , twist.angular.z);

    }
  }
}


bool KeyboardControl::init(){

  RCLCPP_INFO(this->get_logger(),"init func start !!!!!!!!!!!!!!!!!!!!!");
  const char path[] = "/dev/input/by-path";
  DIR *dev_dir = opendir(path);
  struct dirent *entry;
  if (dev_dir == NULL){
    return false;
  }

  while ((entry = readdir(dev_dir)) != NULL){
    std::string dir_str = entry->d_name;
    if (dir_str.find("event-kbd") < dir_str.length()){
      port_name_ = std::string(path) + "/" + dir_str;
      RCLCPP_INFO(this->get_logger(),"INFO: The keyboard port is : '%s' " , port_name_.c_str());
      break;
    }
  }
  closedir(dev_dir);

  if (port_name_ != ""){
    fd_ = open(port_name_.c_str(), O_RDONLY);
    if (fd_ < 0){
      RCLCPP_INFO(this->get_logger(),"ERROR: Can't Open The Port : '%s' " , port_name_.c_str());
      return false;
    }else{
      RCLCPP_INFO(this->get_logger(),"INFO: Open The Port : '%s' " , port_name_.c_str());
      return true;
    }
  }else{
    return false;
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(KeyboardControl)

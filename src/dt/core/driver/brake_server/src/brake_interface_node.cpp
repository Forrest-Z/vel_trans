#include <brake_interface_node.hpp>

BrakeInterfaceNode::BrakeInterfaceNode(const rclcpp::NodeOptions & node_options) 
: Node("brake_interface", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  port_ = declare_parameter("port", "/dev/ttyUSB0");
  baud_ = declare_parameter("baud", 230400);
  brake_value_ = declare_parameter("brake_value", 10000);
  init_value_ = declare_parameter("init_value", 1000);
  retrace_value_ = declare_parameter("retrace_value", 9000);
  // set parameter for serial
  this->serial_.setPort(port_);
  this->serial_.setBaudrate(baud_);
  this->serial_.setTimeout(std::numeric_limits<uint32_t>::max(), 5000, 0, 5000, 0);
  this->serial_.open() ;
  init();
  const double dt = 1.0 / 10.0;
  auto timer_callback = std::bind(&BrakeInterfaceNode::on_timer, this);
  auto period =
    std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(dt));
  timer_ = std::make_shared<rclcpp::GenericTimer<decltype(timer_callback)>>(
    this->get_clock(), period, std::move(timer_callback),
    this->get_node_base_interface()->get_context());
  this->get_node_timers_interface()->add_timer(timer_, nullptr);
  
  control_service_ = create_service<brake_server::srv::ControlService>(
  "wash_brush_server", std::bind(&BrakeInterfaceNode::controlService, this, _1, _2));
}

void BrakeInterfaceNode::controlService(
  const std::shared_ptr<brake_server::srv::ControlService::Request> req,
  std::shared_ptr<brake_server::srv::ControlService::Response>)
{
    if(req->type == "brake" )
    {
        if(req->start == true) {
            this->brake_start_ = 1;
            RCLCPP_INFO(this->get_logger(), "brake start: %d\n", this->brake_start_);
        }
        else {
            this->brake_start_ = 0;
            RCLCPP_INFO(this->get_logger(), "brake close: %d\n", this->brake_start_);
        }
    }
}

void BrakeInterfaceNode::setmode()
{
  uint8_t payload[8];
  payload[0] = 0x01;
  payload[1] = 0x06;
  payload[2] = 0x51;
  payload[3] = 0x00;
  payload[4] = 0x01;
  payload[5] = 0x52;
  unsigned short tmp = 0xffff;
  unsigned short ret1 = 0;
  for(int n = 0; n < 6; n++){
    tmp = payload[n] ^ tmp;
    for(int i = 0; i < 8; i++){  
      if(tmp & 0x01){
        tmp = tmp >> 1;
        tmp = tmp ^ 0xa001;
      }   
      else{
        tmp = tmp >> 1;
      }   
    }   
  } 
  ret1 = tmp >> 8;
  ret1 = ret1 | (tmp << 8); 
  payload[6]  = ret1 >> 8;
  payload[7]  = ret1 & 0xff;
  this->serial_.flushInput();
  this->serial_.write(payload, 8);
}

void BrakeInterfaceNode::setpose(int value_)
{
  uint8_t payload[13];
  payload[0] = 0x01;
  payload[1] = 0x10;
  payload[2] = 0x00;
  payload[3] = 0x50;
  payload[4] = 0x00;
  payload[5] = 0x02;
  payload[6] = 0x04;
  unsigned short data1;
  if (value_ >= 0){
    data1 = value_  >> 16 & 0xffff;
  }
  else{
    data1 = (value_ - 65535)  >> 16 & 0xffff;
  }
  payload[7]  = data1 >> 8 & 0xff;
  payload[8]  = data1 & 0xff;
  unsigned short data2  = value_ & 0xffff;
  payload[9]  = data2 >> 8 & 0xff;
  payload[10] = data2 & 0xff;
  unsigned short tmp = 0xffff;
  unsigned short ret1 = 0;
  for(int n = 0; n < 11; n++){
    tmp = payload[n] ^ tmp;
    for(int i = 0; i < 8; i++){  
      if(tmp & 0x01){
        tmp = tmp >> 1;
        tmp = tmp ^ 0xa001;
      }   
      else{
        tmp = tmp >> 1;
      }   
    }   
  } 
  ret1 = tmp >> 8;
  ret1 = ret1 | (tmp << 8); 
  payload[11]  = ret1 >> 8;
  payload[12]  = ret1 & 0xff;
  this->serial_.flushInput();
  this->serial_.write(payload, 13);
}

int BrakeInterfaceNode::getpose()
{
    uint8_t payload[9];
    payload[0] = 0x01;
    payload[1] = 0x00;
    payload[2] = 0x2A;
    payload[3] = 0xE8;
    payload[4] = 0x00;
    payload[5] = 0x00;
    payload[6] = 0xE9;
    payload[7] = 0x00;
    payload[8] = 0x00;
  
    // unsigned short tmp = 0xffff;
    // unsigned short ret1 = 0;
    // for(int n = 0; n < 9; n++){
    //   tmp = payload[n] ^ tmp;
    //   for(int i = 0; i < 8; i++){  
    //     if(tmp & 0x01){
    //       tmp = tmp >> 1;
    //       tmp = tmp ^ 0xa001;
    //     }   
    //     else{
    //       tmp = tmp >> 1;
    //     }   
    //   }   
    // } 
    // ret1 = tmp >> 8;
    // ret1 = ret1 | (tmp << 8); 
    // payload[9]  = ret1 >> 8;
    // payload[10]  = ret1 & 0xff;

    this->serial_.flushInput();
    this->serial_.write(payload, 9);
    uint8_t recbuff[9];
    this->serial_.read(recbuff, (int)9);
    uint8_t number_high;
    number_high = recbuff[4];
    number_high <<= 8;
    number_high |= recbuff[5];
    uint8_t number_low;
    number_low = recbuff[7];
    number_low <<= 8;
    number_low |= recbuff[8];
    uint8_t number_final;
    number_final = number_high;
    number_final <<= 16;
    number_final |= number_low;
    return number_final;
}

void BrakeInterfaceNode::init()
{
  setmode();
  setpose(init_value_);
  int pose = getpose();
  while(fabs(pose - init_value_) < 10){
    setpose(init_value_);
    int pose_tmt = getpose();
    pose = pose_tmt - pose;
  }
  setpose(-1 * retrace_value_);
}

void BrakeInterfaceNode::on_timer()
{
  if(brake_start_ == 1){
    setpose(brake_value_);
    brake_start_ = 2;
  }
  else if(brake_start_ == 0){
    setpose(-1 * brake_value_);
    brake_start_ = 2;
  }
  else{
    
  }   
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(BrakeInterfaceNode)

#include "chasu/can_bus_service_node.hpp"

CanBusServiceNode::CanBusServiceNode(const rclcpp::NodeOptions & node_options)
: Node("can_bus_service_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  // set parameter
  gDevType1_ = declare_parameter("gDevType1", 3);
  gDevIdx1_ = declare_parameter("gDevIdx1", 0);
  gChMask1_ = declare_parameter("gChMask1", 0);
  gBaud1_ = declare_parameter("gBaud1", 7168);
  gTxType1_ = declare_parameter("gTxType1", 0);
  guard_bit = 0;
  if (init()) {
    RCLCPP_INFO(this->get_logger(), " start ok");
    qianjin_pub = create_publisher<chasu::msg::Cmd>("/qianjin_pub", 10);
    jiaodu_pub = create_publisher<chasu::msg::Cmd>("/jiaodu_pub", 10);
    qianjin_rec = create_publisher<chasu::msg::Cmd>("/qianjin_rec", 10);
    jiaodu_rec = create_publisher<chasu::msg::Cmd>("/jiaodu_rec", 10);
    brush_pub = create_publisher<chasu::msg::Cmd>("/brush_pub", 10);
    enable_debug_ = declare_parameter("enable_debug", true);
    canbus_msg_pub1_ = create_publisher<chasu::msg::CanBusMessage>("canbus_msg", 10);
    canbus_msg_service_ = create_service<chasu::srv::CanBusService>(
    "canbus_server", std::bind(&CanBusServiceNode::canbus_service, this, _1, _2));
    boost::thread parse_thread1(boost::bind(&CanBusServiceNode::parse_msg1, this));
  }
}
bool CanBusServiceNode::trans_msg(int dev, int num, int ID, int externflag, vector <unsigned char> & data){
  CAN_OBJ can;
  memset(&can, 0, sizeof(CAN_OBJ));
  can.ID = ID;
  can.SendType = 0;
  can.ExternFlag = externflag;
  can.RemoteFlag = 0;
  can.DataLen = 8;  
  for (int i = 0; i < 8; i++) {
    can.Data[i] = data[i];
  }
  if (Transmit(dev, num, 0, &can, 1) == 0) {
    RCLCPP_INFO(this->get_logger(), "transmit failed\n");
    return false;
  }
  return true;
}
void CanBusServiceNode::closedev(){
  if (!ResetCAN(this->gDevType1_, this->gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
  }
  if (!CloseDevice(this->gDevType1_, this->gDevIdx1_)) {
    RCLCPP_INFO(this->get_logger(), "CloseCAN1 failed\n");
  }
}
bool CanBusServiceNode::opendev(){
  while (!OpenDevice(gDevType1_, gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "gDevType1_%d\n", gDevType1_);
    RCLCPP_INFO(this->get_logger(), "gDevIdx1_%d\n", gDevIdx1_);
    RCLCPP_INFO(this->get_logger(), "OpenDevice1 failed\n");
    if (!ResetCAN(gDevType1_, gDevIdx1_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
    }
    if (!CloseDevice(gDevType1_, gDevIdx1_)) {
      RCLCPP_INFO(this->get_logger(), "CloseCAN1 failed\n");
    }
  }
  INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xffffff;
  config.Filter = 0;
  config.Mode = 0;
  config.Timing0 = gBaud1_ & 0xff;
  config.Timing1 = gBaud1_ >> 8;
  if (!InitCAN(gDevType1_, gDevIdx1_, 0, &config)) {
    RCLCPP_INFO(this->get_logger(), "InitDevice1 failed\n");
    return false;
  }
  if (!StartCAN(gDevType1_, gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "StartCAN1 failed\n");
    return false;
  }
  return true;
}
void CanBusServiceNode::startdev(){
  closedev();
  bool state = opendev();
  while(!state) {
    state = opendev();
  }
}
bool CanBusServiceNode::init()
{
  bool rec = false;
  while(!rec) {
    startdev();
    rclcpp::Time t1 = rclcpp::Node::now();
    rclcpp::Time t2 = rclcpp::Node::now();
    bool rec = false;
    while((t2.seconds() - t1.seconds()) < 2) {
      CAN_OBJ can1[1];
      int cnt1;  // current received
      cnt1 = Receive(gDevType1_, gDevIdx1_, gChMask1_, can1, 1, 2);
      if (cnt1 == 1) {
        rec = true;
        break;
      }
      t2 = rclcpp::Node::now();
    }
    if(rec) {
      return true;
    } else {    
    }
  }
}
void CanBusServiceNode::parse_msg1()
{
  while (true) {
    CAN_OBJ can[RX_BUFF_SIZE];  // buffer
    int cnt;                    // current received
    int i;
    cnt = Receive(gDevType1_, gDevIdx1_, gChMask1_, can, RX_BUFF_SIZE, RX_WAIT_TIME);
    if (cnt == 0xFFFFFFFF || cnt != RX_BUFF_SIZE) {
      continue;
    }
    for (i = 0; i < cnt; i++) {
      chasu::msg::CanBusMessage msg;
      if (verify_frame(&can[i], &msg)) {
        canbus_msg_pub1_->publish(msg);
        if (enable_debug_) {
          chasu::msg::Cmd c_msg;
          c_msg.header.stamp = rclcpp::Node::now();
          c_msg.node_type = msg.node_type;
          c_msg.payload = msg.payload;
          if (c_msg.node_type == 1) {
            qianjin_rec->publish(c_msg);
          }
          if (c_msg.node_type == 2) {
            jiaodu_rec->publish(c_msg);
          }
        }
      } else {
      }
    }
  }
}

bool CanBusServiceNode::canbus_service(
  const std::shared_ptr<chasu::srv::CanBusService::Request> req,
  std::shared_ptr<chasu::srv::CanBusService::Response>)
{
  if (req->requests.size() > 0) 
  {
    for (int i = 0; i < 1; i++) 
    {
      chasu::msg::CanBusMessage msg = req->requests[i];
      CAN_OBJ can;
      memset(&can, 0, sizeof(CAN_OBJ));
      int is_data = 0;
      switch (msg.node_type) 
      {
        case 1:
          if (trans_msg(gDevType1_, gDevIdx1_, 0x201, 0, msg.payload) == false) 
          {
            RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
          } 
          if (enable_debug_) {
            chasu::msg::Cmd c_msg;
            c_msg.header.stamp = rclcpp::Node::now();
            c_msg.node_type = msg.node_type;
            c_msg.payload = msg.payload;
            qianjin_pub->publish(c_msg);
          }   
          break;
        default:
          break;
      }
    } 
    return true;
  } else {
    return false;
  }
}

bool CanBusServiceNode::verify_frame(CAN_OBJ * can, chasu::msg::CanBusMessage * msg)
{
  if (can->DataLen > 8) return 0;  // error: data length
  switch (can->ID) {
    case 0x181:
      msg->node_type = 1;
      msg->payload.clear();
      for (int i = 0; i < 8; i++) {
        msg->payload.push_back(can->Data[i]);
      }
      break;
    default:
      return 0;
      break;
  }
  return 1;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanBusServiceNode)

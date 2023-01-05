#include "can_bus/can_bus_service_node.hpp"

CanBusServiceNode::CanBusServiceNode(const rclcpp::NodeOptions & node_options)
: Node("can_bus_service_node", node_options)
{
  using std::placeholders::_1;
  using std::placeholders::_2;
  // set parameter
  gDevType1_ = declare_parameter("gDevType1", 3);
  gDevIdx1_ = declare_parameter("gDevIdx1", 0);
  gChMask1_ = declare_parameter("gChMask1", 0);
  gBaud1_ = declare_parameter("gBaud1", 7169);
  gTxType1_ = declare_parameter("gTxType1", 0);
  gDevType2_ = declare_parameter("gDevType2", 3);
  gDevIdx2_ = declare_parameter("gDevIdx2", 1);
  gChMask2_ = declare_parameter("gChMask2", 0);
  gBaud2_ = declare_parameter("gBaud2", 7168);
  gTxType2_ = declare_parameter("gTxType2", 0);
  enable_debug_ = declare_parameter("enable_debug", false);
  brush_state = declare_parameter("brush_state", 0);
  light_limit = declare_parameter("light_limit", 10);
  light_state = declare_parameter("light_state", 0);
  guard_bit = 0;
  if (init()) {
    RCLCPP_INFO(this->get_logger(), " start ok");
    qianjin_pub = create_publisher<can_bus::msg::Cmd>("/qianjin_pub", 10);
    jiaodu_pub = create_publisher<can_bus::msg::Cmd>("/jiaodu_pub", 10);
    qianjin_rec = create_publisher<can_bus::msg::Cmd>("/qianjin_rec", 10);
    jiaodu_rec = create_publisher<can_bus::msg::Cmd>("/jiaodu_rec", 10);
    brush_pub = create_publisher<can_bus::msg::Cmd>("/brush_pub", 10);
    canbus_msg_pub1_ = create_publisher<can_bus::msg::CanBusMessage>("canbus_msg", 10);
    canbus_msg_service_ = create_service<can_bus::srv::CanBusService>(
      "canbus_server", std::bind(&CanBusServiceNode::canbus_service, this, _1, _2));
    boost::thread parse_thread1(boost::bind(&CanBusServiceNode::parse_msg1, this));
    boost::thread parse_thread2(boost::bind(&CanBusServiceNode::parse_msg2, this));
    control_service_ = create_service<can_bus::srv::ControlService>(
    "wash_brush_server", std::bind(&CanBusServiceNode::controlService, this, _1, _2));
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
  if (!ResetCAN(this->gDevType2_, this->gDevIdx2_, 0)) {
    RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
  }
  if (!CloseDevice(this->gDevType2_, this->gDevIdx2_)) {
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
  vector<unsigned char> data1{0xAA,0xAA,0x00,0x00,0x00,0x00,0x00,0x00};
  trans_msg(gDevType1_, gDevIdx1_, 0x0CFF1C65, 1, data1);
  RCLCPP_INFO(this->get_logger(), "send vel start!!!!!!!!!!!!!!!!!!!\n");

  vector<unsigned char> data2{0x55,0x21,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  trans_msg(gDevType1_, gDevIdx1_, 0x18FF8017, 1, data2);
  vector<unsigned char> data3{0xAA,0x21,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  trans_msg(gDevType1_, gDevIdx1_, 0x18FF8017, 1, data3);
  vector<unsigned char> data4= {0xAA,0x22,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
  trans_msg(gDevType1_, gDevIdx1_, 0x18FF8017, 1, data4);
  RCLCPP_INFO(this->get_logger(), "brush start!!!!!!!!!!!!!!!!!!!\n");

  while (!OpenDevice(gDevType2_, gDevIdx2_, 0)) {
    RCLCPP_INFO(this->get_logger(), "gDevType2_%d\n", gDevType2_);
    RCLCPP_INFO(this->get_logger(), "gDevIdx2_%d\n", gDevIdx2_);
    RCLCPP_INFO(this->get_logger(), "OpenDevice2 failed\n");
    if (!ResetCAN(gDevType2_, gDevIdx2_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN2 failed\n");
    }
    if (!CloseDevice(gDevType2_, gDevIdx2_)) {
      RCLCPP_INFO(this->get_logger(), "closeCAN2 failed\n");
    }
  }
  config.AccCode = 0;
  config.AccMask = 0xffffff;
  config.Filter = 0;
  config.Mode = 0;
  config.Timing0 = gBaud2_ & 0xff;
  config.Timing1 = gBaud2_ >> 8;
  if (!InitCAN(gDevType2_, gDevIdx2_, 0, &config)) {
    RCLCPP_INFO(this->get_logger(), "InitDevice2 failed\n");
    return false;
  }
  if (!StartCAN(gDevType2_, gDevIdx2_, 0)) {
    RCLCPP_INFO(this->get_logger(), "StartCAN2 failed\n");
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
      int cnt2;  // current received
      cnt2 = Receive(gDevType2_, gDevIdx2_, gChMask2_, can1, 1, 2);
      if (cnt1 == 1 && cnt2 == 1) {
        rec = true;
        break;
      }
      t2 = rclcpp::Node::now();
    }
    if(rec) {
      return true;
    } else {    
      int tep = gDevIdx1_;
      gDevIdx1_ = gDevIdx2_;
      gDevIdx2_ = tep;
    }
  }
}
void CanBusServiceNode::controlService(
  const std::shared_ptr<can_bus::srv::ControlService::Request> req,
  std::shared_ptr<can_bus::srv::ControlService::Response>)
{
    if(req->type == "guard" )
    {
        if(req->start == true) {
            RCLCPP_INFO(this->get_logger(), "guard true\n");
            guard_bit = 64;
        }
        else {
            RCLCPP_INFO(this->get_logger(), "guard false\n");
            guard_bit = 0;
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
      can_bus::msg::CanBusMessage msg;
      if (verify_frame(&can[i], &msg)) {
        canbus_msg_pub1_->publish(msg);
        if (enable_debug_) {
          can_bus::msg::Cmd c_msg;
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

void CanBusServiceNode::parse_msg2()
{
  while (true) {
    CAN_OBJ can[RX_BUFF_SIZE];  // buffer
    int cnt;                    // current received
    int i;
    cnt = Receive(gDevType2_, gDevIdx2_, gChMask2_, can, RX_BUFF_SIZE, RX_WAIT_TIME);
    if (cnt == 0xFFFFFFFF || cnt != RX_BUFF_SIZE) {
      continue;
    }
    for (i = 0; i < cnt; i++) {
      can_bus::msg::CanBusMessage msg;
      if (verify_frame(&can[i], &msg)) {
        canbus_msg_pub1_->publish(msg);
        if (enable_debug_) {
          can_bus::msg::Cmd c_msg;
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
  const std::shared_ptr<can_bus::srv::CanBusService::Request> req,
  std::shared_ptr<can_bus::srv::CanBusService::Response>)
{
  if (req->requests.size() > 0) {
    for (int i = 0; i < req->requests.size(); i++) {
      can_bus::msg::CanBusMessage msg = req->requests[i];
      CAN_OBJ can;
      memset(&can, 0, sizeof(CAN_OBJ));
      int is_data = 0;
      switch (msg.node_type) {
        case 1:
          can.ID = 0x0CFF17EF;
          is_data = 1;
          break;
        case 2:
          can.ID = 0x469;
          is_data = 2;
          break;
        case 3:
          can.ID = 0x18FA1117;
          is_data = 3;
          break;
        default:
          break;
      }
      if (is_data == 1) {
        if (trans_msg(gDevType1_, gDevIdx1_, 0x0CFF17EF, 1, msg.payload) == false) {
          RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
        } else {
          if (enable_debug_) {
            can_bus::msg::Cmd c_msg;
            c_msg.header.stamp = rclcpp::Node::now();
            c_msg.node_type = msg.node_type;
            c_msg.payload = msg.payload;
            qianjin_pub->publish(c_msg);
          }
        }
      } else if (is_data == 2) {
        if (trans_msg(gDevType2_, gDevIdx2_, 0x469, 0, msg.payload) == false) {
          RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");

        } else {
          if (enable_debug_) {
            can_bus::msg::Cmd c_msg;
            c_msg.header.stamp = rclcpp::Node::now();
            c_msg.node_type = msg.node_type;
            c_msg.payload = msg.payload;
            jiaodu_pub->publish(c_msg);
          }
          double rd = (msg.payload[3] * 256 + msg.payload[4] - 1024) / 4;
          int light_cmd;
          if (fabs(rd) < light_limit){
            light_cmd = 0;
          }
          else if(rd >=light_limit){
            light_cmd = 1;
          }
          else{
            light_cmd = 2;
          }
          if(light_state != light_cmd && light_cmd == 0){
            switch (brush_state) {
              case 0:{
                vector<unsigned char> data1{0x00,0x00,0x00,0x00+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                light_state = 0;
                break;
              }
              case 1:{
                vector<unsigned char> data1{0x00,32,0x00,0x01+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                light_state = 0;
                break;
              }
              case 2:{
                vector<unsigned char> data1{0x00,32,0x00,0x05+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                light_state = 0;
                break;
              }
              default:
                break;
            }
          }
          else if(light_state != light_cmd && light_cmd == 1){
            switch (brush_state) {
              case 0:{
                vector<unsigned char> data1{0x01,32,0x00,0x00+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x02,0x00,0x00,0x00,0x00,0x00,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 1;
                break;
              }
              case 1:{
                vector<unsigned char> data1{0x01,32,0x00,0x01+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x02,0x00,0x00,0x00,0x00,0x00,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 1;
                break;
              }
              case 2:{
                vector<unsigned char> data1{0x01,32,0x00,0x05+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x02,0x00,0x00,0x00,0x00,0x00,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 1;
                break;
              }
              default:
                break;
            }
          }
          else if(light_state != light_cmd && light_cmd == 2){
            switch (brush_state) {
              case 0:{
                vector<unsigned char> data1{0x00,0x00,0x00,0x00+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x01,0x00,0x00,0x00,0x00,0x01,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 2;
                break;
              }
              case 1:{
                vector<unsigned char> data1{0x00,32,0x00,0x01+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x01,0x00,0x00,0x00,0x00,0x01,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 2;
                break;
              }
              case 2:{
                vector<unsigned char> data1{0x00,32,0x00,0x05+guard_bit,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1);
                vector<unsigned char> data2{0x00,0x00,0x00,0x80,0x00,0x00,0x00,0x00};
                trans_msg(gDevType1_, gDevIdx1_, 0x18F21117, 1, data2);
                vector<unsigned char> data3{0x01,0x00,0x00,0x00,0x00,0x01,0x3f,0x52};
                trans_msg(gDevType1_, gDevIdx1_, 0x18FA6218, 1, data3);
                light_state = 2;
                break;
              }
              default:
                break;
            }
          }
          else{
          }
        }
      } else if (is_data == 3) {
        vector<unsigned char> data1;
        for (int i = 0; i < 8; i++) {
          data1[i] = msg.payload[i];
        }
        data1[3] = data1[3] + guard_bit;
        if (trans_msg(gDevType1_, gDevIdx1_, 0x18FA1117, 1, data1) == false) {
          RCLCPP_INFO(this->get_logger(), "TYPE3 transmit failed\n");
        } else {
          if (enable_debug_) {
            can_bus::msg::Cmd c_msg;
            c_msg.header.stamp = rclcpp::Node::now();
            c_msg.node_type = msg.node_type;
            c_msg.payload = msg.payload;
            brush_pub->publish(c_msg);
          }
        }
      }
    }
    return true;
  } else {
    return false;
  }
}

bool CanBusServiceNode::verify_frame(CAN_OBJ * can, can_bus::msg::CanBusMessage * msg)
{
  if (can->DataLen > 8) return 0;  // error: data length
  switch (can->ID) {
    case 0x0CFF15EF:
      msg->node_type = 1;
      msg->payload.clear();
      for (int i = 0; i < 8; i++) {
        msg->payload.push_back(can->Data[i]);
      }
      break;
    case 0x401:
      msg->node_type = 2;
      msg->payload.clear();
      for (int i = 0; i < 8; i++) {
        msg->payload.push_back(can->Data[i]);
      }
      break;
    case 0x1821E5F1:
      msg->node_type = 3;
      msg->payload.clear();
      for (int i = 0; i < 8; i++) {
        msg->payload.push_back(can->Data[i]);
      }
      break;
    case 0x18FA0221:
      if(can->Data[0] == 0x02 && can->Data[2] == 0x00 && can->Data[3] == 0x00 && brush_state != 0){
        brush_state = 0;
      }
      else if(can->Data[0] == 0x02 && can->Data[2] == 0x00 && can->Data[3] == 0x40 && brush_state != 1){
        brush_state = 1;
      }
      else if(can->Data[0] == 0x02 && can->Data[2] == 0x40 && can->Data[3] == 0x40 && brush_state != 2){
        brush_state = 2;
      }
      else{
      }
      return 0;
      break;
    default:
      return 0;
      break;
  }
  return 1;
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanBusServiceNode)

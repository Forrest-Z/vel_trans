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
  int num = init();

  RCLCPP_INFO(this->get_logger(), " start num %d \n", num);

  if (num == 2) {
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

int CanBusServiceNode::init()
{
  if (!ResetCAN(this->gDevType1_, this->gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
  }
  if (!CloseDevice(this->gDevType1_, this->gDevIdx1_)) {
    RCLCPP_INFO(this->get_logger(), "CloseCAN1 failed\n");
  }
  while (!OpenDevice(this->gDevType1_, this->gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "gDevType1_%d\n", this->gDevType1_);
    RCLCPP_INFO(this->get_logger(), "gDevIdx1_%d\n", this->gDevIdx1_);
    RCLCPP_INFO(this->get_logger(), "OpenDevice1 failed\n");
    if (!ResetCAN(this->gDevType1_, this->gDevIdx1_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
    }
    if (!CloseDevice(this->gDevType1_, this->gDevIdx1_)) {
      RCLCPP_INFO(this->get_logger(), "CloseCAN1 failed\n");
    }
  }
  INIT_CONFIG config;
  config.AccCode = 0;
  config.AccMask = 0xffffff;
  config.Filter = 0;
  config.Mode = 0;
  config.Timing0 = this->gBaud1_ & 0xff;
  config.Timing1 = this->gBaud1_ >> 8;
  // config.Timing0 = 0x01;
  // config.Timing1 = 0x1C;
  if (!InitCAN(this->gDevType1_, this->gDevIdx1_, 0, &config)) {
    RCLCPP_INFO(this->get_logger(), "InitDevice1 failed\n");
    return 0;
  }
  if (!StartCAN(this->gDevType1_, this->gDevIdx1_, 0)) {
    RCLCPP_INFO(this->get_logger(), "StartCAN1 failed\n");
    return 0;
  }

  CAN_OBJ can;
  memset(&can, 0, sizeof(CAN_OBJ));
  can.ID = 0x0CFF1C65;
  can.SendType = gTxType1_;
  can.ExternFlag = 1;
  can.RemoteFlag = 0;
  can.DataLen = 8;

  for (int i = 0; i < 8; i++) {
    can.Data[i] = 0;
  }
  for (int i = 0; i < 2; i++) {
    can.Data[i] = 0xAA;
  }
  for (int i = 2; i < 8; i++) {
    can.Data[i] = 0x00;
  }
  if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
    RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
  }

  RCLCPP_INFO(this->get_logger(), "send vel start!!!!!!!!!!!!!!!!!!!\n");
  can.ID = 0x18FF8017;
  can.SendType = gTxType1_;
  can.ExternFlag = 1;
  can.RemoteFlag = 0;
  can.DataLen = 8;
  for (int i = 0; i < 8; i++) {
    can.Data[i] = 0;
  }
  can.Data[0] = 0x55;
  can.Data[1] = 0x21;
  for (int i = 2; i < 8; i++) {
    can.Data[i] = 0xFF;
  }
  if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
    RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
  }
  for (int i = 0; i < 8; i++) {
    can.Data[i] = 0;
  }
  can.Data[0] = 0xAA;
  can.Data[1] = 0x21;
  for (int i = 2; i < 8; i++) {
    can.Data[i] = 0xFF;
  }
  if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
    RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
  }
  for (int i = 0; i < 8; i++) {
    can.Data[i] = 0;
  }
  can.Data[0] = 0xAA;
  can.Data[1] = 0x22;
  for (int i = 2; i < 8; i++) {
    can.Data[i] = 0xFF;
  }
  if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
    RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
  }
  RCLCPP_INFO(this->get_logger(), "brush start!!!!!!!!!!!!!!!!!!!\n");
  rclcpp::Time t1 = rclcpp::Node::now();
  rclcpp::Time t2 = rclcpp::Node::now();
  bool rec = false;
  while ((t2.seconds() - t1.seconds()) < 2) {
    CAN_OBJ can1[1];
    int cnt;  // current received
    cnt = Receive(gDevType1_, gDevIdx1_, gChMask1_, can1, 1, 2);
    if (cnt == 1) {
      rec = true;
      break;
    }
    t2 = rclcpp::Node::now();
  }
  if (!rec) {
    RCLCPP_INFO(this->get_logger(), "resend start!!!!!!!!!!!!!!!!!!!\n");
    if (!ResetCAN(gDevType1_, gDevIdx1_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN1 failed\n");
    }
    if (!CloseDevice(gDevType1_, gDevIdx1_)) {
      RCLCPP_INFO(this->get_logger(), "CloseCAN1 failed\n");
    }
    int tep = gDevIdx1_;
    gDevIdx1_ = gDevIdx2_;
    gDevIdx2_ = tep;
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
    config.AccCode = 0;
    config.AccMask = 0xffffff;
    config.Filter = 0;
    config.Mode = 0;
    config.Timing0 = gBaud1_ & 0xff;
    config.Timing1 = gBaud1_ >> 8;
    if (!InitCAN(gDevType1_, gDevIdx1_, 0, &config)) {
      RCLCPP_INFO(this->get_logger(), "InitDevice1 failed\n");
      return 0;
    }
    if (!StartCAN(gDevType1_, gDevIdx1_, 0)) {
      RCLCPP_INFO(this->get_logger(), "StartCAN1 failed\n");
      return 0;
    }
    memset(&can, 0, sizeof(CAN_OBJ));
    can.ID = 0x0CFF1C65;
    can.SendType = gTxType1_;
    can.ExternFlag = 1;
    can.RemoteFlag = 0;
    can.DataLen = 8;
    for (int i = 0; i < 8; i++) {
      can.Data[i] = 0;
    }
    for (int i = 0; i < 2; i++) {
      can.Data[i] = 0xAA;
    }
    for (int i = 2; i < 8; i++) {
      can.Data[i] = 0x00;
    }
    if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
      RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
    }
    RCLCPP_INFO(this->get_logger(), "send vel start!!!!!!!!!!!!!!!!!!!\n");
    can.ID = 0x18FF8017;
    can.SendType = gTxType1_;
    can.ExternFlag = 1;
    can.RemoteFlag = 0;
    can.DataLen = 8;
    for (int i = 0; i < 8; i++) {
      can.Data[i] = 0;
    }
    can.Data[0] = 0x55;
    can.Data[1] = 0x21;
    for (int i = 2; i < 8; i++) {
      can.Data[i] = 0xFF;
    }
    if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
      RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
    }
    for (int i = 0; i < 8; i++) {
      can.Data[i] = 0;
    }
    can.Data[0] = 0xAA;
    can.Data[1] = 0x21;
    for (int i = 2; i < 8; i++) {
      can.Data[i] = 0xFF;
    }
    if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
      RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
    }
    for (int i = 0; i < 8; i++) {
      can.Data[i] = 0;
    }
    can.Data[0] = 0xAA;
    can.Data[1] = 0x22;
    for (int i = 2; i < 8; i++) {
      can.Data[i] = 0xFF;
    }
    if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
      RCLCPP_INFO(this->get_logger(), "TYPE1 transmit failed\n");
    }
    RCLCPP_INFO(this->get_logger(), "brush start!!!!!!!!!!!!!!!!!!!\n");

    if (!ResetCAN(gDevType2_, gDevIdx2_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN2 failed\n");
    }
    if (!CloseDevice(gDevType2_, gDevIdx2_)) {
      RCLCPP_INFO(this->get_logger(), "closeCAN2 failed\n");
    }
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
      return 1;
    }
    if (!StartCAN(gDevType2_, gDevIdx2_, 0)) {
      RCLCPP_INFO(this->get_logger(), "StartCAN2 failed\n");
      return 1;
    }
    return 2;
  } else {
    if (!ResetCAN(gDevType2_, gDevIdx2_, 0)) {
      RCLCPP_INFO(this->get_logger(), "ResetCAN2 failed\n");
    }
    if (!CloseDevice(gDevType2_, gDevIdx2_)) {
      RCLCPP_INFO(this->get_logger(), "closeCAN2 failed\n");
    }
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
      return 1;
    }
    if (!StartCAN(gDevType2_, gDevIdx2_, 0)) {
      RCLCPP_INFO(this->get_logger(), "StartCAN2 failed\n");
      return 1;
    }
    return 2;
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
        can.SendType = gTxType1_;
        can.ExternFlag = 1;
        can.RemoteFlag = 0;
        can.DataLen = 8;
        for (int i = 0; i < 8; i++) {
          can.Data[i] = msg.payload[i];
        }
        if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
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
        can.SendType = gTxType2_;
        can.ExternFlag = 0;
        can.RemoteFlag = 0;
        can.DataLen = 8;
        for (int i = 0; i < 8; i++) {
          can.Data[i] = msg.payload[i];
        }
        if (Transmit(gDevType2_, gDevIdx2_, 0, &can, 1) == 0) {
          RCLCPP_INFO(this->get_logger(), "TYPE2 transmit failed\n");

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
              case 0:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 0;
                break;
              case 1:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[1] = 32;
                can.Data[3] = 1;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 0;
                break;
              case 2:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[1] = 32;
                can.Data[3] = 5;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 0;
                break;
              default:
                break;
            }
          }
          else if(light_state != light_cmd && light_cmd == 1){
            switch (brush_state) {
              case 0:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 1;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x02;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 1;
                break;
              case 1:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 1;
                can.Data[1] = 32;
                can.Data[3] = 1;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x02;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 1;
                break;
              case 2:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 1;
                can.Data[1] = 32;
                can.Data[3] = 5;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x02;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 1;
                break;
              default:
                break;
            }
          }
          else if(light_state != light_cmd && light_cmd == 2){
            switch (brush_state) {
              case 0:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[3] = 0x80;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x01;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 2;
                break;
              case 1:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[1] = 32;
                can.Data[3] = 1;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[3] = 0x80;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x01;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 2;
                break;
              case 2:
                can.ID = 0x18FA1117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[1] = 32;
                can.Data[3] = 5;
                can.Data[3] = can.Data[3] + guard_bit;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18F21117;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[3] = 0x80;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                can.ID = 0x18FA6218;
                can.SendType = gTxType1_;
                can.ExternFlag = 1;
                can.RemoteFlag = 0;
                can.DataLen = 8;
                for (int i = 0; i < 8; i++) {
                  can.Data[i] = 0;
                }
                can.Data[0] = 0x01;
                can.Data[6] = 0x3f;
                can.Data[7] = 0x52;
                Transmit(gDevType1_, gDevIdx1_, 0, &can, 1);
                light_state = 2;
                break;
              default:
                break;
            }
          }
          else{
          }
        }
      } else if (is_data == 3) {
        can.SendType = gTxType1_;
        can.ExternFlag = 1;
        can.RemoteFlag = 0;
        can.DataLen = 8;
        for (int i = 0; i < 8; i++) {
          can.Data[i] = msg.payload[i];
        }
        can.Data[3] = can.Data[3] + guard_bit;
        if (Transmit(gDevType1_, gDevIdx1_, 0, &can, 1) == 0) {
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

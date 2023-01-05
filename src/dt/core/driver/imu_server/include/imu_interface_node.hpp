#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define DL_HEADER1                        0x5a
#define DL_HEADER2                        0x5a
#define DL_CLASS_MINIAHRS                 0x0f
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS  0x01

#define DL_PAYLOAD_LENGTH                 56
#define DL_CHECK_LENGTH                   57
#define DL_CHECKSUM_LENGTH                0x02

#define DL_NO_ERR                         0x00
#define DL_UNKNOW_MESSAGE                 0x01
#define DL_CHECKSUM_ERR                   0x02
#define DL_PAYLOAD_LENGTH_ERR             0x04

#define COEF_DEG_TO_RAD                   57.29578
#define g                                 9.8
typedef struct imu102n
{
  union{
    float fGx;
    int32_t int_gx;
  }Gx;
    union{
    float fGy;
    int32_t int_gy;
  }Gy;
    union{
    float fGz;
    int32_t int_gz;
  }Gz;
    union{
    float fAx;
    int32_t int_ax;
  }Ax;
    union{
    float fAy;
    int32_t int_ay;
  }Ay;
    union{
    float fAz;
    int32_t int_az;
  }Az;
}imu102n_t;
using namespace std;



class ImuInterfaceNode : public rclcpp::Node
{
public:



  /**
   * @brief constructor
   */
  explicit ImuInterfaceNode(const rclcpp::NodeOptions & options);

private:
  // Set up ROS.
  string port_;
  int baud_;
  sensor_msgs::msg::Imu refe;
  bool started ;
  int num ;
  rclcpp::Time time_now;

  serial::Serial serial_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  //bool check_eq(serial::Serial* serial_1, uint8_t num);
  //void fetch_payload(serial::Serial* serial_, uint8_t* payload);
  bool check_eq( uint8_t num);
  uint8_t convert_byte(uint8_t * ptr);
  bool imu_checksum(uint8_t * payload);
  float convert_float(uint8_t * ptr);
  void convert_to_msg(sensor_msgs::msg::Imu& msg, uint8_t * payload);
  void fetch_payload(uint8_t* payload);
  void on_timer();
};



bool ImuInterfaceNode::check_eq( uint8_t num){
  uint8_t buffer;
  this->serial_.read(&buffer, 1);
  if (buffer == num){
    return true;
  }else{
    return false;
  }
}

uint8_t ImuInterfaceNode::convert_byte(uint8_t * ptr){
  uint8_t f;
  memcpy(&f, ptr, 1);
  return f;
}

bool ImuInterfaceNode::imu_checksum(uint8_t * payload){
  uint8_t checksum = 0;
  for (int i=0; i<DL_PAYLOAD_LENGTH; i++){
    checksum += payload[i];
  }
  return checksum == payload[DL_CHECK_LENGTH-1];
}
float ImuInterfaceNode::convert_float(uint8_t * ptr){
  float f;
  memcpy(&f, ptr, 4);
  return f;
}


void ImuInterfaceNode::convert_to_msg(sensor_msgs::msg::Imu& msg, uint8_t * payload){
  imu102n_t imu_bd;
  msg.header.frame_id = "imu_link";
  msg.header.stamp = rclcpp::Node::now();

  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;
  msg.orientation.w = 1;
  // msg.orientation_covariance[0] = -1.0;
  imu_bd.Gx.int_gx = payload[3] << 24 | payload[2] << 16 | payload[1] << 8 | payload[0] << 0;
  imu_bd.Gy.int_gy = payload[7] << 24 | payload[6] << 16 | payload[5] << 8 | payload[4] << 0;
  imu_bd.Gz.int_gz = payload[11] << 24 | payload[10] << 16 | payload[9] << 8 | payload[8] << 0;
  imu_bd.Ax.int_ax = payload[15] << 24 | payload[14] << 16 | payload[13] << 8 | payload[12] << 0;
  imu_bd.Ay.int_ay = payload[19] << 24 | payload[18] << 16 | payload[17] << 8 | payload[16] << 0;
  imu_bd.Az.int_az = payload[23] << 24 | payload[22] << 16 | payload[21] << 8 | payload[20] << 0;
  msg.angular_velocity.x = (double)imu_bd.Gx.fGx / COEF_DEG_TO_RAD;
  msg.angular_velocity.y = (double)imu_bd.Gy.fGy / COEF_DEG_TO_RAD;
  msg.angular_velocity.z = (double)imu_bd.Gz.fGz / COEF_DEG_TO_RAD;

  msg.linear_acceleration.x = (double)imu_bd.Ax.fAx * g;
  msg.linear_acceleration.y = (double)imu_bd.Ay.fAy * g;
  msg.linear_acceleration.z = (double)imu_bd.Az.fAz * g;


}
//void ImuInterfaceNode::fetch_payload(serial::Serial* serial_, uint8_t* payload){
void ImuInterfaceNode::fetch_payload( uint8_t* payload){
  unsigned char state = 0;

    while(1){
      switch (state){
      case 0:{ // Header 1
        //state = check_eq(serial_, DL_HEADER1) ? 1 : 0;
        state = check_eq( DL_HEADER1) ? 1 : 0;
     
        break;
      }case 1:{ // Header 2
        //state = check_eq(serial_, DL_HEADER2) ? 2 : 0;
        state = check_eq( DL_HEADER2) ? 2 : 0;
   
        break;
      }case 2:{ // PAYLOAD
        size_t read_payload_size = this->serial_.read(payload, (int)DL_CHECK_LENGTH);
        state = read_payload_size == DL_CHECK_LENGTH ? 3 : 0;
        break;
      }case 3:{ // CHECKSUM
        state = imu_checksum(payload) ? 4 : 0;
        break;
      }case 4:{
        state = 0;
        return;
      }default:{
        state = 0;
        break;
      }
      }
    
  }
}


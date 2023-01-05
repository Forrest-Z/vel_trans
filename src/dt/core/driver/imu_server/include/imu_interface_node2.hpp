#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

#define DL_HEADER1                        0xaa
#define DL_HEADER2                        0x36
#define DL_CLASS_MINIAHRS                 0x0f
#define DL_MINIAHRS_ATTITUDE_AND_SENSORS  0x01

#define DL_PAYLOAD_LENGTH                 23
#define DL_CHECK_LENGTH                   24


#define COEF_DEG_TO_RAD                   200 / (30000 * 57.29578)
#define g                                 9.8 * 12 / 30000
typedef struct imu102n
{
  union{
    float fGx;
    signed short int_gx;
  }Gx;
    union{
    float fGy;
    signed short int_gy;
  }Gy;
    union{
    float fGz;
    signed short int_gz;
  }Gz;
    union{
    float fAx;
    signed short int_ax;
  }Ax;
    union{
    float fAy;
    signed short int_ay;
  }Ay;
    union{
    float fAz;
    signed short int_az;
  }Az;
}imu102n_t;
using namespace std;



class ImuInterfaceNode2 : public rclcpp::Node
{
public:



  /**
   * @brief constructor
   */
  explicit ImuInterfaceNode2(const rclcpp::NodeOptions & options);

private:
  // Set up ROS.
  string port_;
  int baud_;
  sensor_msgs::msg::Imu refe;
  bool started ;
  int num ;
  rclcpp::Time time_now;
  std::string topic_name;
  serial::Serial serial_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer_;

  //bool check_eq(serial::Serial* serial_1, uint8_t num);
  //void fetch_payload(serial::Serial* serial_, uint8_t* payload);
  bool check_eq( uint8_t num);
  bool imu_checksum(uint8_t * payload);
  void convert_to_msg(sensor_msgs::msg::Imu& msg, uint8_t * payload);
  void fetch_payload(uint8_t* payload);
  void on_timer();
};



bool ImuInterfaceNode2::check_eq(uint8_t num){
  uint8_t buffer;
  this->serial_.read(&buffer, 1);
  if (buffer == num){
    return true;
  }else{
    return false;
  }
}



bool ImuInterfaceNode2::imu_checksum(uint8_t * payload){
  uint8_t checksum = 0;
  for (int i=0; i<DL_PAYLOAD_LENGTH; i++){
    checksum += payload[i];
  }
  return checksum == payload[DL_CHECK_LENGTH-1];
}



void ImuInterfaceNode2::convert_to_msg(sensor_msgs::msg::Imu& msg, uint8_t * payload){
  imu102n_t imu_bd;
  msg.header.frame_id = "imu_link";
  msg.header.stamp = rclcpp::Node::now();

  msg.orientation.x = 0;
  msg.orientation.y = 0;
  msg.orientation.z = 0;
  msg.orientation.w = 1;
  // msg.orientation_covariance[0] = -1.0;
  imu_bd.Gz.int_gz = payload[2] << 8 | payload[1] << 0;
  imu_bd.Gx.int_gx = payload[4] << 8 | payload[3] << 0;
  imu_bd.Gy.int_gy = payload[6] << 8 | payload[5] << 0;
  imu_bd.Az.int_az = payload[8] << 8 | payload[7] << 0;
  imu_bd.Ax.int_ax = payload[10] << 8 | payload[9] << 0;
  imu_bd.Ay.int_ay = payload[12] << 8 | payload[11] << 0;
  msg.angular_velocity.x = (double)imu_bd.Gx.int_gx * COEF_DEG_TO_RAD;
  msg.angular_velocity.y = (double)imu_bd.Gy.int_gy * COEF_DEG_TO_RAD;
  msg.angular_velocity.z = (double)imu_bd.Gz.int_gz * COEF_DEG_TO_RAD;

  msg.linear_acceleration.x = (double)imu_bd.Ax.int_ax * g;
  msg.linear_acceleration.y = (double)imu_bd.Ay.int_ay * g;
  msg.linear_acceleration.z = (double)imu_bd.Az.int_az * g;


}
//void ImuInterfaceNode::fetch_payload(serial::Serial* serial_, uint8_t* payload){
void ImuInterfaceNode2::fetch_payload( uint8_t* payload){
  unsigned char state = 0;

    while(1){
      switch (state){
      case 0:{ // Header 1
        //state = check_eq(serial_, DL_HEADER1) ? 1 : 0;
        state = check_eq( DL_HEADER1) ? 1 : 0;
        break;
      }case 1:{ // Header 2
        size_t read_payload_size = this->serial_.read(payload, (int)DL_CHECK_LENGTH);
        state = read_payload_size == DL_CHECK_LENGTH ? 2 : 0;
        break;
      }case 2:{ // PAYLOAD
        state = DL_HEADER2 == payload[0]? 3 : 0;
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


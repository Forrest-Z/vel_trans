#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <serial/serial.h>
#include <tf2/LinearMath/Quaternion.h>
#include <unistd.h>
#include "nav_msgs/msg/odometry.hpp"
#include <algorithm>
#include <vector>
#include <stdint.h>
#include <eigen3/Eigen/Eigen>
#include <osgEarth/GeoData>
using namespace std;

const double epsilon = 0.000000000000001;
const double pi = 3.14159265358979323846;
const double d2r = pi / 180;
const double r2d = 180 / pi;
const double a = 6378137.0;		//椭球长半轴
const double f_inverse = 298.257223563;			//扁率倒数
const double b = a - a / f_inverse;
//const double b = 6356752.314245;			//椭球短半轴
const double e = sqrt(a * a - b * b) / a;

class FixTransNode : public rclcpp::Node
{
public:
  /**
   * @brief constructor
   */
  explicit FixTransNode(const rclcpp::NodeOptions & options);


  ~FixTransNode();



private:
  // Set up ROS.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  void handle_odom_msg(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
  void Blh2Xyz(double &x, double &y, double &z)
  {
    double L = x * d2r;
    double B = y * d2r;
    double H = z;
    double N = a / sqrt(1 - e * e * sin(B) * sin(B));
    x = (N + H) * cos(B) * cos(L);
    y = (N + H) * cos(B) * sin(L);
    z = (N * (1 - e * e) + H) * sin(B);
  }

  void CalEcef2Enu(Eigen::Vector3d& topocentricOrigin, Eigen::Matrix4d& resultMat)
  {
    double rzAngle = -(topocentricOrigin.x() * d2r + pi / 2);
    Eigen::AngleAxisd rzAngleAxis(rzAngle, Eigen::Vector3d(0, 0, 1));
    Eigen::Matrix3d rZ = rzAngleAxis.matrix();
    double rxAngle = -(pi / 2 - topocentricOrigin.y() * d2r);
    Eigen::AngleAxisd rxAngleAxis(rxAngle, Eigen::Vector3d(1, 0, 0));
    Eigen::Matrix3d rX = rxAngleAxis.matrix();
    Eigen::Matrix4d rotation;
    rotation.setIdentity();
    rotation.block<3, 3>(0, 0) = (rX * rZ);
    //cout << rotation << endl;
    double tx = topocentricOrigin.x();
    double ty = topocentricOrigin.y();
    double tz = topocentricOrigin.z();
    Blh2Xyz(tx, ty, tz);
    Eigen::Matrix4d translation;
    translation.setIdentity();
    translation(0, 3) = -tx;
    translation(1, 3) = -ty;
    translation(2, 3) = -tz;
    resultMat = rotation * translation;
  }
};

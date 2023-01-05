#include "sonar_control_node.hpp"

SonarControlNode::SonarControlNode(const rclcpp::NodeOptions & options)
: Node("sonar_control_node", options), updater_(this)
{
  using std::placeholders::_1;
  is_FB_close = false;
  is_LR_close = false;
  // Node Parameter
  sonar_distance_LR_ = declare_parameter("sonar_distance_LR", 0.35);
  sonar_distance_FB_ = declare_parameter("sonar_distance_FB", 0.35);
  is_LR_check_ = declare_parameter("is_LR_check", true);
  is_FB_check_ = declare_parameter("is_FB_check", true);

  // Subscriber
  sub_sonar_lr_ = this->create_subscription<std_msgs::msg::Float32>(
    "lr_sonar_dist", 1, std::bind(&SonarControlNode::checkSonarDistanceLR, this, _1));
  sub_sonar_fb_ = this->create_subscription<std_msgs::msg::Float32>(
    "fb_sonar_dist", 1, std::bind(&SonarControlNode::checkSonarDistanceFB, this, _1));

  // Diagnostic Updater
  updater_.setHardwareID("sonar_distance_control");
  updater_.add("sonar_distance_control", this, &SonarControlNode::RefreshStatus);
}

void SonarControlNode::checkSonarDistanceLR(std_msgs::msg::Float32::SharedPtr msg)
{
  const bool value = msg->data < sonar_distance_LR_ && msg->data > 0.01;
  LR_distance = msg->data;
  is_LR_close = value ? true : false;
  updater_.force_update();
}

void SonarControlNode::checkSonarDistanceFB(std_msgs::msg::Float32::SharedPtr msg)
{
  const bool value = msg->data < sonar_distance_FB_ && msg->data > 0.01;
  FB_distance = msg->data;
  is_FB_close = value ? true : false;
  updater_.force_update();
}

void SonarControlNode::RefreshStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  using DiagStatus = diagnostic_msgs::msg::DiagnosticStatus;
  // stat.add("here is min distance of left and right : ", LR_distance );
  if (is_LR_close && is_FB_close && is_FB_check_ && is_LR_check_) {
    stat.addf("LR_distance", "%.3f", LR_distance);
    stat.addf("FB_distance", "%.3f", FB_distance);
    stat.summary(DiagStatus::ERROR, "left or right, and front or behind sonar distance ERROR!!!");
  } else if (is_LR_close && is_LR_check_) {
    stat.addf("LR_distance", "%.3f", LR_distance);
    stat.summary(DiagStatus::ERROR, "left or right sonar distance ERROR!!!");
  } else if (is_FB_close && is_FB_check_) {
    stat.addf("FB_distance", "%.3f", FB_distance);
    stat.summary(DiagStatus::ERROR, "front or behind sonar distance ERROR!!!");
  } else {
    stat.summary(DiagStatus::OK, "OK");
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(SonarControlNode)

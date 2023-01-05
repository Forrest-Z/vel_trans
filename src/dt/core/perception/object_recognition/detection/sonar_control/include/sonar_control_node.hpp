#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>

#include <string>
class SonarControlNode : public rclcpp::Node
{
public:
  explicit SonarControlNode(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sonar_lr_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_sonar_fb_;
  void checkSonarDistanceLR(std_msgs::msg::Float32::SharedPtr msg);
  void checkSonarDistanceFB(std_msgs::msg::Float32::SharedPtr msg);
  void RefreshStatus(diagnostic_updater::DiagnosticStatusWrapper & stat);
  float sonar_distance_LR_, sonar_distance_FB_;
  float LR_distance, FB_distance;
  bool is_LR_close, is_FB_close;
  bool is_LR_check_, is_FB_check_;
  // String error_direction_;

  // Diagnostic Updater
  diagnostic_updater::Updater updater_{this};
  // diagnostic_updater::Updater updater_fb_{this};
};


#include "path_rviz_plugin/path_rviz_plugin.hpp"
#include <QString>
#include <QVBoxLayout>
#include <rviz_common/display_context.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rviz_plugins
{
PathRvizPlugin::PathRvizPlugin(QWidget *parent) : rviz_common::Panel(parent) {
    auto *button_layout = new QHBoxLayout;
    start_record_button_ = new QPushButton("录制路径");
    button_layout->addWidget(start_record_button_);
    stop_record_button_ = new QPushButton("保存路径");
    button_layout->addWidget(stop_record_button_);

    start_task_button_ = new QPushButton("开始任务");
    button_layout->addWidget(start_task_button_);
    stop_task_button_ = new QPushButton("取消任务");
    button_layout->addWidget(stop_task_button_);
    setLayout(button_layout);

    stop_record_button_->setEnabled(false);

    connect(start_record_button_, SIGNAL(clicked()), SLOT(start_record_callback()));
    connect(stop_record_button_, SIGNAL(clicked()), SLOT(stop_record_callback()));
    connect(start_task_button_, SIGNAL(clicked()), SLOT(start_task_callback()));
    connect(stop_task_button_, SIGNAL(clicked()), SLOT(stop_task_callback()));

    setLayout(button_layout);

}

void PathRvizPlugin::onInitialize()
{
  raw_node_ = this->getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();

  start_record_client_ = raw_node_->create_client<path_server::srv::SetPathName>(
    "/start_record_path", rmw_qos_profile_services_default);
  stop_record_client_ = raw_node_->create_client<path_server::srv::SetPathName>(
    "/stop_record_path", rmw_qos_profile_services_default);
  engage_pub_ = raw_node_->create_publisher<autoware_vehicle_msgs::msg::Engage>("/vehicle/engage", rclcpp::QoS(1));
}

void PathRvizPlugin::start_record_callback() {
    auto msg = std::make_shared<path_server::srv::SetPathName::Request>();
    msg->path_name = "default_path";
    if (!start_record_client_->service_is_ready()) {
      RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
      return;
    };

    auto future = start_record_client_->async_send_request(
    msg, [this](rclcpp::Client<path_server::srv::SetPathName>::SharedFuture) {
        stop_record_button_->setEnabled(true);
        start_task_button_->setEnabled(false);
    });
    if (!future.valid()) {
      RCLCPP_ERROR(raw_node_->get_logger(), "录制路径发生错误，请重试！");
    }
}

void PathRvizPlugin::stop_record_callback() {
    auto msg = std::make_shared<path_server::srv::SetPathName::Request>();
    msg->path_name = "default_path";
    if (!stop_record_client_->service_is_ready()) {
      RCLCPP_INFO(raw_node_->get_logger(), "client is unavailable");
      return;
    };

    auto future = stop_record_client_->async_send_request(
    msg, [this](rclcpp::Client<path_server::srv::SetPathName>::SharedFuture) {
        start_task_button_->setEnabled(true);
        stop_record_button_->setEnabled(false);
    });
    if (!future.valid()) {
      RCLCPP_ERROR(raw_node_->get_logger(), "保存路径发生错误，请重试！");
    }
}

void PathRvizPlugin::start_task_callback() {
    autoware_vehicle_msgs::msg::Engage engage;
    engage.stamp = raw_node_->get_clock()->now();
    engage.engage = true;
    engage_pub_->publish(engage);
}

void PathRvizPlugin::stop_task_callback() {
    // actionlib_msgs::GoalID msg;
    // msg.stamp = ros::Time::now();
    // cancel_task_pub_.publish(msg);
    autoware_vehicle_msgs::msg::Engage engage;
    engage.stamp = raw_node_->get_clock()->now();
    engage.engage = false;
    engage_pub_->publish(engage);
}
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::PathRvizPlugin, rviz_common::Panel)

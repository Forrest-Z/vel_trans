#ifndef PATH_RVIZ_PLUGIN_HPP_
#define PATH_RVIZ_PLUGIN_HPP_

#include <QHBoxLayout>
#include <QPushButton>
#include <QString>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/panel.hpp>
#include <std_srvs/srv/empty.hpp>
#include <path_server/srv/set_path_name.hpp>
#include <autoware_vehicle_msgs/msg/engage.hpp>

namespace rviz_plugins {
class PathRvizPlugin : public rviz_common::Panel {
    Q_OBJECT
    public:
        PathRvizPlugin(QWidget *parent = nullptr);

        void onInitialize() override;

    public Q_SLOTS:

        void start_record_callback();

        void stop_record_callback();

        void start_task_callback();

        void stop_task_callback();

    private:
        rclcpp::Client<path_server::srv::SetPathName>::SharedPtr start_record_client_;
        rclcpp::Client<path_server::srv::SetPathName>::SharedPtr stop_record_client_;
        rclcpp::Publisher<autoware_vehicle_msgs::msg::Engage>::SharedPtr engage_pub_;
        rclcpp::Node::SharedPtr raw_node_;
        QPushButton *start_record_button_;
        QPushButton *stop_record_button_;
        QPushButton *start_task_button_;
        QPushButton *stop_task_button_;
    };
}


#endif //PATH_RVIZ_PLUGIN_HPP_

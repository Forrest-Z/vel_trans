#include "chasu/can_bus_pmdriver_node.hpp"

CanBusPmdriver::CanBusPmdriver(const rclcpp::NodeOptions & node_options) 
: Node("can_bus_pmdriver_node", node_options),
current_vel_l(0.0),
current_vel_r(0.0),
first_send_odom_flag_(true)
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    typedef std::chrono::duration<double,std::ratio<1,1>> second_type;
    // union{
    // float speed;
    // unsigned char  buf[4];
    // } speed_l,speed_r;
    // speed_l.buf[0] = 10;
    // speed_l.buf[1] = 215;
    // speed_l.buf[2] = 35;
    // speed_l.buf[3] = 60;
    // speed_r.buf[0] = 10;
    // speed_r.buf[1] = 215;
    // speed_r.buf[2] = 35;
    // speed_r.buf[3] = 188;
    // RCLCPP_INFO(this->get_logger(), "  vl: %f \n", speed_l.speed); 
    // RCLCPP_INFO(this->get_logger(), "  vr: %f \n", speed_r.speed); 
    odom_frame_ =   declare_parameter("odom_frame", std::string("odom"));
    base_frame_ =    declare_parameter("base_frame", std::string("base_link"));
    rate_ =    declare_parameter("rate", 10);
    speed_cor =   declare_parameter("speed_cor", 1);
    max_speed_ =   declare_parameter("max_speed", 2.0);
    publish_tf_ =  declare_parameter("publish_tf", false);
    wheelbase =  declare_parameter("wheelbase", 0.34);
    wheeltread =  declare_parameter("wheeltread", 0.568);
    tf_broadcaster_ =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odom", 10);
    steer_pub_ = create_publisher<std_msgs::msg::Float64>("~/output/steer", 10);

    canbus_msg_subscriber_ = create_subscription<chasu::msg::CanBusMessage>("~/input/canbus_msg", 100, std::bind(&CanBusPmdriver::handle_canbus_msg, this, _1) );
    twist_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("~/input/cmd_vel", 10, std::bind(&CanBusPmdriver::handle_twist_msg, this, _1));
    canbus_client_ = create_client<chasu::srv::CanBusService>("canbus_server"); 
    odom_timer_ = create_wall_timer( second_type(1.0/rate_),std::bind(&CanBusPmdriver::parse_msg, this));

}


void CanBusPmdriver::handle_canbus_msg(const chasu::msg::CanBusMessage::ConstSharedPtr msg) {
    if((msg->node_type == CANBUS_NODETYPE_ECU) && (!msg->payload.empty())){
        union{
        float speed;
        char  buf[4];
        } agv_speed_l,agv_speed_r;
        for (int i = 0; i < 4; i++){
            agv_speed_l.buf[i] = msg->payload[i];
        }
        for (int i = 4; i < 8; i++){
            agv_speed_r.buf[i-4] = msg->payload[i];
        }
        current_vel_l = agv_speed_l.speed;
        current_vel_r = -1 * agv_speed_r.speed;
        // RCLCPP_INFO(this->get_logger(), "  vl: %f \n", current_vel_l);
        // RCLCPP_INFO(this->get_logger(), "  vr: %f \n", current_vel_r);
    }       
}

void CanBusPmdriver::handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    twist_cache_ = *msg;
    if (twist_cache_.linear.x >= 0){
        twist_cache_.linear.x = std::min(twist_cache_.linear.x , max_speed_);
    }
    else{
        twist_cache_.linear.x = std::max(twist_cache_.linear.x , -max_speed_);
    }
    double baselink_v_anle = tan(twist_cache_.angular.z) * twist_cache_.linear.x / wheelbase;

    double vl = speed_cor * (twist_cache_.linear.x - baselink_v_anle * wheeltread * 0.5);
    double vr = speed_cor * (twist_cache_.linear.x + baselink_v_anle * wheeltread * 0.5);
    auto request = std::make_shared<chasu::srv::CanBusService::Request>();
    request->requests.clear();
    chasu::msg::CanBusMessage target_speed;
    target_speed.node_type = CANBUS_NODETYPE_ECU;
    target_speed.payload.clear();
    for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);   
    }
    int id_l, id_r;
    double ks_l, ks_r;
    for(id_l = 0;id_l < sizeof(pushtable);id_l++){
        if(speedtable[id_l] >= std::fabs(vl)){
            break;
        }
    }
    switch (id_l)
    {
    case 0:  
        ks_l = pushtable[0];
        break;
    case sizeof(pushtable):
        ks_l = pushtable[sizeof(pushtable)-1];
        break;
    default:
        ks_l = pushtable[id_l - 1] + (pushtable[id_l] - pushtable[id_l - 1]) * (std::fabs(vl) - speedtable[id_l - 1]) / (speedtable[id_l] - speedtable[id_l - 1]);
        break;
    }
    // vl = vl / ks_l;
    for(id_r = 0;id_r < 8;id_r++){
        if(speedtable[id_r] >= std::fabs(vr)){
            break;
        }
    }
    switch (id_r)
    {
    case 0:  
        ks_r = pushtable[0];
        break;
    case sizeof(pushtable):
        ks_r = pushtable[sizeof(pushtable)-1];
        break;
    default:
        ks_r = pushtable[id_r - 1] + (pushtable[id_r] - pushtable[id_r - 1]) * (std::fabs(vr) - speedtable[id_r - 1]) / (speedtable[id_r] - speedtable[id_r - 1]);
        break;
    }
    // vr = vr / ks_r;
	union HEX float_num_l, float_num_r;
	float_num_l.num = vl * 1;
    // RCLCPP_INFO(this->get_logger(), "  vl: %f \n", float_num_l.num); 
     
    // float_num_l.num = 0.01;
	for(size_t l = 0; l < 4; l++){	//大端模式顺着来0-4，小端模式逆着来4-0
        target_speed.payload[l] = float_num_l.hex_num[l];
        // RCLCPP_INFO(this->get_logger(), "  num1 %02X \n", float_num_l.hex_num[l]);  	
	}
	float_num_r.num = vr * -1;
    // RCLCPP_INFO(this->get_logger(), "  vr: %f \n", float_num_r.num);
    // float_num_r.num = -0.01;
	for(size_t r = 0; r < 4; r++){	//大端模式顺着来0-4，小端模式逆着来4-0
        target_speed.payload[r+4] = float_num_r.hex_num[r];	
        // RCLCPP_INFO(this->get_logger(), "  num2 %02X \n", float_num_l.hex_num[r]); 
	}
    request->requests.push_back(target_speed);
    canbus_client_->async_send_request(request);  
}

void CanBusPmdriver::parse_msg( ){
    rclcpp::Time now = rclcpp::Node::now();
        if (first_send_odom_flag_) {
            last_send_odom_time_ = now;
            accumulation_x_ = 0.0;
            accumulation_y_ = 0.0;
            accumulation_yaw_ = 0.0;
            last_a = 0.0;
            last_v = 0.0;
            first_send_odom_flag_ = false;
        } else {
            double current_vel = (current_vel_l + current_vel_r) * 0.5; 
            double delta_time = (now - last_send_odom_time_).seconds();
            double delta_dis = current_vel * delta_time;
            double w = (current_vel_r- current_vel_l) / wheeltread;
            double delta_theta = w * delta_time;
            double deltaX, deltaY;
            if (delta_theta == 0) {
                deltaX = delta_dis;
                deltaY = 0.0;
            } else {
                deltaX = delta_dis * (sin(delta_theta) / delta_theta);
                deltaY = delta_dis * ((1 - cos(delta_theta)) / delta_theta);
            }
            accumulation_x_ += (cos(accumulation_yaw_) * deltaX - sin(accumulation_yaw_) * deltaY);
            accumulation_y_ += (sin(accumulation_yaw_) * deltaX + cos(accumulation_yaw_) * deltaY);
            accumulation_yaw_ += delta_theta;
            tf2::Quaternion q;
            q.setRPY(0, 0, accumulation_yaw_);


            if (publish_tf_) {
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped.header.stamp = now;
                transform_stamped.header.frame_id = odom_frame_;
                transform_stamped.child_frame_id = base_frame_;
                transform_stamped.transform.translation.x = accumulation_x_;
                transform_stamped.transform.translation.y = accumulation_y_;
                transform_stamped.transform.translation.z = 0.0;
                transform_stamped.transform.rotation.x = q.x();
                transform_stamped.transform.rotation.y = q.y();
                transform_stamped.transform.rotation.z = q.z();
                transform_stamped.transform.rotation.w = q.w();
                tf_broadcaster_->sendTransform(transform_stamped);
            }
            nav_msgs::msg::Odometry odom_msg;
            odom_msg.header.frame_id = odom_frame_;
            odom_msg.child_frame_id = base_frame_;
            odom_msg.header.stamp = now;
            odom_msg.pose.pose.position.x = accumulation_x_;
            odom_msg.pose.pose.position.y = accumulation_y_;
            odom_msg.pose.pose.position.z = 0;
            odom_msg.pose.pose.orientation.x = q.getX();
            odom_msg.pose.pose.orientation.y = q.getY();
            odom_msg.pose.pose.orientation.z = q.getZ();
            odom_msg.pose.pose.orientation.w = q.getW();
            odom_msg.twist.twist.linear.x = delta_dis / delta_time;
            odom_msg.twist.twist.linear.y = 0;
            odom_msg.twist.twist.angular.z = delta_theta / delta_time;
            odom_pub_->publish(odom_msg); 
            current_angle = atan(w * wheelbase / current_vel);           
            std_msgs::msg::Float64 steer;
            steer.data = current_angle;
            steer_pub_->publish(steer);
            last_send_odom_time_ = now;
        }

}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanBusPmdriver)

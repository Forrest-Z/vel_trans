#include "can_bus/can_bus_pmdriver_node.hpp"

CanBusPmdriver::CanBusPmdriver(const rclcpp::NodeOptions & node_options) 
: Node("can_bus_pmdriver_node", node_options),
current_vel(0.0),
current_angle(0.0),
first_send_odom_flag_(true)
{
    using std::placeholders::_1;
    using std::placeholders::_2;
    typedef std::chrono::duration<double,std::ratio<1,1>> second_type;

    // set parameter
    odom_frame_ =   declare_parameter("odom_frame", std::string("odom"));
    base_frame_ =    declare_parameter("base_frame", std::string("base_link"));
    rate_ =    declare_parameter("rate", 10);
    speed_cor =   declare_parameter("speed_cor", 11.1);
    angle_cor =   declare_parameter("angle_cor", -4);
    max_speed_ =   declare_parameter("max_speed", 2.0);
    angle_acel_ =  declare_parameter("angle_acel", 100);
    max_angle_ =   declare_parameter("max_angle", 30);
    publish_tf_ =  declare_parameter("publish_tf", false);
    wheelbase =  declare_parameter("wheelbase", 0.7945);
    check_model_ =  declare_parameter("check_model", true);
    brake_model_ =  declare_parameter("brake_model", false);
    v_flag =  declare_parameter("v_flag", 0);
    brake_value_ =  declare_parameter("brake_value", 10000);
    status_sonar.level = 0;
    status_laser.level = 0;
    brake_start_ = 2;
    guard_start_ = 2;
    tf_broadcaster_ =  std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/output/odom", 10);
    bms_pub_ = create_publisher<std_msgs::msg::Float64>("~/output/bms", 10);
    steer_pub_ = create_publisher<std_msgs::msg::Float64>("~/output/steer", 10);

    canbus_msg_subscriber_ = create_subscription<can_bus::msg::CanBusMessage>("~/input/canbus_msg", 100, std::bind(&CanBusPmdriver::handle_canbus_msg, this, _1) );
    twist_subscriber_ = create_subscription<geometry_msgs::msg::Twist>("~/input/cmd_vel", 10, std::bind(&CanBusPmdriver::handle_twist_msg, this, _1));
    sonar_sub_ = create_subscription<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics_err", 1000, std::bind(&CanBusPmdriver::diagCallback, this, _1));
    canbus_client_ = create_client<can_bus::srv::CanBusService>("canbus_server"); 
    sub_engage_ = create_subscription<autoware_vehicle_msgs::msg::Engage>("/vehicle/engage", rclcpp::QoS{1}, std::bind(&CanBusPmdriver::onEngage, this, _1));
    odom_timer_ = create_wall_timer( second_type(1.0/rate_),std::bind(&CanBusPmdriver::parse_msg, this));
    brush_timer_ = create_wall_timer(second_type(1.0), std::bind(&CanBusPmdriver::brush_msg, this));
    wash_timer_ = create_wall_timer(second_type(1.0), std::bind(&CanBusPmdriver::wash_msg, this));
    brake_timer_ = create_wall_timer(second_type(1.0), std::bind(&CanBusPmdriver::brake_msg, this));
    // guard_timer_ = create_wall_timer(second_type(1.0), std::bind(&CanBusPmdriver::guard_msg, this));
    control_service_ = create_service<can_bus::srv::ControlService>(
    "wash_brush_server", std::bind(&CanBusPmdriver::controlService, this, _1, _2));


}

void CanBusPmdriver::controlService(
  const std::shared_ptr<can_bus::srv::ControlService::Request> req,
  std::shared_ptr<can_bus::srv::ControlService::Response>)
{
    int caseNumber ;
    caseNumber = 0;
    if(req->type == "wash" ) caseNumber = 1 ;
    if(req->type == "brush" ) caseNumber = 2 ;
    if(req->type == "brake" ) caseNumber = 3 ;
    switch(caseNumber){
        case 1 :
        if(req->start == true) {
            this->wash_start_ = 1;
            RCLCPP_INFO(this->get_logger(), "wash start: %d\n", this->wash_start_);
        }
        else {
            this->wash_start_ = 0;
            RCLCPP_INFO(this->get_logger(), "wash close: %d\n", this->wash_start_);
        }
        break;
    case 2 :
        if(req->start == true) {
            this->brush_start_ = 1;
            RCLCPP_INFO(this->get_logger(), "brush start: %d\n", this->brush_start_);
        }
        else {
            this->brush_start_ = 0;
            RCLCPP_INFO(this->get_logger(), "brush close: %d\n", this->brush_start_);
        }
       break;
    case 3 :
        if(req->start == true) {
            this->brake_start_ = 1;
            RCLCPP_INFO(this->get_logger(), "brake start: %d\n", this->brake_start_);
        }
        else {
            this->brake_start_ = 0;
            RCLCPP_INFO(this->get_logger(), "brake close: %d\n", this->brake_start_);
        }
       break;
    default : {RCLCPP_INFO(this->get_logger(), "error !!!!!!!!1 control order,must brush or wash");
                break;};
}

 }

void CanBusPmdriver::handle_canbus_msg(const can_bus::msg::CanBusMessage::ConstSharedPtr msg) {
        if((msg->node_type == CANBUS_NODETYPE_ECU) && (!msg->payload.empty())){
            current_vel_time = rclcpp::Node::now();
            int low = msg->payload[0];
            int high = msg->payload[1];
            int t_k = ((high << 8)|(low & 0xff));
            if(t_k > 30000){
                t_k = t_k -65535;
            }
            float current_vel_tmp = t_k  * 0.94 / (60 * speed_cor);
            if(fabs(current_vel_tmp) < 3) {
                current_vel = current_vel_tmp;
            }
        }
        else if((msg->node_type == CANBUS_NODETYPE_TCU) && (!msg->payload.empty())){
            if(!check_model_){                
                current_angle_time = rclcpp::Node::now();
                current_angle = (float) (msg->payload[3]*256 + msg->payload[4] - 1024) * M_PI /  (180 * angle_cor);
            }
            else{
                uint8_t x = 0;
                for(int i =0;i < 7;i++){
                    x = x^(msg->payload[i]);   
                }
                if(msg->payload[7] == x){
                    current_angle_time = rclcpp::Node::now();
                    current_angle = (float) (msg->payload[3]*256 + msg->payload[4] - 1024) * M_PI / (180 * angle_cor);
                }
            }           
        }
        else if((msg->node_type == CANBUS_NODETYPE_ST) && (!msg->payload.empty())){
            std_msgs::msg::Float64 batter;
            batter.data = msg->payload[4] * 0.5;
            bms_pub_->publish(batter);         
        }
        
}

void CanBusPmdriver::handle_twist_msg(const geometry_msgs::msg::Twist::ConstSharedPtr msg) {
    twist_cache_ = *msg;
    auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
    request->requests.clear();
    can_bus::msg::CanBusMessage target_speed;
    target_speed.node_type = CANBUS_NODETYPE_ECU;
    target_speed.payload.clear();
    for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);   
    }
    if((status_sonar.level == 2 && engage_msg.engage == true) || (status_laser.level == 2 && engage_msg.engage == true)){
        target_speed.payload[3] = 1;
        target_speed.payload[4] = 1;
        target_speed.payload[5] = 1;
        target_speed.payload[7] = 1;
        v_flag = 0;
        request->requests.push_back(target_speed);
    }
    else{
        double v_t = fabs(twist_cache_.linear.x) * speed_cor  * 60 / (0.94);
        int id;
        for(id = 0;id < 23;id++){
            if(speedtable[id] >= v_t){
                break;
            }
        }
        int push;
        switch (id)
        {
        case 0:
        
            push = 0;
            break;
        case 23:
            push = (v_t - KB) / KA;
    
            break;
        default:
            push = pushtable[id - 1] + (pushtable[id] - pushtable[id - 1]) * (v_t - speedtable[id - 1]) / (speedtable[id] - speedtable[id - 1]);
            break;
        }
        target_speed.payload[2] = (unsigned char) push;
        if(twist_cache_.linear.x > 0){
            target_speed.payload[3] = 0;
        }
        else if(twist_cache_.linear.x < 0){
            target_speed.payload[3] = 1;
        }
        else{
            target_speed.payload[3] = v_flag;
        }
        target_speed.payload[4] = 1;
        if(twist_cache_.linear.x == 0 && brake_model_){
            target_speed.payload[5] = 1;
        }
        target_speed.payload[7] = 1;
        if(fabs(twist_cache_.linear.x) < max_speed_){
            request->requests.push_back(target_speed);
            if(target_speed.payload[3] == 0){
                v_flag = 0;
            }
            else{
                v_flag = 1;
            }
        }
    }
    can_bus::msg::CanBusMessage target_angle;
    target_angle.node_type = CANBUS_NODETYPE_TCU;
    target_angle.payload.clear();
    for(int i = 0;i < 8;i++){
        target_angle.payload.push_back(0);   
    }
    target_angle.payload[0] = 0x20;
    int seita;
    seita = angle_cor * twist_cache_.angular.z * 180 / M_PI;
    if(seita >= 0){
            target_angle.payload[3] = 4;
            if(fabs(seita) > max_angle_){
                seita = max_angle_;
            }
            target_angle.payload[4] = (unsigned char)fabs(seita);
        }
    else{
        target_angle.payload[3] = 3;
            if(fabs(seita) > max_angle_){
                seita = -1 * max_angle_;
            }
        target_angle.payload[4] = (unsigned char)(256 - fabs(seita));
    }
    target_angle.payload[5] = 0x55;
    target_angle.payload[6] = angle_acel_;
    uint8_t x = 0;
    for(int i = 0;i < 7;i++){
        x = x^(target_angle.payload[i]);   
    }
    target_angle.payload[7] = x;
    request->requests.push_back(target_angle);
    canbus_client_->async_send_request(request);  
}
void CanBusPmdriver::onEngage(const autoware_vehicle_msgs::msg::Engage::ConstSharedPtr msg){
    engage_msg = *msg;   
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
            double delta_time = (now - last_send_odom_time_).seconds();
            double delta_dis = (current_vel + last_v) * 0.5 * (now - last_send_odom_time_).seconds();
            last_v = current_vel;
            double baselink_v_anle = tan((current_angle + last_a) * 0.5) * current_vel / wheelbase;
            last_a = current_angle;
            double delta_theta = baselink_v_anle * (now - last_send_odom_time_).seconds();
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
            std_msgs::msg::Float64 steer;
            steer.data = current_angle;
            steer_pub_->publish(steer);
            last_send_odom_time_ = now;
        }

}

void CanBusPmdriver::brush_msg( ){

    if(brush_start_ == 1){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        target_speed.payload[1] = 32;
        target_speed.payload[3] = 1;
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        brush_start_ = 2;
    }
    else if(brush_start_ == 0){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();;
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        brush_start_ = 2;
    }
    else{
    }
    
}

void CanBusPmdriver::wash_msg( ){

    if(wash_start_ == 1){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        target_speed.payload[1] = 32;
        target_speed.payload[3] = 5;
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        wash_start_ = 2;
    }
    else if(wash_start_ == 0){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        wash_start_ = 2;
    }
    else{
    }

}
void CanBusPmdriver::brake_msg( ){

    if(brake_start_ == 1){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        target_speed.payload[0] = 2;
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        brake_start_ = 2;
    }
    else if(brake_start_ == 0){
        auto request = std::make_shared<can_bus::srv::CanBusService::Request>();
        request->requests.clear();
        can_bus::msg::CanBusMessage target_speed;
        target_speed.node_type = 3;
        target_speed.payload.clear();
        for(int i = 0;i < 8;i++){
        target_speed.payload.push_back(0);
        }
        request->requests.push_back(target_speed);
        canbus_client_->async_send_request(request); 
        brake_start_ = 2;
    }
    else{
    }

}

void CanBusPmdriver::diagCallback(const diagnostic_msgs::msg::DiagnosticArray::ConstSharedPtr diag_msg)
{
    array_ = *diag_msg;
    findDiagStatus("/autoware/perception/autonomous_driving/performance_monitoring/sonar_control/sonar_control_node: sonar_distance_control", status_sonar);
    findDiagStatus("/autoware/planning/performance_monitoring/trajectory_validation/surround_obstacle_checker/surround_obstacle_checker: surround_obstacle_checker", status_laser);
}
bool CanBusPmdriver::findDiagStatus(const std::string & name, DiagStatus & status)  
{
    for (size_t i = 0; i < array_.status.size(); ++i) {
        if (array_.status[i].name == name) {
            status = array_.status[i];
            return true;
        }
    }
    return false;
}
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CanBusPmdriver)

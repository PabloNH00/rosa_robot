#include "rosa_driver.h"

#include <string>
using std::string;

namespace ROSA{
  
  // Function to initialize the driver
  void RosaDriver::init()
  {
    auto node_ptr = shared_from_this();
    // Retrieve the parameters from the parameter server/////
    ///// PARAM1: device name/////////////////////////////////
    string port_name{"/dev/ttyUSB1"};
    try {
      port_name = declare_parameter<string>("device_name", port_name);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
      throw ex;
    }
    /////PARAM2: baud_rate////////////////////////////////////
    uint32_t baud_rate{115200};
    try {
      baud_rate = declare_parameter<int>("baud_rate", baud_rate);
    } catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
      throw ex;
    }
    ///// PARAM3: odometry frame/////////////////////////////////
    frame_id = "odom";
    try {
      frame_id = declare_parameter<string>("odometryFrame", frame_id);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The odometry frame provided was invalid");
      throw ex;
    }
    ///// PARAM4: odometry topic/////////////////////////////////
    odometry_topic = "odom";
    try {
      odometry_topic = declare_parameter<string>("odometryTopic", odometry_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The odometry topic provided was invalid");
      throw ex;
    }
    ///// PARAM5: robot base frame/////////////////////////////////
    robot_frame_id = "base_footprint";
    try {
      robot_frame_id = declare_parameter<string>("robotBaseFrame", robot_frame_id);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The robot base frame provided was invalid");
      throw ex;
    }
    cmd_topic = "cmd_vel";
    try {
      cmd_topic = declare_parameter<string>("commandTopic", cmd_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The command topic provided was invalid");
      throw ex;
    }
    curr_vel_topic = "current_vel";
    try {
      curr_vel_topic = declare_parameter<string>("curr_vel_topic", curr_vel_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The curr_vel_topic provided was invalid");
      throw ex;
    }
    target_vel_topic = "target_vel";
    try {
      target_vel_topic = declare_parameter<string>("target_vel_topic", target_vel_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The target_vel_topic provided was invalid");
      throw ex;
    }
    encoder_count_topic = "encoder_count";
    try {
      encoder_count_topic = declare_parameter<string>("encoder_count_topic", encoder_count_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The encoder_count_topic provided was invalid");
      throw ex;
    }
    battery_topic = "battery";
    try {
      battery_topic = declare_parameter<string>("battery_topic", battery_topic);
    }catch (rclcpp::ParameterTypeException & ex) {
      RCLCPP_ERROR(get_logger(), "The battery_topic provided was invalid");
      throw ex;
    }
    
    //initialize SerialPort handler
    
    hw_ptr = std::make_shared<RosaPort>(node_ptr,port_name, baud_rate);

    //initialize suscribers and publishers and messages
    odom_msg.header.frame_id = frame_id;
    transform_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = robot_frame_id;
    transform_msg.child_frame_id = robot_frame_id;
    odometry_pub = create_publisher<nav_msgs::msg::Odometry>("/" + odometry_topic, 2);
    current_vel_pub = create_publisher<std_msgs::msg::Int32MultiArray>("/" + curr_vel_topic, 2);
    target_vel_pub = create_publisher<std_msgs::msg::Int32MultiArray>("/" + target_vel_topic, 2);
    encoder_count_pub = create_publisher<std_msgs::msg::Int32MultiArray>("/" + encoder_count_topic, 2);
    battery_pub = create_publisher<std_msgs::msg::Float32>("/" + battery_topic, 2);
    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    //suscriptors
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>("/" + cmd_topic, 
    rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
    std::bind(&RosaDriver::cmd_vel_callback, this, std::placeholders::_1));

   
    // Create a timer to read data from the robot 
    _timer = this->create_wall_timer(
            std::chrono::milliseconds(20),
            std::bind(&RosaDriver::loop, this));
  }

  // Function to update the node
  void RosaDriver::loop()
  {
    while(hw_ptr->there_is_message()){
      auto m = hw_ptr->get_message();
      if(m.id == ROSA_ROBOT_DATA)
      {
        RobotData rd;

        auto intarray32 = std_msgs::msg::Int32MultiArray();
        auto battery = std_msgs::msg::Float32();

        m.read_array<int32_t>(rd.current_velocity, 4);
        intarray32.data = {rd.current_velocity[0], rd.current_velocity[1], rd.current_velocity[2], rd.current_velocity[3]};
        current_vel_pub->publish(intarray32);

        m.read_array<int32_t>(rd.target_velocity, 4);
        intarray32.data = {rd.target_velocity[0], rd.target_velocity[1], rd.target_velocity[2], rd.target_velocity[3]};
        target_vel_pub->publish(intarray32);

        m.read_array<int32_t>(rd.encoder_counts, 4);
        intarray32.data = {rd.encoder_counts[0], rd.encoder_counts[1], rd.encoder_counts[2], rd.encoder_counts[3]};
        encoder_count_pub->publish(intarray32);

        rd.battery_voltage = m.read<float>();
        battery.data = rd.battery_voltage;
        battery_pub->publish(battery);
        
      }
      if(m.id == ROSA_ODOMETRY){
        float pos_x=m.read<float>();
        float pos_y=m.read<float>();
        float yaw=m.read<float>();
        float lin_vel_x =0;
        float lin_vel_y =0;
        float ang_vel_z =0;
        auto quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
        auto stamp = now();
        odom_msg.header.stamp = stamp;
        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation = quat;
        odom_msg.twist.twist.linear.x = lin_vel_x;
        odom_msg.twist.twist.linear.y = lin_vel_y;
        odom_msg.twist.twist.angular.z = ang_vel_z;
        if(odometry_pub)odometry_pub->publish(odom_msg);
        
        transform_msg.header.stamp = stamp;
        transform_msg.transform.translation.x=pos_x;
        transform_msg.transform.translation.y=pos_y;
        transform_msg.transform.translation.z=0;
        transform_msg.transform.rotation=quat;
        if(odom_broadcaster)odom_broadcaster->sendTransform(transform_msg);
      }
      if(m.id == ROSA_ODOMETRY_EXTENDED){
        float pos_x=m.read<float>();
        float pos_y=m.read<float>();
        float yaw=m.read<float>();
        float lin_vel_x=m.read<float>();
        float lin_vel_y=m.read<float>();
        float ang_vel_z=m.read<float>();
 
        auto quat = tf2::toMsg(tf2::Quaternion(tf2::Vector3(0.0, 0.0, 1.0), yaw));
        auto stamp = now();
        odom_msg.header.stamp = stamp;
        odom_msg.pose.pose.position.x = pos_x;
        odom_msg.pose.pose.position.y = pos_y;
        odom_msg.pose.pose.position.z = 0;
        odom_msg.pose.pose.orientation = quat;
        odom_msg.twist.twist.linear.x = lin_vel_x;
        odom_msg.twist.twist.linear.y = lin_vel_y;
        odom_msg.twist.twist.angular.z = ang_vel_z;
        if(odometry_pub)odometry_pub->publish(odom_msg);
        
        transform_msg.header.stamp = stamp;
        transform_msg.transform.translation.x=pos_x;
        transform_msg.transform.translation.y=pos_y;
        transform_msg.transform.translation.z=0;
        transform_msg.transform.rotation=quat;
        if(odom_broadcaster)odom_broadcaster->sendTransform(transform_msg);
      }
    }

  }
// Callback function to read data from cmd_vel topic
void RosaDriver::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr m)
{
  hw_ptr->send_message(cmd_vel_message(m->linear.x, m->linear.y, m->angular.z));
  RCLCPP_DEBUG(get_logger(), "Command sent via serial");
}
 
} //namespace ROSA

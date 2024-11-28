// This will be the node running on the Jetson to fly the drone

#include <chrono>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <omav_hovery_core/hl/rc_predicate_arming_logic.h>
#include <omav_hovery_core/ll/joystick_rc_receiver_adapter.h>
#include <omav_hovery_core/ll/motor_interface.h>
#include <omav_hovery_core/mock/memory_motor_adapter.h>
#include <omav_hovery_core/omav_base_client.h>
#include <omav_hovery_ros/msg_conv_ros1.h>
#include <omav_hovery_ros/ros_speed_angle_adapter.h>
#include <omav_msgs/UAVStatus.h>
#include <ros/ros.h>
// Avero includes:
#include <iostream>
#include <omav_hovery_drivers/arduino/pwm_motor_adapter.h>
#include <omav_hovery_drivers/dynamixel/dynamixel_motor_adapter.h>
#include <omav_hovery_core/ll/motor_interface_combinator.h>
#include <omav_hovery_ros/gazebo_multimotor_adapter.h>
#include "AveroController.h"
#include "AveroAllocator.h"
#include <omav_hovery_core/mock/memory_arming_logic.h>
#include <omav_hovery_core/mock/memory_rcreceiver_adapter.h>
#include <gazebo_msgs/ModelStates.h>
#include <ros/ros.h>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <nav_msgs/Odometry.h>
#include "mav_msgs/Actuators.h"
#include "omav_hovery_drivers/sbus/sbus_rc_receiver_adapter.h"

#include <std_msgs/Float64MultiArray.h>

using namespace std::chrono;
using namespace std::chrono_literals;

class AveroFlyNode {
 public:
  // TODO develop actual controller and allocator
  PIDController controller_;
  AveroJerkAllocator allocator_;

  AveroFlyNode(ros::NodeHandle nh) 
              : omav_client_(10ms), controller_(), allocator_(true, true, false) {  // set up as 100hz
    
    // Portnames and ID's
    std::string ArduinoPort = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"; 
    std::string portRC = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_RC-Receiver_FT6YE2HI-if00-port0"; 
    std::string DynamixelPort = "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FT4TFRC4-if00-port0"; 
    std::array<int,6> _6ids = {100,101,102,103,93,105};

// set up ROS magic
    first_odom_received = false; //don't change
    // use_initial_setpoint_offset = false; //set to true if you want des_odom to be offset by the pose that is received in the first message from the state estimator
    sub_setpoint_ = nh.subscribe("command/pose", 1, &AveroFlyNode::setpointCallback, this);
    sub_odom_ = nh.subscribe("/Avero_Dove/msf_core/odometry", 1, &AveroFlyNode::currentOdom, this);
    // ------ToDo: make the correct message formation from the message type
    sub_des_odom_ = nh.subscribe("/desired_drone_odometry", 1, &AveroFlyNode::UpdateDesiredOdometry, this);
    //gets desired Drone Odometry from JoyStick

    pub_rc_ = nh.advertise<sensor_msgs::Joy>("rc", 1);
    pub_uavstate_ = nh.advertise<omav_msgs::UAVStatus>("uavstate", 1); 
    pub_wrench = nh.advertise<std_msgs::Float64MultiArray>("des_wrench", 1);

    // set up Hovery magic
    auto rc = std::make_shared<omV::drv::SBUSRCReceiverAdapter>(portRC);
    auto arming_logic = std::make_shared<omV::hl::RCPredicateArmingLogic>(omV::hl::RCPredicateArmingLogic::AveroArmingPredicate, 1000ms,
                                                                          omV::hl::RCPredicateArmingLogic::AveroKillingPredicate);

    //setting 1. Desired point - stays this if don't get updated
    w_desired_position = { 0, 0, 0.5}; 
    w_desired_velocity = { 0, 0, 0 };
    desired_q_wb = Eigen::Quaterniond::Identity();
    w_desired_angular_velocity = {0 ,0 ,0};
    
    //To set initial Pose to zero
    getOdometry_ZERO();

    // AVERO Arduino interface to control propellers via PWM and Interface to control Dynamixels
    typedef omV::ll::PWMMotorAdapter<_PWM, _PWM, _PWM> ThreePWM;
    typedef omV::drv::DynamixelMotorAdapter<_6POS> SixDynamixels;
    typedef omV::ll::MotorInterfaceCombinator<ThreePWM, SixDynamixels, _PWM, _PWM, _PWM, _6POS> AveroAdapter;

    auto avero_arduino_interface = std::make_shared<ThreePWM>();
    auto avero_dynamixel = std::make_shared<SixDynamixels>(DynamixelPort, 3000000, _6ids);
    auto avero_interface = std::make_shared<AveroAdapter>(avero_arduino_interface, avero_dynamixel);
    // load a real joystick.
    // rc->init();
    ROS_WARN_STREAM("Setup interface");
    avero_arduino_interface->open(ArduinoPort); 
    avero_dynamixel->open(); 
    omav_client_.setInterface(avero_interface, rc, arming_logic);
    ros::Duration(7).sleep();
    ROS_WARN_STREAM("done setting up");
  }

  void threadloop() {
    omav_client_.waitForData();

    // publish RC
    auto rc_state = omav_client_.getRC();
    pub_rc_.publish(omV::ros1::toRosMessage(rc_state));

    // get feedback data
    auto current_state = omav_client_.getFullState();
    pub_uavstate_.publish(omV::ros1::toRosMessage(current_state));
    Vector9d omav_state = getOmavState(current_state);

    // get wrench and setpoint
    Vector6d wrench = controller_.control( w_desired_position, w_desired_velocity, desired_q_wb, w_desired_angular_velocity, 
                                           w_position, w_velocity, q_wb, w_angular_velocity); 

    //publish the wrench
    std_msgs::Float64MultiArray wrench_msg;
    wrench_msg.data = {wrench[0], wrench[1], wrench[2], wrench[3], wrench[4], wrench[5]};
    pub_wrench.publish(wrench_msg);
    // std::cout << "Wrench: " << wrench.transpose() << std::endl;

    std::array<double,9> new_setpoints_zuvor = allocator_.allocate(omav_state, wrench);
    std::array<double,9> new_setpoints; 

    //PWM from Allocator between 50 and 950 getso moved to between 1050 and 1950
    for(int i=0; i<3; i++){
      new_setpoints[i]=new_setpoints_zuvor[i] + 1050;
    }

    for(int i=3; i<9; i++){
      ROS_WARN_STREAM("Offset: "<<_6offsets[i-3]);
      new_setpoints[i]=(-4*new_setpoints_zuvor[i])+_6offsets[i-3];
      ROS_WARN_STREAM("Points Allocator: "<<new_setpoints_zuvor[i]<< " Danach: "<<new_setpoints[i]);
    }

	//new_setpoints[0] = 0;
    	//new_setpoints[1] = 0;
	//new_setpoints[2] = 1100;

	//needs to be deleted ifesc get recabled !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    double storage = new_setpoints[1];
    new_setpoints[1] = new_setpoints[2];
    new_setpoints[2] = storage;
    //new_setpoints[0] = 0;
    //new_setpoints[1] = 0;
    //new_setpoints[5] = 0;
    //new_setpoints[6] = 0;
    //new_setpoints[7] = 0;
    //new_setpoints[8] = 0;


    omav_client_.setFullState(new_setpoints);
  }

    //on real drone could be the joystick
  void UpdateDesiredOdometry(const nav_msgs::Odometry::ConstPtr& des_odom){
    //gets desired Drone Odometry from JoyStick

    //Position
    w_desired_position.x() = des_odom->pose.pose.position.x;
    w_desired_position.y() = des_odom->pose.pose.position.y;
    w_desired_position.z() = des_odom->pose.pose.position.z;


    //Velocity
    w_desired_velocity.x() = des_odom->twist.twist.linear.x;
    w_desired_velocity.y() = des_odom->twist.twist.linear.y;
    w_desired_velocity.z() = des_odom->twist.twist.linear.z;

    //Angular Velocity
    w_desired_angular_velocity.x() = des_odom->twist.twist.angular.x;
    w_desired_angular_velocity.y() = des_odom->twist.twist.angular.y;
    w_desired_angular_velocity.z() = des_odom->twist.twist.angular.z;
    
    //Rotation-Quaternion
    desired_q_wb.x() = des_odom->pose.pose.orientation.x;
    desired_q_wb.y() = des_odom->pose.pose.orientation.y;
    desired_q_wb.z() = des_odom->pose.pose.orientation.z;
    desired_q_wb.w() = des_odom->pose.pose.orientation.w;
  

    // ROS_INFO("Desired Odometry updated");
    // ROS_INFO_STREAM("Desired Position: " << w_desired_position);
  }

  // Run the client
  void run() {
    omav_client_.start();
    omav_client_.setHighPriority();  // try to elevate priority
  }

  // Stop the client
  void stop() { omav_client_.stop(); }

  Vector9d getOmavState(auto StatusArrayState) {
    Vector9d state;
    for (int i = 0; i < 3; ++i) {
        state[i] = StatusArrayState[i].velocity;
    }
    for (int i = 3; i < 9; ++i) {
        state[i] = StatusArrayState[i].position;
    }
    return state;
  }

    // get the odometry states from the simulation
  void getOdometry_ZERO() {
    
    //Position
    w_position.x() = 0;
    w_position.y() = 0;
    w_position.z() = 0;
    //Velocity
    w_velocity.x() = 0;
    w_velocity.y() = 0;
    w_velocity.z() = 0;
    //Rotation-Quaternion
    q_wb.x() = 0;
    q_wb.y() = 0;
    q_wb.z() = 0;
    q_wb.w() = 1;
    //Angular Velocity
    w_angular_velocity.x() = 0;
    w_angular_velocity.y() = 0;
    w_angular_velocity.z() = 0;
    
  }

  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose) {
    // not used at the moment

    // as this is not even a controller, we just listen to position
    // please use proper converison fct in real code ;-)
    setpoint_.x() = msg_pose->pose.position.x;
    setpoint_.y() = msg_pose->pose.position.y;
    setpoint_.z() = msg_pose->pose.position.z;
  }

  void currentOdom(const nav_msgs::Odometry::ConstPtr& msg_odom) {
    
    //Position
    w_position.x() = msg_odom->pose.pose.position.x;
    w_position.y() = msg_odom->pose.pose.position.y;
    w_position.z() = msg_odom->pose.pose.position.z;
    //Velocity
    b_velocity.x() = msg_odom->twist.twist.linear.x;
    b_velocity.y() = msg_odom->twist.twist.linear.y;
    b_velocity.z() = msg_odom->twist.twist.linear.z;
    w_velocity = q_wb * b_velocity;

    //Rotation-Quaternion
    q_wb.x() = msg_odom->pose.pose.orientation.x;
    q_wb.y() = msg_odom->pose.pose.orientation.y;
    q_wb.z() = msg_odom->pose.pose.orientation.z;
    q_wb.w() = msg_odom->pose.pose.orientation.w;
    q_bw = q_wb.inverse();

    //Angular Velocity
    b_angular_velocity.x() = msg_odom->twist.twist.angular.x;
    b_angular_velocity.y() = msg_odom->twist.twist.angular.y;
    b_angular_velocity.z() = msg_odom->twist.twist.angular.z;
    w_angular_velocity = q_wb * b_angular_velocity;
  }

  void initialize(){
    ros::Duration(1.5).sleep();
    omav_client_.waitForData();
    auto current_state_before = omav_client_.getFullState();
      // Set the Offsets
    for(int i = 3; i<9; i++){ // ALL all states from 2-8 are all the dynamixels!
        _6offsets[i-3] = current_state_before[i].position; // Get Radians
        ROS_WARN_STREAM("Offset dynamixel #"<<100+i-3<<" set to: "<< _6offsets[i-3]<<" Radians");
    }

    omav_client_.setFullState({1050,1050,1050, _6offsets[0], _6offsets[1], _6offsets[2], _6offsets[3], _6offsets[4], _6offsets[5]});

    while(!omav_client_.arming_logic_->isArmed()){
      ros::Duration(1/100).sleep();
    }
    ROS_WARN_STREAM("Initialized");
    ros::Duration(3).sleep();
  }

 private:
  Eigen::Vector3d setpoint_, odom_;
  Eigen::Vector3d w_position, w_velocity, b_velocity, w_angular_velocity, b_angular_velocity;
  Eigen::Quaterniond q_wb;
  Eigen::Quaterniond q_bw;
  ros::Subscriber sub_setpoint_, sub_odom_, sub_modelStates_, sub_des_odom_;
  ros::Subscriber sub_omav_state_sim, sub_wrench;
  ros::Publisher pub_rc_, pub_uavstate_, pub_wrench;
  ros::ServiceClient clientApplyWrench;
  bool first_odom_received;
  // bool use_initial_setpoint_offset;
  // Eigen::Vector3d w_initial_position;
  // Eigen::Quaterniond q_wb_initial;
  


  //Desired States
  Eigen::Vector3d w_desired_position; //in world frame
  Eigen::Vector3d w_desired_velocity; //in world frame
  Eigen::Quaterniond desired_q_wb; //from world to body frame
  Eigen::Vector3d w_desired_angular_velocity ; //in world frame

  std::array<double, 6> _6offsets;



  // AVERO adaption: 3 props + 6 DXLs.   
  omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS> omav_client_;     // FOR ACTUAL PROPELLERS
  using MotorArray = omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS>::SetpointArray;
  using MotorStatusArray = omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS>::StatusArray;
};

int main(int argc, char* argv[]) {



  // Actual program
  ros::init(argc, argv, "AveroFlyNode");
  ros::NodeHandle nh;
  ROS_WARN_STREAM("setting up");
  AveroFlyNode ctrl_node(nh);

  ROS_WARN_STREAM("starting");
  ctrl_node.run();

  ctrl_node.initialize(); 

  while (ros::ok()) {
    ros::spinOnce();
    ctrl_node.threadloop();
  }

  ctrl_node.stop();

  return 0;
}

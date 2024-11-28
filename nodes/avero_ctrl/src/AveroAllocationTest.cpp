// Adaptation of omav_hovery_demo/src/controller_example.cpp to the AVERO hardware and controller.

#include <chrono>
#include <cmath>
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

#include <std_msgs/Float64MultiArray.h>

using namespace std::chrono;
using namespace std::chrono_literals;

//--------------------------------------------------------------------------------------------
//Still need to add the parameterserver for the gains, start position, etc.
//--------------------------------------------------------------------------------------------

class AveroHoveryNode {
  public:
  PIDController controller_;
  AveroJerkAllocator allocator_;

  AveroHoveryNode(ros::NodeHandle nh) 
                  : omav_client_(10ms), controller_(), allocator_(true, true, false) {  // set up as 100hz
    //The constructors of the controller and allocator are called with : directly behind the constructor of hovery_node


    // set up ROS magic
    sub_modelStates_ = nh.subscribe("/pose_base_link", 10, &AveroHoveryNode::getOdometry_sim, this);
    sub_setpoint_ = nh.subscribe("command/pose", 1, &AveroHoveryNode::setpointCallback, this);
    sub_odom_ = nh.subscribe("perfect_state_estimator/odometry", 1, &AveroHoveryNode::setpointOdom, this);
    sub_des_odom_ = nh.subscribe("/desired_drone_odometry", 1, &AveroHoveryNode::UpdateDesiredOdometry, this);

    //only for simulation - don't know yet how to work with motor adappter plugin (is needed for flight on drone)
    sub_omav_state_sim = nh.subscribe("/gazebo/motor_states", 1, &AveroHoveryNode::updateOMAV_state_Sim, this);

    pub_rc_ = nh.advertise<sensor_msgs::Joy>("rc", 1);
    pub_uavstate_ = nh.advertise<omav_msgs::UAVStatus>("uavstate", 1); 

    pub_wrench = nh.advertise<std_msgs::Float64MultiArray>("des_wrench", 1);

    sub_wrench = nh.subscribe("des_wrench", 1, &AveroHoveryNode::setWrench, this);


    clientApplyWrench = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench"); // service client to apply wrenches to the simulation

    // set up Hovery magic
    auto rc = std::make_shared<omV::ll::JoystickRCReceiverAdapter>();
    auto arming_logic = std::make_shared<omV::hl::RCPredicateArmingLogic>(
        omV::hl::RCPredicateArmingLogic::Button1ArmingPredicate, 1000ms,
        omV::hl::RCPredicateArmingLogic::Button0KillingPredicate);
    auto mock_rc = std::make_shared<omV::mock::MemoryRCReceiverAdapter>();            // MOCK RC AND ARMING_LOGIC, DO NOT USE WITH ACTUAL HARDWARE!!!
    auto mock_arming_logic = std::make_shared<omV::mock::MemoryArmingLogic>();
    
    //setting 1. Desired point - stays this if don't get updated
    w_desired_position = { 0, 0, 0.5}; 
    w_desired_velocity = { 0, 0, 0 };; 
    desired_q_wb = Eigen::Quaterniond::Identity();
    w_desired_angular_velocity = {0 ,0 ,0};
    omav_state.setZero();
    

    // AVERO Arduino interface to control propellers via PWM and Interface to control Dynamixels
    typedef omV::ll::PWMMotorAdapter<_PWM, _PWM, _PWM> ThreePWM;
    typedef omV::mock::MockMotorSpeedAdapter<_6POS> SixDynamixels;
    typedef omV::ll::MotorInterfaceCombinator<ThreePWM, SixDynamixels, _PWM, _PWM, _PWM, _6POS> AveroAdapter;
    typedef omV::GazeboMultiMotoradapter<_VEL, _VEL, _VEL, _6POS> GazeboAdapter;

    auto avero_arduino_interface = std::make_shared<ThreePWM>();
    auto avero_mock_dynamixel = std::make_shared<SixDynamixels>();
    auto avero_interface = std::make_shared<AveroAdapter>(avero_arduino_interface, avero_mock_dynamixel);
    auto avero_gazebo = std::make_shared<GazeboAdapter>(nh);
    // auto avero_dynamixel_interface = std::make_shared<omV::drv::DynamixelMotorAdapter<_6POS>>();

    // load a real joystick.
    rc->init();

    // put everything into the base client
    omav_client_.setInterface(avero_gazebo, mock_rc, mock_arming_logic);
    mock_arming_logic->arm();
    ROS_WARN_STREAM("done setting up");

  }

  void threadloop(const std::chrono::steady_clock::time_point& end_wrench_apply) {
    //ROS_WARN_STREAM("Running ctrl!");
    omav_client_.waitForData();

    // publish RC
    auto rc_state = omav_client_.getRC();
    pub_rc_.publish(omV::ros1::toRosMessage(rc_state));

    // get feedback data
    auto current_state = omav_client_.getFullState(); //should in future happen with this plugin dont know yet how it works
    pub_uavstate_.publish(omV::ros1::toRosMessage(current_state));
    Vector9d omav_state = getOmavState(current_state);

   


    // get wrench and apply to simulation
    // Vector6d wrench = controller_.control( w_desired_position, w_desired_velocity, desired_q_wb, w_desired_angular_velocity, 
    //                                        w_position, w_velocity, q_wb, w_angular_velocity); 

    std::cout << "wrench: " << wrench_.transpose() << std::endl; 


    //publish the wrench

    
    // if (std::chrono::steady_clock::now() > end_wrench_apply){
    //   SimApplyWrench_sim(wrench);
    // }
    // SimApplyWrench_sim(wrench);

    //that we can fly to a hover position before we start the allocation
    // if (std::chrono::steady_clock::now() < end_wrench_apply){
      // SimApplyWrench_sim(wrench);
    // }

  //   // else{
  //   //   get setpoints

  //   //   NEED TO UNCOMMENT THIS FOR C++ IMPLEMENTATION, COMMENT FOR PYTHON
    std::array<double,9> new_setpoints = allocator_.allocate(omav_state, wrench_);
    omav_client_.setFullState(new_setpoints);
  //   // }

  //   // std::array<double,9> new_setpoints;
  //   // new_setpoints = {450, 450, 450, 0, 0, 0, 0, 0, 0}; 
  }


  // Run the client
  void run() {
    omav_client_.start();
    omav_client_.setHighPriority();  // try to elevate priority
  }

  // Stop the client
  void stop() { omav_client_.stop(); }

  void setpointCallback(const geometry_msgs::PoseStamped::ConstPtr& msg_pose) {
    // as this is not even a controller, we just listen to position
    // please use proper converison fct in real code ;-)
    setpoint_.x() = msg_pose->pose.position.x;
    setpoint_.y() = msg_pose->pose.position.y;
    setpoint_.z() = msg_pose->pose.position.z;
  }

  void updateOMAV_state_Sim(const mav_msgs::Actuators::ConstPtr& msg_actuators) {
    omav_state(0) = msg_actuators->angular_velocities[0];
    omav_state(1) = msg_actuators->angular_velocities[1];
    omav_state(2) = msg_actuators->angular_velocities[2];
    omav_state(3) = msg_actuators->angles[3];
    omav_state(4) = msg_actuators->angles[4];
    omav_state(5) = msg_actuators->angles[5];
    omav_state(6) = msg_actuators->angles[6];
    omav_state(7) = msg_actuators->angles[7];
    omav_state(8) = msg_actuators->angles[8];

    std::cout << "omav_state: " << omav_state.transpose() << std::endl;

  }

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

  void setpointOdom(const nav_msgs::Odometry::ConstPtr& msg_odom) {
    // perfect state estimates 100% of the time!
    odom_.x() = msg_odom->pose.pose.position.x;
    odom_.y() = msg_odom->pose.pose.position.y;
    odom_.z() = msg_odom->pose.pose.position.z;
  }

  void setWrench(const std_msgs::Float64MultiArray::ConstPtr& msg_wrench) {
    std::cout << "wrench received" << std::endl;
    for (int i = 0; i < 6; i++) {
      wrench_[i] = msg_wrench->data[i];
    }
  }

  // get the odometry states from the simulation
  void getOdometry_sim(const nav_msgs::Odometry::ConstPtr& msg_base_link) {
    
    //Position
    w_position.x() = msg_base_link->pose.pose.position.x;
    w_position.y() = msg_base_link->pose.pose.position.y;
    w_position.z() = msg_base_link->pose.pose.position.z;
    //Velocity
    w_velocity.x() = msg_base_link->twist.twist.linear.x;
    w_velocity.y() = msg_base_link->twist.twist.linear.y;
    w_velocity.z() = msg_base_link->twist.twist.linear.z;
    //Rotation-Quaternion
    q_wb.x() = msg_base_link->pose.pose.orientation.x;
    q_wb.y() = msg_base_link->pose.pose.orientation.y;
    q_wb.z() = msg_base_link->pose.pose.orientation.z;
    q_wb.w() = msg_base_link->pose.pose.orientation.w;
    //Angular Velocity
    w_angular_velocity.x() = msg_base_link->twist.twist.angular.x;
    w_angular_velocity.y() = msg_base_link->twist.twist.angular.y;
    w_angular_velocity.z() = msg_base_link->twist.twist.angular.z;
    
  }

  // Function to apply a wrench to the simulation. Use it to validate wrench outputs of controllers.
  void SimApplyWrench_sim(Vector6d b_wrench) {
    // Create a service message object and fill in the request
    gazebo_msgs::ApplyBodyWrench wrenchMsg;
    Vector6d w_wrench;

    //applying the wrench to the base_link-frame does not work, we have to rotate the wrench to the world frame
    wrenchMsg.request.body_name = "drone::base_link";

    w_wrench.head(3) = q_wb * b_wrench.head(3);
    w_wrench.tail(3) = q_wb * b_wrench.tail(3);
    
    wrenchMsg.request.reference_point.x = 0.0;
    wrenchMsg.request.reference_point.y = 0.0;
    wrenchMsg.request.reference_point.z = 0.0;
    wrenchMsg.request.wrench.force.x = w_wrench[0];
    wrenchMsg.request.wrench.force.y = w_wrench[1];
    wrenchMsg.request.wrench.force.z = w_wrench[2];
    wrenchMsg.request.wrench.torque.x = w_wrench[3];
    wrenchMsg.request.wrench.torque.y = w_wrench[4];
    wrenchMsg.request.wrench.torque.z = w_wrench[5];
    wrenchMsg.request.start_time = ros::Time::now();
    wrenchMsg.request.duration = ros::Duration(0.01); // This should match the omav_client_(10ms) which runs at 100 Hz.

    //print the wrench
    //ROS_INFO_STREAM("Wrench: " << wrench.transpose());


    // Call the service, inform if it failed
    if (!clientApplyWrench.call(wrenchMsg)) {
        ROS_ERROR("Failed to call service");
    }
  }

  //on real drone could be the joystick
  void UpdateDesiredOdometry(const nav_msgs::Odometry::ConstPtr& des_odom){

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
    

    ROS_INFO("Desired Odometry updated");
    ROS_INFO_STREAM("Desired Position: " << w_desired_position);
  }

 private:
  Eigen::Vector3d setpoint_, odom_;
  Eigen::Vector3d w_position, w_velocity, w_angular_velocity;
  Eigen::Quaterniond q_wb;
  ros::Subscriber sub_setpoint_, sub_odom_, sub_modelStates_, sub_des_odom_;
  ros::Subscriber sub_omav_state_sim, sub_wrench;
  ros::Publisher pub_rc_, pub_uavstate_, pub_wrench;
  ros::ServiceClient clientApplyWrench;


  


  //Desired States
  Eigen::Vector3d w_desired_position; //in world frame
  Eigen::Vector3d w_desired_velocity; //in world frame
  Eigen::Quaterniond desired_q_wb; //from world to body frame
  Eigen::Vector3d w_desired_angular_velocity ; //in world frame

  //omav motor states
  Vector9d omav_state;

  //wrench vector
  Vector6d wrench_;

  // AVERO adaption: 3 props + 6 DXLs.   
  // omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS> omav_client_;     // FOR ACTUAL PROPELLERS
  omV::OMAVBaseClient<_VEL, _VEL, _VEL, _6POS> omav_client_;        // FOR SIMULATION
  using MotorArray = omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS>::SetpointArray;
  using MotorStatusArray = omV::OMAVBaseClient<_PWM, _PWM, _PWM, _6POS>::StatusArray;
};


int main(int argc, char* argv[]) {


  //Controll Variables
  //desired

  ros::init(argc, argv, "AveroSimulationNode");
  ros::NodeHandle nh;
  ROS_WARN_STREAM("setting up");
  AveroHoveryNode ctrl_node(nh);

  ROS_WARN_STREAM("starting");
  ctrl_node.run();

  //so that we go to hover position with aply_wrench and than start the allocation
  std::chrono::steady_clock::time_point start_wrench_apply = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point end_wrench_apply = start_wrench_apply + std::chrono::seconds(10);

  while (ros::ok()) {
    ros::spinOnce();
    ctrl_node.threadloop(end_wrench_apply);
  }

  ctrl_node.stop();

  return 0;
}
#include <chrono>
#include <Eigen/Dense>
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
#include <omav_hovery_core/mock/memory_arming_logic.h>
#include <omav_hovery_core/mock/memory_rcreceiver_adapter.h>

//TestSpecific 
#include <cmath> // use Pi
#include <std_msgs/Header.h>
#include "avero_msgs/testing_msg.h"

using namespace std::chrono;
using namespace std::chrono_literals;


class AveroTestingNode {
 private:
    // ros::Subscriber sub_setpoint_, sub_odom_, sub_modelStates_;
    ros::Publisher pub_uavstate_, pub_state;

    // AVERO Show thing: 
    omV::OMAVBaseClient<_PWM,_PWM,_PWM, _4POS,_POS,_POS> omav_client_show;
    
    // AVERO adaption: 1 prop + 2 DXLs.   
    omV::OMAVBaseClient<_PWM, _POS, _POS> omav_client_; 
    
    // Put these later in a Launch file. 
    double StepsBaseNozzle =90; 
    double StepsTopNozzle = 90;
    double StepsPWM =  400;
    double MaxPWMSingal = 1400; 
    double HoldingTime = 5; //Time same RPM is held
    int TopOffset = 3; 
    int BaseOffset = 25; 
    int offset_base1=0; 
    int offset_base2=0; 
    int offset_base3=0; 
    int offset_top1=0;  
    int offset_top2=0;  
    int offset_top3=0;
    int ResetTime; 
    int RampUpTime;
    int decayTime; 
    std::array<int, 2> ids; 
    std::array<int, 6> _6ids;
    std::array<double, 2> offsets;
    std::array<double, 6> _6offsets;
    std::array<double, 2> uebersetztung; 
    std::string ArduinoPort = "/dev/ttyUSB1";
    std::string DynamixelPort = "/dev/ttyUSB0";
    std::string Interfacetype = "avero";
    ros::Duration time_turning_fan = ros::Duration(0);
    double rate{100};
    double maxRampVel_ = 80; 
    double startPWM = 1050;

 public: 
    AveroTestingNode(ros::NodeHandle nh){};


    void run() {
        if(Interfacetype=="Show"){
            omav_client_show.start();
            omav_client_show.setHighPriority();
        } else{
            omav_client_.start();
            omav_client_.setHighPriority();
        }
    }

    void stop() {
        if(Interfacetype=="Show"){
            omav_client_show.stop();
        } else {
            omav_client_.stop();
        }
    }

};
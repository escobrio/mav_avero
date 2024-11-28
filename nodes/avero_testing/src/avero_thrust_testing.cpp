#include "../include/avero_thrust_testing.h"
#include <omav_hovery_core/mock/memory_arming_logic.h>
#include <omav_hovery_core/mock/memory_motor_adapter.h>
#include <omav_hovery_core/mock/memory_rcreceiver_adapter.h>
#include <omav_hovery_drivers/arduino/pwm_motor_adapter.h>
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <omav_msgs/MotorStatus.h>

//Constructor:
AveroThrustTestingNode::AveroThrustTestingNode(){
    LOG(W, "You are using the Testing Node!");
    ros::NodeHandle nh{};
    ros::NodeHandle pnh{"~"};
    std::string interfaceType;    
    std::string device;

    //Get Parameters
    LOG(E, !pnh.getParam("startingValue", startingValue_), "Failed to retrieve minRPM param!");
    LOG(E, !pnh.getParam("endValue", endValue_), "Failed to retrieve maxRPM param!");
    LOG(E, !pnh.getParam("maxRampRPM", maxRampVel_), "Failed to retrieve maxRampRPM param!");
    LOG(E, !pnh.getParam("interface", interfaceType), "Failed to retrieve interface param!");
    LOG(E, !pnh.getParam("device", device), "Failed to retrieve device param!");
    LOG(E, !pnh.getParam("holdingTime", holdingTime_),"Failed to retrieve device param!");


    //Publish the Commands to the AveroPWMCommand topic
    cmd_vel_pub_ = nh.advertise<geometry_msgs::PointStamped>("AveroPWMCommand", 1); 

    if(interfaceType=="dummy"){
        LOG(I, "Starting dummy ESC interface");
        esc_adapter_ = std::make_shared<omV::mock::MockMotorSpeedAdapter<_PWM,_PWM,_PWM>>();
    } else{
        LOG(E, "Starting the PWM Adapter");
        auto avero_duct_adapter = std::make_shared<omV::ll::PWMMotorAdapter<_PWM,_PWM,_PWM>>(); //define here how to communicate with the DuctedFans
        avero_duct_adapter->open(device); //opens the port defined in the launch file. er
        esc_adapter_ = avero_duct_adapter; 
    }

    auto dummy_rc = std::make_shared<omV::mock::MemoryRCReceiverAdapter>();
    auto arming_logic = std::make_shared<omV::mock::MemoryArmingLogic>();
    arming_logic->arm();
    ms_client_.setInterface(esc_adapter_, dummy_rc, arming_logic);
    ros::Duration(3).sleep();   //setup the Interface
}

//Definition of Testsequence
void AveroThrustTestingNode::thrustTesting() {
    LOG(W, "Starting thrust testing...");
    double cmdVel{startingValue_};
    bool testingDone{false};
    double rate{100};
    double dt{1.0 / rate};
    ros::Rate r(rate);  // 100 hz
    ros::Time endTime = ros::Time::now() + ros::Duration(holdingTime_);

    LOG(W, "Current testing goal:" << endValue_ << " PWM.");
    LOG(W, "Current starting Value: " << cmdVel);
    while(ros::ok() && !testingDone){
        std::array<double, 3> a{cmdVel, cmdVel, cmdVel};
        
        //Write the command to the Motors!
        ms_client_.setFullState(a);

        //Generate a mesage: 
        geometry_msgs::PointStamped cmdVelMsg;
            cmdVelMsg.header.stamp = ros::Time::now();
            cmdVelMsg.point.x = cmdVel;
            cmdVelMsg.point.y = endValue_;
            cmd_vel_pub_.publish(cmdVelMsg);


        //debg start
        omV::OMAVBaseClient<_PWM,_PWM,_PWM>::StatusArray status = ms_client_.getFullState();
        omav_msgs::MotorStatus msg;
        msg.setpoint = status[0].setpoint;
        std::cout << msg << std::endl;
        //debug end

        if (cmdVel==endValue_){
            LOG(W,"Final PWM reached, holding");
            while(ros::Time::now()< endTime && ros::ok()){
                geometry_msgs::PointStamped cmdVelMsg;
                    cmdVelMsg.header.stamp = ros::Time::now();
                    cmdVelMsg.point.x = cmdVel;
                    cmdVelMsg.point.y = endValue_;
                    cmd_vel_pub_.publish(cmdVelMsg);

                ms_client_.setFullState({cmdVel, cmdVel, cmdVel});
                ros::spinOnce(); //einmal ausführen
                r.sleep(); 
            }
            std::array<double, 3> b{0.0, 0.0, 0.0};
            ms_client_.setFullState(b);
            testingDone = true;
        } else{
            cmdVel = std::clamp(cmdVel + maxRampVel_ * dt, 0.0, endValue_); // limitiere die cmdVel zwischen den Values
            ros::spinOnce(); //einmal ausführen
            r.sleep(); 
        }
    }
    std::array<double, 3> a{0.0, 0.0, 0.0};
    ms_client_.setFullState(a); 
    ms_client_.arming_logic_->disarm();
    LOG(W, "Thrust testing completed!");
}

void AveroThrustTestingNode::start() { ms_client_.start(); }

void AveroThrustTestingNode::stop() { ms_client_.stop(); }
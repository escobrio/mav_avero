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
#include <omav_hovery_drivers/sbus/sbus_rc_receiver_adapter.h>


#include "omav_hovery_core/hl/rc_predicate_arming_logic.h"
#include "omav_hovery_core/ll/rc_receiver_interface.h"
#include "omav_hovery_core/mock/memory_rcreceiver_adapter.h"
#include "omav_hovery_drivers/exbus/exbus_rc_receiver_adapter.h"
#include "omav_hovery_drivers/sbus/sbus_rc_receiver_adapter.h"
#include "std_msgs/Int32MultiArray.h"
//TestSpecific 
#include <cmath> // use Pi
#include <std_msgs/Header.h>

#include "avero_msgs/testing_msg.h"

using namespace std::chrono;
using namespace std::chrono_literals;

double DegreesToRadians(double degree, std::string position){
    double radians;
    if(position=="top"){
        radians = degree * M_PI / 180.0*52/12;
    } else if (position=="new") //Neue Nozzle hat Übersetzung 4 und drehe ccw; 
    {
        radians = degree * M_PI / 180.0*(-4);
    }else{
        radians = degree * M_PI / 180.0*52/15;
    }
        return radians;
}


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
    int TopOffset = 0; 
    int BaseOffset = 0; 
    int offset_base1; 
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
    std::string Interfacetype = "Show";
    std::string input_command = "IMU";
    std::string portRC = "/dev/serial/by-id/usb-FTDI_TTL232R-3V3_RC-Receiver_FT6YE2HI-if00-port0"; 
    ros::Duration time_turning_fan = ros::Duration(0);
    double rate{100};
    double maxRampVel_ = 80; 
    double startPWM = 1050;

    //NEW
    ros::Publisher pub_rc_, pub_all;
    ros::Subscriber sub_PWM_RC, sub_PWM_Terminal, sub_offsets, rc_sub, sub_imu_joints;
    double pwm_hover; 
    bool firstOne = true; 
    float base_position_imu; 
    float top_position_imu;
    float base_position;
    float top_position;




 public:
    AveroTestingNode(ros::NodeHandle nh){
        std::cout<<std::endl;
        std::cout<<std::endl;


        // NEW
        pub_rc_ = nh.advertise<sensor_msgs::Joy>("rc", 1);
        // pub_all =nh.advertise<avero_msgs::hover_test_msg>("HoverStates", 1); 

        ROS_WARN_STREAM("Setting Test up!");

        pub_uavstate_ = nh.advertise<omav_msgs::UAVStatus>("uavstate", 1);
        pub_state = nh.advertise<avero_msgs::testing_msg>("testing_topic", 1);
        ros::NodeHandle pnh{"~"};
        double id_base, id_top; 

        // Load Parameters
        // Steps
        LOG(E, !pnh.getParam("steps_base_nozzle", StepsBaseNozzle), "Failed to retrieve StepsBaseNozzle param!");
        LOG(E, !pnh.getParam("steps_top_nozzle", StepsTopNozzle), "Failed to retrieve StepsTopNozzle param!");
        LOG(E, !pnh.getParam("steps_pwm", StepsPWM), "Failed to retrieve StepsPWM param!");

        // PWM 
        LOG(E, !pnh.getParam("start_pwm", startPWM), "Failed to retrieve startPWM param!");
        LOG(E, !pnh.getParam("max_pwm_signal", MaxPWMSingal), "Failed to retrieve MaxPWMSingal param!");
        LOG(E, !pnh.getParam("max_ramp_vel", maxRampVel_), "Failed to retrieve maxRampVel_ param!");

        // Times 
        LOG(E, !pnh.getParam("ramp_up_time", RampUpTime), "Failed to retrieve maxRampVel_ param!");
        LOG(E, !pnh.getParam("reset_time", ResetTime), "Failed to retrieve reset_time param!");
        LOG(E, !pnh.getParam("holding_time", HoldingTime), "Failed to retrieve HoldingTime param!");
        LOG(E, !pnh.getParam("decay_time", decayTime), "Failed to retrieve decayTime param!");

        // Offsets
        // Test
        LOG(E, !pnh.getParam("offset_top", TopOffset), "Failed to retrieve TopOffset param!");
        LOG(E, !pnh.getParam("offset_base", BaseOffset), "Failed to retrieve BaseOffset param!");
        // Show
        LOG(E, !pnh.getParam("offset_base1",offset_base1), "Failed to retrieve Offset param");
        LOG(E, !pnh.getParam("offset_base2",offset_base2), "Failed to retrieve Offset param");
        LOG(E, !pnh.getParam("offset_base3",offset_base3) , "Failed to retrieve Offset param");
        LOG(E, !pnh.getParam("offset_top1",offset_top1), "Failed to retrieve Offset param");
        LOG(E, !pnh.getParam("offset_top2",offset_top2), "Failed to retrieve Offset param");
        LOG(E, !pnh.getParam("offset_top3",offset_top3), "Failed to retrieve Offset param");

        // ID
        LOG(E, !pnh.getParam("id_base", id_base), "Failed to retrieve id_base param!");
        LOG(E, !pnh.getParam("id_top", id_top), "Failed to retrieve id_top param!");
        
        // Ports
        LOG(E, !pnh.getParam("arduino_port", ArduinoPort), "Failed to retrieve ArduinoPort param!");
        LOG(E, !pnh.getParam("dynamixel_port", DynamixelPort), "Failed to retrieve DynamixelPort param!");
        LOG(E, !pnh.getParam("interface_type", Interfacetype), "Failed to retrieve Interfacetype param!");
        
        // Input Command
        LOG(E, !pnh.getParam("input_command", input_command), "Failed to retrieve input_command param!");

        // Set ID's 
        ids[0]=id_base;
        ids[1]=id_top; 
        _6ids = {100,101,102,103,104,105};
        
        // Set offsets's // Only implemented for one Nozzle at the moment!
        offsets = {DegreesToRadians(BaseOffset,"base"),DegreesToRadians(TopOffset,"top")}; //Base Top
        _6offsets =  {DegreesToRadians(offset_base1, "new"), DegreesToRadians(offset_top1, "new"), DegreesToRadians(offset_base2, "new"), DegreesToRadians(offset_top2, "new"), DegreesToRadians(offset_base3, "new"), DegreesToRadians(offset_top3, "new")}; 
        
        // Defintions
        typedef omV::ll::PWMMotorAdapter<_PWM> OneArduinoPWM;
        typedef omV::ll::PWMMotorAdapter<_PWM,_PWM,_PWM> ThreeArduinoPWM; 
        typedef omV::drv::DynamixelMotorAdapter<_POS, _POS> TwoDynamixel;
        typedef omV::drv::DynamixelMotorAdapter<_4POS,_POS,_POS> SixDynamixel; 
        typedef omV::mock::MockMotorSpeedAdapter<_POS, _POS> TwoMockDynamixel;

        typedef omV::mock::MockMotorSpeedAdapter<_4POS,_POS,_POS> SixMockDynamixel;

        auto avero_one_arduino = std::make_shared<OneArduinoPWM>();
        auto avero_three_arduino = std::make_shared<ThreeArduinoPWM>(); 
        auto avero_two_dynamixel = std::make_shared<TwoDynamixel>(DynamixelPort, 3000000, ids, offsets);
        auto avero_six_dynamixel = std::make_shared<SixDynamixel>(DynamixelPort, 3000000, _6ids, _6offsets);
        auto avero_two_mock_dynamixel = std::make_shared<TwoMockDynamixel>();
        auto six_avero_two_mock_dynamixel = std::make_shared<SixMockDynamixel>();

        // Combinatorsoffsets_
        typedef omV::ll::MotorInterfaceCombinator<OneArduinoPWM, TwoDynamixel, _PWM, _POS, _POS> AveroTestingCombined;
        typedef omV::ll::MotorInterfaceCombinator<OneArduinoPWM, TwoMockDynamixel,_PWM, _POS, _POS> AveroMockTestingCombined;
        typedef omV::ll::MotorInterfaceCombinator<ThreeArduinoPWM, SixMockDynamixel,_PWM,_PWM,_PWM, _4POS,_POS,_POS> SixAveroMockTestingCombined;
        typedef omV::ll::MotorInterfaceCombinator<ThreeArduinoPWM, SixDynamixel,_PWM,_PWM,_PWM, _4POS,_POS,_POS> AveroShowCombined;

        // Defined Adapters
        auto avero_test_adapter = std::make_shared<AveroTestingCombined>(avero_one_arduino, avero_two_dynamixel); 
        auto avero_show_adapter = std::make_shared<AveroShowCombined>(avero_three_arduino,avero_six_dynamixel); 

        auto avero_mock_test_adapter = std::make_shared<AveroMockTestingCombined>(avero_one_arduino, avero_two_mock_dynamixel); 
        auto six_avero_mock_test_adapter = std::make_shared<SixAveroMockTestingCombined>(avero_three_arduino, six_avero_two_mock_dynamixel); 


//---------------------------------------------------------------------------------------------------
    // Define the Interfaces
        omV::ll::RCReceiverInterface* rc_receiver;
        omV::hl::ArmingLogicInterface* arming_logic; 

        // Define the RC
        omV::mock::MemoryRCReceiverAdapter* mock_rc = new omV::mock::MemoryRCReceiverAdapter();
        omV::drv::SBUSRCReceiverAdapter* sbus_rc = new omV::drv::SBUSRCReceiverAdapter(portRC);

        //Define the Arming Logic
        omV::mock::MemoryArmingLogic* arming_logic_MOCK = new omV::mock::MemoryArmingLogic(); 
        // omV::hl::RCPredicateArmingLogic* arming_logic_RC= new omV::hl::RCPredicateArmingLogic (omV::hl::RCPredicateArmingLogic::AveroArmingPredicate, 1000ms,
        //                                                                                          omV::hl::RCPredicateArmingLogic::AveroKillingPredicate);
    // Define the 
        switch (0) {
        case 0: // Dummy
                rc_receiver = mock_rc;
                arming_logic = arming_logic_MOCK; 
            break;
        case 1: // ThrustTestStand
                // rc_receiver = sbus_rc;
                // arming_logic = arming_logic_RC; 
            break;
        case 2: // FlyConfiguration
            // rc_receiver = exbus_receiver;
            break;
        }

        std::shared_ptr<omV::ll::RCReceiverInterface> rc_receiver_shared(rc_receiver);
        std::shared_ptr<omV::hl::ArmingLogicInterface> arming_logic_shared(arming_logic);


//---------------------------------------------------------------------------------------------------
        //sets up the BaseClient with the right interface
        if(Interfacetype=="dummy"){
            ROS_WARN_STREAM("Set Interface to dummy!");
            ros::Duration(3.0).sleep();
            omav_client_.setInterface(avero_mock_test_adapter, rc_receiver_shared, arming_logic_shared);
            arming_logic_shared->arm();
            rc_sub = nh.subscribe("rc",1,&AveroTestingNode::stepointcallback, this);
            ros::Duration(3.0).sleep();
        } else if (Interfacetype =="Show"){
            ///To Do merge Show and HoverDummy
            ROS_WARN_STREAM("Set Interface to Show"); 
            using MotorArray =  omV::OMAVBaseClient<_PWM,_PWM,_PWM, _4POS,_POS,_POS>::SetpointArray;
            using MotorStatusArray = omV::OMAVBaseClient<_PWM,_PWM,_PWM, _4POS,_POS,_POS>::StatusArray;
            avero_three_arduino->open(ArduinoPort); 
            avero_six_dynamixel->open(); 
            // sbus_rc ->init();
            omav_client_show.setInterface(avero_show_adapter, rc_receiver_shared, arming_logic_shared); 
            //mock_arming_logic->arm();


            ROS_WARN_STREAM("Wait for BaseClient");
            ros::Duration(7.0).sleep();
            
            ROS_ERROR_STREAM("SetUp done!");
        }else if (Interfacetype=="HoverDummy"){
            ROS_WARN_STREAM("Set Interface to HoverDummy"); 
            using MotorArray =  omV::OMAVBaseClient<_PWM,_PWM,_PWM, _4POS,_POS,_POS>::SetpointArray;
            using MotorStatusArray = omV::OMAVBaseClient<_PWM,_PWM,_PWM, _4POS,_POS,_POS>::StatusArray;
            avero_three_arduino->open(ArduinoPort); 
            avero_six_dynamixel->open(); 
            // avero_show_adapter , six_avero_mock_test_adapter
            omav_client_show.setInterface(avero_show_adapter, rc_receiver_shared, arming_logic_shared); 
            
            // Necessary Subscribers
            sub_imu_joints = nh.subscribe("joint_angles", 1, &AveroTestingNode::IMUJointAngles, this);
            sub_offsets = nh.subscribe("offset", 1, &AveroTestingNode::SetOffsets, this);
            rc_sub = nh.subscribe("rc",1,&AveroTestingNode::stepointcallback, this);

            ROS_WARN_STREAM("Wait for BaseClient");
            ros::Duration(7.0).sleep(); 
            ROS_ERROR_STREAM("SetUp done!");
        }else { // Works with RC!
            ROS_INFO_STREAM("Avero Setup choosen!");
            using MotorArray =  omV::OMAVBaseClient<_PWM, _POS, _POS>::SetpointArray;
            using MotorStatusArray = omV::OMAVBaseClient<_PWM, _POS, _POS>::StatusArray;
            avero_one_arduino->open(ArduinoPort);
            avero_two_dynamixel->open();
            omav_client_.setInterface(avero_test_adapter, rc_receiver_shared, arming_logic_shared);
            ROS_INFO_STREAM("Wait to BaseClient is starting up!");
            ros::Duration(7.0).sleep();
            ROS_WARN_STREAM("SetUp done!");
        }
    }

    // Selects which test sequence gets started
    void test() {
        if(Interfacetype=="Show"){
            std::cout << "No Show defined atm.";
        } else if (Interfacetype=="avero"){
            TestOnePWMForDiffrentPositions(StepsBaseNozzle, StepsTopNozzle, StepsPWM);
        } else if (Interfacetype=="HoverDummy"){
            HoverTest();
        }else {
            std::cout << "No defined Interface"; 
        }
    }

    std::array<double, 2> GetRcInputToSteerTwoDynamixel(auto RCState){ // Right Stick is the input stick and converts -1 to -180° and so on...
        // Set with RC input
        float base_position_rc   = RCState.axes[0];
        float top_position_rc    = RCState.axes[1];

        if(base_position_rc<0.05&&base_position_rc>-0.05){
            base_position_rc = 0.0;
        } 
        
        if(top_position_rc<0.05&&top_position_rc>-0.05){
            top_position_rc = 0.0;
        }
        
        base_position = base_position_rc*180.0;
        top_position = top_position_rc*180.0;

        return std::array<double, 2> {base_position, top_position}; 
    }
//---------------------------------------------------------------------------------------------------------------------------------------------------------------
// This is for all six dynamixel tests

    // Gets all PWM and all base and top in degrees.
    std::array<double, 9> SetAllStates( double pwm1, double pwm2, double pwm3, 
                                        double base1, double top1, double base2,
                                        double top2, double base3, double top3){
        std::array<double, 9> a;
        if(ros::ok()){
            a ={pwm1,pwm2,pwm3, 
                DegreesToRadians(base1,"new"), DegreesToRadians(top1,"new"), 
                DegreesToRadians(base2,"new"), DegreesToRadians(top2,"new"), 
                DegreesToRadians(base3,"new"), DegreesToRadians(top3,"new")};
        }
        return a;
    }

// rostopic pub -r 1 /offset std_msgs/Int32MultiArray "data: [20,30,0,0,0,0]"
//  rosrun omav_hovery_demo omav_hovery_rc_poc 
    // void printTheStatusHover(){
    //     avero_msgs::testing_msg msg_new;
    //     msg_new.header.stamp=ros::Time::now();
    //     msg_new.header.frame_id="Hover";
    //     msg_new.PWM = msg_old.PWM;
    //     msg_new.TopPosition = msg_old.TopPosition;
    //     msg_new.BasePosition = msg_old.BasePosition;
    //     msg_new.Status = status_; 
    //     pub_state.publish(msg_new);
    // }

    // Callback of the IMU Joint Angles sets the base and top position according to the IMU
    void IMUJointAngles(const std_msgs::Int32MultiArray& msg){
        base_position_imu = msg.data[0];
        top_position_imu = msg.data[1];
    }

    // Base1, Top1, Base2, Top2, Base3, Top3
    /// Not needed anymore after the Homing command
    void SetOffsets(const std_msgs::Int32MultiArray& offset_msg){
        offset_base1 =   offset_msg.data[0]; 
        offset_base2 =   offset_msg.data[1]; 
        offset_base3 =   offset_msg.data[2]; 
        offset_top1 =    offset_msg.data[3];  
        offset_top2 =    offset_msg.data[4];  
        offset_top3 =    offset_msg.data[5];
    }

    // Callback function for the RC Reciever. Sets the pwm_hover value as % for the PWM value
    void stepointcallback(const sensor_msgs::Joy& msg_joy){
        pwm_hover = msg_joy.axes[2];
    }

    // Subscribe to the RC message, Convert the PWM stick to a PWM Signal  
    std::array<double, 9>  SetNewSetPoints(){
        double range = MaxPWMSingal -1050; //define the range // now we go from 1050 =0% to MaxPWMSignal = 100%

        double pwm_value; 

        if(pwm_hover<0){
            pwm_value =1050; 
        } else if(pwm_hover<0.1){
            pwm_value = 1100; 
        } else{
            pwm_value = 1050 + range*pwm_hover; 
        }
        // Achtung hier setze die gewollten Angles!!!
        std::array<double, 9> state = SetAllStates(pwm_value,pwm_value,pwm_value,base_position+_6offsets[0],top_position+_6offsets[1],
                                                            base_position+_6offsets[2],top_position+_6offsets[3],
                                                            base_position+_6offsets[4],top_position+_6offsets[5]);
        return state; 
    }

    void Hovering(){

        if(firstOne&&omav_client_show.arming_logic_->isArmed()){
            //ROS_INFO_STREAM("Homing...");
            omav_client_show.waitForData();
            auto rc_state = omav_client_show.getRC();
            pub_rc_.publish(omV::ros1::toRosMessage(rc_state));
            auto current_state = omav_client_show.getFullState();
            pub_uavstate_.publish(omV::ros1::toRosMessage(current_state));

            // Maybe add here go to hover position
            
            if(input_command=="IMU"){
                base_position = base_position_imu;
                top_position = top_position_imu;
            } else{
                // Set with RC input
                float base_position_rc   = rc_state.axes[0];
                float top_position_rc    = rc_state.axes[1];

                if(base_position_rc<0.05&&base_position_rc>-0.05){
                    base_position_rc = 0.0;
                } 
                
                if(top_position_rc<0.05&&top_position_rc>-0.05){
                    top_position_rc = 0.0;
                }
                
                base_position = base_position_rc*180.0;
                top_position = top_position_rc*180.0;
            }


            auto new_setpoints = SetAllStates(1050,1050,1050,base_position+_6offsets[0],top_position+_6offsets[1],
                                                            base_position+_6offsets[2],top_position+_6offsets[3],
                                                            base_position+_6offsets[4],top_position+_6offsets[5]);
            
            omav_client_show.setFullState(new_setpoints);
            // Get the Positions of the motors
            auto current_state_ = omav_client_show.getFullState();

            std::cout << "Positions: " << " Dy1: "<< current_state_[3].position/M_PI*180.0/(-4)-_6offsets[0]<<" Dy2: "<< current_state_[4].position/M_PI*180.0/(-4)-_6offsets[1]<<" Dy3: "<< current_state_[5].position/M_PI*180.0/(-4)-_6offsets[2]<< " Dy4: "
                     << current_state_[6].position/M_PI*180.0/(-4)-_6offsets[3]<< " Dy5: " << current_state_[7].position/M_PI*180.0/(-4)-_6offsets[4] << " Dy6: " << current_state_[8].position/M_PI*180.0/(-4)-_6offsets[5]<< std::endl; 
            


            //DEBUG COMMANDS 
/*
            //std::cout<< "Offsets: " << offset_base1 << ",  "<< offset_base2 << ",  "<< offset_base3 << ",  "<< offset_top1 << ",  "<< offset_top2<< ",  " << offset_top3<< std::endl; 
            // avero_msgs::hover_test_msg status_msg; 
            // status_msg.header = ros::Time::now(); 
*/


            if(rc_state.switches[0]==1&&rc_state.axes[3]<-0.9&&rc_state.axes[4]>0.9){
                ROS_ERROR_STREAM("Offset Configured!"); 
                ros::Duration(5).sleep();
                ROS_ERROR_STREAM("Start Fly!"); 
                ros::Duration(0.3).sleep();
                firstOne = false;
            }

        } else if(!firstOne&&omav_client_show.arming_logic_->isArmed()){
            omav_client_show.waitForData();
            auto rc_state = omav_client_show.getRC();

            pub_rc_.publish(omV::ros1::toRosMessage(rc_state));
            auto current_state_ = omav_client_show.getFullState();
            pub_uavstate_.publish(omV::ros1::toRosMessage(current_state_));

            auto new_setpoints = SetNewSetPoints(); 
            std::cout << new_setpoints[0]<< std::endl;
            std::cout << "Positions: " << " Dy1: "<< current_state_[3].position/M_PI*180.0/(-4)-_6offsets[0]<<" Dy2: "<< current_state_[4].position/M_PI*180.0/(-4)-_6offsets[1]<<" Dy3: "<< current_state_[5].position/M_PI*180.0/(-4)-_6offsets[2]<< " Dy4: "
                     << current_state_[6].position/M_PI*180.0/(-4)-_6offsets[3]<< " Dy5: " << current_state_[7].position/M_PI*180.0/(-4)-_6offsets[4] << " Dy6: " << current_state_[8].position/M_PI*180.0/(-4)-_6offsets[5]<< std::endl; 


            // set Full State of client
            omav_client_show.setFullState(new_setpoints);
        } else{
            std::cout << "Not Armed, starting up..."<< std::endl; 
        }
    }

    bool initialize(){ // Returns True if BaseClient armed! And sets the State to 0 with the offsets.
        omav_client_show.waitForData();
        auto rc_state = omav_client_show.getRC();
        pub_rc_.publish(omV::ros1::toRosMessage(rc_state));
        
        auto current_state = omav_client_show.getFullState();
        pub_uavstate_.publish(omV::ros1::toRosMessage(current_state));
        omav_client_show.setFullState(SetAllStates(1050,1050,1050,_6offsets[0],_6offsets[1],_6offsets[2],_6offsets[3],_6offsets[4],_6offsets[5]));
        return omav_client_show.arming_logic_->isArmed(); 
    }
 
    void HoverTest(){
        ROS_ERROR_STREAM("Set MaxPwm to: "<< MaxPWMSingal);
        omav_client_show.waitForData();
        auto current_state_before = omav_client_show.getFullState();
        ros::Duration(1.5).sleep();
        
        // Set the Offsets
        for(int i = 3; i<9; i++){ // ALL all states from 2-8 are all the dynamixels!
            _6offsets[i-3] = ((current_state_before[i].position)/M_PI*180.0/(-4)); // Get Radians calculated to degrees with the transmission of 4.
            std::cout<< "Offset dynamixel #"<<100+i-3<<" set to: "<< _6offsets[i-3]<<std::endl;
            std::cout<< std::endl;
        }

        ROS_INFO_STREAM("Withs Offset");
        omav_client_show.setFullState(SetAllStates(1050,1050,1050,_6offsets[0],_6offsets[1],_6offsets[2],_6offsets[3],_6offsets[4],_6offsets[5]));
        ros::Duration(3).sleep();
        omav_client_show.waitForData();
        ROS_ERROR_STREAM("Zerro");
        auto current_state_ = omav_client_show.getFullState();

        std::cout << "Positions: " << " Dy1: "<< current_state_[3].position/M_PI*180.0/(-4)-_6offsets[0]<<" Dy2: "<< current_state_[4].position/M_PI*180.0/(-4)-_6offsets[1]<<" Dy3: "<< current_state_[5].position/M_PI*180.0/(-4)-_6offsets[2]<< " Dy4: "
                    << current_state_[6].position/M_PI*180.0/(-4)-_6offsets[3]<< " Dy5: " << current_state_[7].position/M_PI*180.0/(-4)-_6offsets[4] << " Dy6: " << current_state_[8].position/M_PI*180.0/(-4)-_6offsets[5]<< std::endl; 

        ros::Duration(3).sleep();

        bool initialized =false;
        while (ros::ok())
        {
            ros::spinOnce();
            if(!initialized){
                initialized = initialize();
            } else{
                Hovering();
            }
        }
    }


//---------------------------------------------------------------------------------------------------------------------------------------------------------------
    std::array<double, 3> GetTwoDynamixelStatesLoad(){
        std::array<double, 2> DynamixelStates = GetRcInputToSteerTwoDynamixel(omav_client_.getRC());
        std::array<double, 3> a = {MaxPWMSingal, DynamixelStates[0], DynamixelStates[1]}; 
        return a; 
    }

    void DynamixelLoadTest(){
        omav_client_.waitForData();
        auto rc_state = omav_client_.getRC();
        pub_rc_.publish(omV::ros1::toRosMessage(rc_state));

        auto current_state = omav_client_.getFullState();
        pub_uavstate_.publish(omV::ros1::toRosMessage(current_state));

        auto new_setpoints = GetTwoDynamixelStatesLoad();

        omav_client_.setFullState(new_setpoints);
    }

    void DynamixelLoadtestLoop(){
        ROS_WARN_STREAM("Testing Dynamixel Load with: " << MaxPWMSingal);
        ros::Duration(4).sleep(); 
        while (ros::ok()){
            DynamixelLoadTest();
        }
    }

    //  Test for Base = 0 and Top = 0 all PWM for calculating error of Thrust vector
    /// ToDo implement the Homing...
    void FinnTesting() {
        avero_msgs::testing_msg SetPointMSG;
        ROS_INFO_STREAM("Go to start position...");
        omav_client_.setFullState({1050, DegreesToRadians(0,"base"), DegreesToRadians(0,"top")});
        ROS_ERROR_STREAM("Starting Test!");
        ros::Duration(2).sleep();

        for(double pwm = 1000; pwm<=MaxPWMSingal; pwm = pwm + StepsPWM){
            SetPointMSG.TopPosition = 0.0;
            SetPointMSG.BasePosition = 0.0;
            SetPointMSG.PWM = pwm; 
            TestThis(SetPointMSG);
        }
    }

    // Test for one PWM signal all positions
    // ToDo: cancle all positions not needed!
    void TestOnePWMForDiffrentPositions(double StepsBaseNozzle_, double StepsTopNozzle_, double StepsPWM_){
        avero_msgs::testing_msg SetPointMSG;
        SetPointMSG.PWM = MaxPWMSingal; // Bis zu diesem PWM wird getestet.
        ROS_INFO_STREAM("Test initialized for a Max PWM Signal of: "<< MaxPWMSingal);
        omav_client_.waitForData();
        auto homing_state = omav_client_.getFullState();
        ros::Duration(1.5).sleep();

        // Homing: read out first states, then set these as offsets. 
        BaseOffset = homing_state[1].position/M_PI*180.0/(-4); 
        TopOffset = homing_state[2].position/M_PI*180.0/(-4); 
        std::cout << "Offsets, Base: " << BaseOffset << "Top: " << TopOffset << std::endl; 
        omav_client_.setFullState({1050, DegreesToRadians(BaseOffset,"new"), DegreesToRadians(TopOffset,"new")});
        // Homing end

        ROS_WARN_STREAM("Starting Test!");
        for(double BasePosition =0; BasePosition<360; BasePosition += StepsBaseNozzle_){//Turn Base Nozzle
            for(double TopPosition=0; TopPosition<360; TopPosition+=StepsTopNozzle_){   //Turn Top Nozzle
                if(ros::ok()){
                    SetPointMSG.TopPosition = TopPosition;
                    SetPointMSG.BasePosition = BasePosition;
                    TestThis(SetPointMSG);
                }
            }
        }
    }

    // Tests a Configuration wihle following the pattern needed for Data Analysis later on
    /// -> publishes the states where it is at the Moment. 
    void TestThis(avero_msgs::testing_msg SetPointMSG_){
        double endPWM = SetPointMSG_.PWM;
        std::string status; 
        ros::Rate r(rate); 
        double dt{1.0 / rate};

        ROS_WARN_STREAM("Test: PWM= "<<endPWM<<", BasePositoion= "<<SetPointMSG_.BasePosition<<", TopPosition= "<<SetPointMSG_.TopPosition);
        ros::Duration(1).sleep();
        status = "start_position"; 
        SetPointMSG_.PWM = 1050; 
        
        ros::Time ResetEndTime = ros::Time::now() + ros::Duration(ResetTime);
        while(ros::Time::now()<ResetEndTime&&ros::ok()){
            omav_client_.setFullState({1050, DegreesToRadians(SetPointMSG_.BasePosition-BaseOffset,"base"), DegreesToRadians(SetPointMSG_.TopPosition-TopOffset,"top")});
            PrintStatus(SetPointMSG_, status); // Send message to Topic... 
            ros::spinOnce();
            r.sleep();
        }

        // Go to PWM:
        ROS_INFO_STREAM("Ramping Up...");
        SetPointMSG_.PWM= endPWM; 
        status = "ramp_up"; 
        ros::Time RampUpEndTime = ros::Time::now() + ros::Duration(RampUpTime);
        while(ros::Time::now()<RampUpEndTime&&ros::ok()){
            omav_client_.setFullState({endPWM, DegreesToRadians(SetPointMSG_.BasePosition,"base"), DegreesToRadians(SetPointMSG_.TopPosition,"top")});
            PrintStatus(SetPointMSG_, status);
            ros::spinOnce();
            r.sleep();
        }

        // Holding after which time? 
        ROS_INFO_STREAM("Holding...");
        status = "holding"; 
        PrintStatus(SetPointMSG_, status);
        ros::Time endholdingTime = ros::Time::now() + ros::Duration(HoldingTime);
        while(ros::Time::now()<endholdingTime && ros::ok()){
            omav_client_.setFullState({endPWM, DegreesToRadians(SetPointMSG_.BasePosition,"base"), DegreesToRadians(SetPointMSG_.TopPosition,"top")});
            PrintStatus(SetPointMSG_, status); 
            ros::spinOnce();
            r.sleep();
        }

        SetPointMSG_.PWM= 1050;
        status = "finished"; 
        PrintStatus(SetPointMSG_, status);

        // Finisehd Test... 
        ROS_INFO_STREAM("Finshed Test");
        ros::Time decayendTime = ros::Time::now() + ros::Duration(decayTime);
        while(ros::Time::now()<decayendTime && ros::ok()){
            omav_client_.setFullState({1050, DegreesToRadians(SetPointMSG_.BasePosition,"base"), DegreesToRadians(SetPointMSG_.TopPosition,"top")});
            PrintStatus(SetPointMSG_, status); 
            ros::spinOnce();
            r.sleep();
        }
    }

    void PrintStatus(avero_msgs::testing_msg msg_old, const std::string& status_ = "Not given..."){
        // Send MotorState Message
        omV::OMAVBaseClient<_PWM,_POS,_POS>::StatusArray status = omav_client_.getFullState();
        omav_msgs::UAVStatus msg = omV::ros1::toRosMessage(status, omav_client_.arming_logic_->getState());
        pub_uavstate_.publish(msg);
        
        // Send Testing sequence
        avero_msgs::testing_msg msg_new;
        msg_new.header.stamp=ros::Time::now();
        msg_new.header.frame_id="Test";
        msg_new.PWM = msg_old.PWM;
        msg_new.TopPosition = msg_old.TopPosition;
        msg_new.BasePosition = msg_old.BasePosition;
        msg_new.Status = status_; 
        pub_state.publish(msg_new);
    }

    void run() {
        if(Interfacetype=="Show"||Interfacetype=="HoverDummy"){
            omav_client_show.start();
            omav_client_show.setHighPriority();
        } else{
            omav_client_.start();
            omav_client_.setHighPriority();
        }
    }

    void stop() {
        if(Interfacetype=="Show"||Interfacetype=="HoverDummy"){
            omav_client_show.stop();
        } else {
            omav_client_.stop();
        }
    }
};

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "AveroTestingNode");
    ros::NodeHandle nh;
    AveroTestingNode ctrl_node(nh);
    if(ros::ok()){
        ctrl_node.run();
        ctrl_node.test();
        ctrl_node.stop();
    }
  return 0;
}


//

/*This in dynamixel motor
Add t
//AVERO start
#define ADDR_OPERATING_MODE 11
#define EXT_POSITION_CONTROL_MODE       4                   // Value for extended position control mode (operating mode)
//AVERO end

      //AVERO start add multiturn
      int dxl_comm_result1 = dxl_packet_handler_->write1ByteTxRx(dxl_port_handler_, dxl_id, ADDR_OPERATING_MODE, EXT_POSITION_CONTROL_MODE, &dxl_error);
      if (dxl_comm_result1 != COMM_SUCCESS)
      {
        printf("%s\n", dxl_packet_handler_->getTxRxResult(dxl_comm_result1));
      }
      else if (dxl_error != 0)
      {
        printf("%s\n", dxl_packet_handler_->getRxPacketError(dxl_error));
      }
      else
      {
        LOG(ERROR) << "Operating mode changed to extended position control mode for Dynamixel #"<< dxl_id;
      }
      //AVERO end add multiturn


*/


// Arcghive of old Tests
/*
 // \brief Define a sequence to show at Review 3
    void ShowShow(){
        // TestingState = {PWM1, PWM2, PWM3, Base1, Top1, Base2, Top2, Base3, Top3}
        std::array<double, 9> TestingState; 
       
        int wait_time =2; 
        int wait_time_base= 1;
        ROS_INFO_STREAM("Testing 6 Dynamixel"); 
        ROS_INFO_STREAM("Go to 0");
        TestingState = SetAllStates(1050,1050,1050,0,0,0,0,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time).sleep(); 

        ROS_INFO_STREAM("Base1");
        TestingState = SetAllStates(1100,1050,1050,360,0,0,0,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time_base).sleep(); 

        ROS_INFO_STREAM("Top1");
        TestingState = SetAllStates(1100,1050,1050,360,360,0,0,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time).sleep(); 
        
        ROS_INFO_STREAM("Base2");
        TestingState = SetAllStates(1100,1050,1050,360,360,360,0,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time_base).sleep(); 
        
        ROS_INFO_STREAM("Top2");
        TestingState = SetAllStates(1100,1050,1050,360,360,360,360,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time).sleep(); 

        ROS_INFO_STREAM("Base3");
        TestingState = SetAllStates(1100,1050,1050,360,360,360,360,360,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time_base).sleep(); 

        ROS_INFO_STREAM("Top3");
        TestingState = SetAllStates(1100,1050,1050,360,360,360,360,360,360);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(wait_time).sleep(); 

        ROS_INFO_STREAM("Dance");
        TestingState = SetAllStates(1100,1050,1050,720,180,720,180,720,180);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(3).sleep();

        TestingState = SetAllStates(1100,1050,1050,600,400,500,400,400,180);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(3).sleep();

        ROS_INFO_STREAM("Go to zero");
        TestingState = SetAllStates(1100,1050,1050,0,0,0,0,0,0);
        omav_client_show.setFullState(TestingState); 
        ros::Duration(3).sleep();
    }

    void turnTest(){
        omav_client_.setFullState({1300, DegreesToRadians(0,"base"), DegreesToRadians(0,"top")});
        ros::Duration(6).sleep(); 
        omav_client_.setFullState({1300, DegreesToRadians(0,"base"), DegreesToRadians(90,"top")});
        ros::    void MocktestConfig(avero_msgs::testing_msg SetPointMSG_){
        ROS_ERROR_STREAM("Tested configuration: PWM= "<<SetPointMSG_.PWM<<", BasePositoion= "<<SetPointMSG_.BasePosition<<", TopPosition= "<<SetPointMSG_.TopPosition);
        PrintStatus(SetPointMSG_);
        ros::Duration(3.0).sleep();
    }Duration(6).sleep(); 
        omav_client_.setFullState({1200, DegreesToRadians(90,"base"), DegreesToRadians(60,"top")});
        ros::Duration(6).sleep();
    }

    void HomingTest(){
        double base=0.0;
        double top=0.0; 
        ROS_INFO_STREAM("Testing Positions!");
        for(base=0.0; base<=180; base+=45){
            for(top=0.0;top<=180; top+=45){
                if(ros::ok()){
                    ROS_INFO_STREAM(base<< "-> " <<DegreesToRadians(base,"base") <<", " << top << "-> "<<DegreesToRadians(top,"top"));
                    omav_client_.setFullState({1000, DegreesToRadians(base,"base"), DegreesToRadians(top,"top")});
                    avero_msgs::testing_msg SetPointMSG;
                    SetPointMSG.PWM =1000;
                    SetPointMSG.TopPosition= top; 
                    SetPointMSG.BasePosition =base; 
                    PrintStatus(SetPointMSG);
                    ros::Duration(2).sleep();
                }
            }
        }
    }

    void MocktestConfig(avero_msgs::testing_msg SetPointMSG_){
        ROS_ERROR_STREAM("Tested configuration: PWM= "<<SetPointMSG_.PWM<<", BasePositoion= "<<SetPointMSG_.BasePosition<<", TopPosition= "<<SetPointMSG_.TopPosition);
        PrintStatus(SetPointMSG_);
        ros::Duration(3.0).sleep();
    }

*/

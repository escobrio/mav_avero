#ifndef AVERO_ALLOCATOR_H
#define AVERO_ALLOCATOR_H

// Example on how to implement a Controller and Allocator class. Template given by Michael Pantic in meeting on Feb 28

#include <array>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "Helperfunctions.h"
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include "avero_msgs/Vector6d.h"
#include "avero_msgs/Vector9d.h"
#include "AllocationInterface.h"
//Ros message type
#include <geometry_msgs/Vector3.h>


typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 6> Matrix96d; //not sure if 9x6 or 6x9
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 6, 9> Matrix69d;


//all the constants that are platform specific should be here

namespace avero_jerk_allocator{
    static constexpr double k_f = 6*1e-5; //Motor constant
    static constexpr double pwm_min = 0; 
    static constexpr double pwm_max = 900; 
    static constexpr double optimal_pwm = 50; 
    static constexpr double max_servo_rate = 2.0; // rad/s assumed the gear ratio is included, i.e. max_servo_rate is actually phi rate
    static constexpr double derivative_gain = 1/0.005; // no noise for now assume perfect derivative
    static constexpr double max_pwm_rate = 1000; // placeholder value needs to be determined expirimentally or through datasheet
    static constexpr double gear_ratio = 1; // placeholder value needs to be determined expirimentally or through datasheet
} 


struct AveroJerkAllocatorParameters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    AveroJerkAllocatorParameters()
        : pwm_min(avero_jerk_allocator::pwm_min),
          pwm_max(avero_jerk_allocator::pwm_max),
          optimal_pwm(avero_jerk_allocator::optimal_pwm),
          max_servo_speed(avero_jerk_allocator::max_servo_rate),
          derivative_gain(avero_jerk_allocator::derivative_gain), 
          max_pwm_rate(avero_jerk_allocator::max_pwm_rate),
          k_f(avero_jerk_allocator::k_f),
          gear_ratio(avero_jerk_allocator::gear_ratio){}

    double pwm_min;
    double pwm_max;
    double optimal_pwm;
    double max_servo_speed;
    double derivative_gain;
    double max_pwm_rate;
    double k_f;
    double gear_ratio;
    double sampling_time = 0.010; //default value

    [[nodiscard]] bool isValid() const {
        bool is_valid = true;

        // check all values for nan and inf (could be improved later on)
        is_valid &= std::isfinite(pwm_min);
        is_valid &= std::isfinite(pwm_max);
        is_valid &= std::isfinite(optimal_pwm);
        is_valid &= std::isfinite(max_servo_speed);
        is_valid &= std::isfinite(derivative_gain);
        is_valid &= std::isfinite(max_pwm_rate);
        is_valid &= std::isfinite(k_f);
        is_valid &= std::isfinite(gear_ratio);  

        return is_valid;
    }
};



//in case we want to switch it off for debugging

class DummyAllocator : public AllocatorInterface {
public:
    DummyAllocator(){};
    std::array<double, 9> allocate(){
        std::array<double, 9>
        setpoints = {1, 1, 1, 0, 0, 0, 0, 0, 0};
        return setpoints;
    }
};








class AveroJerkAllocator : public AllocatorInterface
{
    public: 
        AveroJerkAllocator(bool for_simulation, bool want_debug_print, bool use_joint_feedback) {

            bool all_parameters_read = true;
            this -> use_joint_feedback = use_joint_feedback;
            this -> for_simulation = for_simulation;
            this -> want_debug_print = want_debug_print;
            if (!setAllocationParameters(params_)){
                ROS_ERROR_STREAM("Invalid parameters for AveroJerkAllocator");
            }
            if (for_simulation){
                params_.gear_ratio = 1;
            }

            if(want_debug_print){
                u_pub = nh.advertise<std_msgs::Float64MultiArray>("command/u", 1);
                u_dot_pub = nh.advertise<std_msgs::Float64MultiArray>("command/u_dot", 1);
                wrench_cmd_pub = nh.advertise<std_msgs::Float64MultiArray>("command/wrench_cmd", 1);
                wrench_alloc_pub = nh.advertise<std_msgs::Float64MultiArray>("command/wrench_alloc", 1);
                u_real_pub = nh.advertise<std_msgs::Float64MultiArray>("command/u_real", 1);

            }

            //read the pseudo inverse params from the parameter server
            all_parameters_read = all_parameters_read & nh.getParam("pseudo_inv/damping", inv_damping);
            D = Eigen::MatrixXd::Identity(6,6)*inv_damping;
            std::cout << "Damping Matrix: " << std::endl << D << std::endl;

            all_parameters_read = all_parameters_read & nh.getParam("pseudo_inv/w_pwm", w_pwm_c);
            all_parameters_read = all_parameters_read & nh.getParam("pseudo_inv/w_phi1", w_phi1_c);
            all_parameters_read = all_parameters_read & nh.getParam("pseudo_inv/w_phi2", w_phi2_c);
            Vector9d pseudo_inverse_weights;
            pseudo_inverse_weights << w_pwm_c, w_pwm_c, w_pwm_c, w_phi1_c, w_phi2_c, w_phi1_c, w_phi2_c, w_phi1_c, w_phi2_c;
            W = pseudo_inverse_weights.asDiagonal();
            std::cout << "Weight Matrix: " << std::endl << W << std::endl;

            if(all_parameters_read){
                ROS_INFO_STREAM("\033[1;32m" << "Allocator sucessfully initialized" << "\033[0m");
            }else{
                ROS_ERROR("Could not read all parameters for AveroJerkAllocator");
                ros::shutdown();
            }
            
        };


        virtual ~AveroJerkAllocator() {};

        [[nodiscard]] bool setAllocationParameters(
            const AveroJerkAllocatorParameters& params); 

        std::array<double, 9> allocate(const Vector9d& omav_state, Vector6d& wrench);
        [[nodiscard]] bool isInitialized() const override; 

        void initialize(const Vector9d& omav_state, const Vector6d& wrench); //initialize the allocator with the current state and wrench
        
    private:
        void computeJerkCmd();
        void computeWrench(const Vector9d& omav_state); //feedback wrench from allocation
        Eigen::MatrixXd computeWeightMatrix(); //adaptable also for gain scheduling if needed 
        void computeAllocationMatrix(const Vector9d& omav_state);
        void saturateActCmd(const double sampling_time);
        void update_wrench(const Vector6d& wrench); 
        void performAllocation(const double sampling_time);
        void compute_u(const double sampling_time);
        void printDebugInfo(const Vector9d& omav_state);
        bool use_joint_feedback;
        // void initialize(const Vector9d& omav_state, const Vector6d& wrench); //initialize the allocator with the current state and wrench



       
       
       
       
       
        AveroJerkAllocatorParameters params_;

        bool drone_version = true; //false for old drone version, true for new drone version

        double inv_damping; //not tunable for now
        double w_pwm_c;
        double w_phi1_c;
        double w_phi2_c;
        const double optimal_pwm_gain = 1; // placeholder for secondary control objective
        

        Eigen::MatrixXd W;
        Eigen::MatrixXd D;

        //Debugging Stuff
        bool want_debug_print;
        ros::NodeHandle nh;
        ros::Publisher u_pub;
        ros::Publisher u_dot_pub;
        ros::Publisher wrench_cmd_pub;
        ros::Publisher wrench_alloc_pub;
        ros::Publisher u_real_pub;

        


};






#endif // AVERO_ALLOCATOR_H
#ifndef ALLOCATIONINTERFACE_H
#define ALLOCATIONINTERFACE_H

#include <array>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "Helperfunctions.h"
#include <ros/ros.h>
#include "avero_msgs/Vector6d.h"
#include "avero_msgs/Vector9d.h"

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 9, 1> Vector9d;
typedef Eigen::Matrix<double, 9, 6> Matrix96d; //not sure if 9x6 or 6x9
typedef Eigen::Matrix<double, 9, 9> Matrix9d;
typedef Eigen::Matrix<double, 6, 9> Matrix69d;

//Interfaces only define what is inside but don't actually have implementations. That's why it's = 0 because later it will be implemented

class AllocatorInterface
{
public:
    AllocatorInterface(){
        desired_setpoints.fill(0);
        phi.setZero();
        u.setZero();
        pwm.setZero();
        
        
    
    };
    virtual ~AllocatorInterface(){};



    // control function will take in current UAV status and setpoints and calculate setpoints
    // Tip: Use nav_msgs/Odometry.msg
    std::array<double, 9> allocate() {
        return desired_setpoints;
    }

    void update_wrench(const Vector6d& wrench){
        this->wrench_now = wrench;
    }

    void update_omav_state(const Vector9d& omav_state){
        
        pwm = omav_state.head(3);
        phi = omav_state.tail(6);

    
    }

    [[nodiscard]] virtual bool isInitialized () const{
        bool is_initialized = true;
        is_initialized &= (prev_wrench.isZero() == false);
        is_initialized &= (wrench_now.isZero() == false);
        

        return is_initialized;
    }


    protected: 
    Vector6d prev_wrench;
    Vector6d wrench_now;
    Vector6d wrench_alloc; 
    Vector6d w_dot;
    Matrix69d allocation_matrix;
    Matrix96d allocation_matrix_inv; 
    bool for_simulation = false;
    bool is_initialized = false;


    

    std::array<double, 9> desired_setpoints; // 3 propeller velocities, 6 dynamixels positions
    

    Vector6d phi; //phi_i_j, i = propeller, j = dynamixel
    Eigen::Vector3d pwm; // pwm signals for the props
    Vector6d prev_phi;
    Eigen::Vector3d prev_pwm;
    Vector9d u;//control inputs 
    Vector9d prev_u; 
    Vector9d u_dot;

};

#endif // ALLOCATIONINTERFACE_H
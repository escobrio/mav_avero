#ifndef AVERO_CONTROLLER_H
#define AVERO_CONTROLLER_H

// Example on how to implement a Controller and Allocator class. Template given by Michael Pantic in meeting on Feb 28

// TODO implement Eigen::Vector6d instead of std::array<double, 6>
#include <array>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "Helperfunctions.h" // for sing function
#include <chrono> // für die Zeitmessung beim integrieren
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/ros.h>

//debugging
#include<std_msgs/Float32.h>

typedef Eigen::Matrix<double, 3, 1> Vector3d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 6, 6> Matrix6d;
typedef Eigen::Matrix<double, 3, 100> Vector5d;


// Interfaces only define how a class is setup but don't have any real implementations. 
// That's why the control function outputs wrench = 0, you can inherit from this Interface and implement your own control function.
class ControllerInterface
{
public:
    //all classic variables

    // Variable: expressed-frame_Variable-name_of-what
    //For Rotations: q_from-which-frame_to-which-frame

    //Model Variables
    Vector6d wrench; //3 forces and 3 torques in center of gravity, body frame
    Vector3d w_wrench_forces; //wrench forces in world frame
    Vector3d wrench_torques; //wrench torques in body frame
    Eigen::Vector3d w_gravitation = {0, 0, -9.81}; //gravitation in world frame - not sure about the sing
    Eigen::Matrix3d inertia; //in body frame in center of gravity
    double mass; //in kg



    //Controll Variables
    //desired
    Eigen::Vector3d w_desired_position; //in world frame
    Eigen::Vector3d w_desired_velocity; //in world frame
    Eigen::Quaterniond desired_q_wb; //from world to body frame
    Eigen::Quaterniond desired_q_bw; //from body to world frame
    Eigen::Vector3d w_desired_angular_velocity; //in world frame

    //current
    Eigen::Vector3d w_position; //in world frame
    Eigen::Vector3d w_velocity; //in world frame
    Eigen::Quaterniond q_wb; //from world to body frame
    Eigen::Quaterniond q_bw; //from body to world frame
    Eigen::Vector3d w_angular_velocity; //in world frame
    Vector6d b_lin_angular_velocity; //in world frame

    
    ControllerInterface() {
        //initialize all variables
        mass = 0;
        inertia.setIdentity();
    }

    ControllerInterface(double mass, Eigen::Matrix3d inertia) {
        this->mass = mass;
        this->inertia = inertia;
    }

    void update_desired(const Eigen::Vector3d& w_desired_position_in, const Eigen::Vector3d& w_desired_velocity_in, const Eigen::Quaterniond& desired_q_wb_in, const Eigen::Vector3d& w_desired_angular_velocity_in){
        //update desired values
        //Todo: implement - Ros Stuff and so on
        this->w_desired_position = w_desired_position_in;
        this->w_desired_velocity = w_desired_velocity_in;
        this->w_desired_angular_velocity = w_desired_angular_velocity_in;
        this->desired_q_wb = desired_q_wb_in;
        this->desired_q_bw = (this->desired_q_wb).inverse();
    }

    void update_current(const Eigen::Vector3d& w_position_in, const Eigen::Vector3d& w_velocity_in, const Eigen::Quaterniond& q_wb_in, const Eigen::Vector3d& w_angular_velocity_in){
        //update current values
        //Todo: implement - Ros Stuff and so on
        this->w_position = w_position_in;
        this->w_velocity = w_velocity_in;
        this->w_angular_velocity = w_angular_velocity_in;
        this->q_wb = q_wb_in;
        this->q_bw = this->q_wb.inverse();

        b_lin_angular_velocity << q_bw*w_velocity,
                                  q_bw*(this->w_angular_velocity);
    }

        /* b_lin_angular_velocity << (-w_position).cross(q_bw*w_angular_velocity) +q_bw*w_velocity,
                                  q_bw*(this->w_angular_velocity); TRYING SOMETHING DONT DELETE THIS*/
    

    // control function will take in current UAV status and setpoints and calculate wrench
    // Tip: Use nav_msgs/Odometry.msg
    Vector6d control(/*Odometry, setpoints, rc, ....*/) {
        return wrench;
    }


    // Simple printWrench member function, probably has to be rewritten when Eigen::Vector6d is implemented
    void printWrench() {
        std::cout << "Wrench: " << std::endl << wrench << std::endl;
    }
};

// PIDController inherits from ControllerInterface and implements its own control function
class PIDController : public ControllerInterface {
public:
    
    //PID gains
    Eigen::Matrix3d k_pp;
    Eigen::Matrix3d k_pd;
    Eigen::Matrix3d k_pi;

    Eigen::Matrix3d k_rp;
    Eigen::Matrix3d k_rd;
    Eigen::Matrix3d k_ri;

    //wrench begrenzung
    Vector6d max_wrench;

    //error saturation
    Vector3d max_e_p;
    Vector3d max_e_v;
    Vector3d max_e_r;
    Vector3d max_e_omega;


    //Error Vectors
    Eigen::Vector3d e_p;
    Eigen::Vector3d e_pi;
    Eigen::Vector3d e_v;

    Eigen::Vector3d e_r;
    Eigen::Vector3d e_ri;
    Eigen::Vector3d e_omega;

    //equation of motion matrices
    Matrix6d M_eqm;
    Matrix6d C_eqm;
    Vector6d g_eqm;

    // "controler input"
    Vector6d b_des_u_dot;

    //integrater variables
    bool first_call;

    double delta_t_p; //time since last control call do i need to calculate this ? - is it allways the same ?
    std::chrono::time_point<std::chrono::high_resolution_clock> last_call_p;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_now_p;

    double delta_t_r;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_call_r;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_now_r;

    //moving average filter for the velocity
    Eigen::MatrixXd last_w_velocities;
    int length_maf_velocities;


    



    PIDController() {

        bool all_parameters_read = true;
        //PID gains
        double p_kpx;
        double p_kpy;
        double p_kpz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kpx", p_kpx);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kpy", p_kpy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kpz", p_kpz);
        k_pp << p_kpx, 0, 0,
                0, p_kpy, 0,
                0, 0, p_kpz;

        double p_kdx;
        double p_kdy;
        double p_kdz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kdx", p_kdx);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kdy", p_kdy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kdz", p_kdz);
        k_pd << p_kdx, 0, 0,
                0, p_kdy, 0,
                0, 0, p_kdz;
        
        double p_kix;
        double p_kiy;
        double p_kiz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kix", p_kix);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kiy", p_kiy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/p_kiz", p_kiz);
        k_pi << p_kix, 0, 0,
                0, p_kiy, 0,
                0, 0, p_kiz;
        
        double r_kpx;
        double r_kpy;
        double r_kpz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kpx", r_kpx);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kpy", r_kpy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kpz", r_kpz);
        k_rp << r_kpx, 0, 0,
                0, r_kpy, 0,
                0, 0, r_kpz;
        
        double r_kdx;
        double r_kdy;
        double r_kdz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kdx", r_kdx);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kdy", r_kdy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kdz", r_kdz);
        k_rd << r_kdx, 0, 0,
                0, r_kdy, 0,
                0, 0, r_kdz;
        
        double r_kix;
        double r_kiy;
        double r_kiz;
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kix", r_kix);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kiy", r_kiy);
        all_parameters_read = all_parameters_read & nh.getParam("pid_gains/r_kiz", r_kiz);
        k_ri << r_kix, 0, 0,
                0, r_kiy, 0,
                0, 0, r_kiz;

        //get the model params
        nh.getParam("model_params/mass", mass);
        double Ixx;
        double Iyy;
        double Izz;
        all_parameters_read = all_parameters_read & nh.getParam("model_params/Ixx", Ixx);
        all_parameters_read = all_parameters_read & nh.getParam("model_params/Iyy", Iyy);
        all_parameters_read = all_parameters_read & nh.getParam("model_params/Izz", Izz);
        inertia << Ixx, 0, 0,
                   0, Iyy, 0,
                   0, 0, Izz;

        //set the max wrench
        double w_px;
        double w_py;
        double w_pz;
        double w_rx;
        double w_ry;
        double w_rz;
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/px", w_px);
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/py", w_py);
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/pz", w_pz);
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/rx", w_rx);
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/ry", w_ry);
        all_parameters_read = all_parameters_read & nh.getParam("max_wrench/rz", w_rz);
        max_wrench << w_px, w_py, w_pz, w_rx, w_ry, w_rz;

        //set the error maxima
        double e_px_max;
        double e_py_max;
        double e_pz_max;
        double e_vx_max;
        double e_vy_max;
        double e_vz_max;
        double e_rx_max;
        double e_ry_max;
        double e_rz_max;
        double e_omegax_max;
        double e_omegay_max;
        double e_omegaz_max;
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/px", e_px_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/py", e_py_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/pz", e_pz_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/vx", e_vx_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/vy", e_vy_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/vz", e_vz_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/rx", e_rx_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/ry", e_ry_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/rz", e_rz_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/omegax", e_omegax_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/omegay", e_omegay_max);
        all_parameters_read = all_parameters_read & nh.getParam("max_errors/omegaz", e_omegaz_max);
        max_e_p << e_px_max, e_py_max, e_pz_max;
        max_e_v << e_vx_max, e_vy_max, e_vz_max;
        max_e_r << e_rx_max, e_ry_max, e_rz_max;
        max_e_omega << e_omegax_max, e_omegay_max, e_omegaz_max;

        
        //equation of motion matrices
        this->M_eqm << mass*Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(),
                       Eigen::Matrix3d::Zero(), inertia;
            

        //setting all errors to zero for the begining
        e_p.setZero();
        e_pi.setZero();
        e_v.setZero();
        e_r.setZero();
        e_ri.setZero();
        e_omega.setZero();
        this->first_call = true;

        //moving average filter for the velocity
        nh.getParam("filter_params/maf_vlength", length_maf_velocities);
        last_w_velocities = Eigen::MatrixXd(3, length_maf_velocities);

        std::cout << "kpp:"<< std::endl << k_pp << std::endl;
        std::cout << "kpd:"<< std::endl << k_pd << std::endl;
        std::cout << "kpi:"<< std::endl << k_pi << std::endl;
        std::cout << "krp:"<< std::endl << k_rp << std::endl;
        std::cout << "krd:"<< std::endl << k_rd << std::endl;
        std::cout << "kri:"<< std::endl << k_ri << std::endl;
        std::cout << "mass:"<< std::endl << mass << std::endl;
        std::cout << "inertia:"<< std::endl << inertia << std::endl;
        std::cout << "max_wrench:"<< std::endl << max_wrench << std::endl;
        std::cout << "max_e_p:"<< std::endl << max_e_p << std::endl;
        std::cout << "max_e_v:"<< std::endl << max_e_v << std::endl;
        std::cout << "max_e_r:"<< std::endl << max_e_r << std::endl;
        std::cout << "max_e_omega:"<< std::endl << max_e_omega << std::endl;
       

        //setting up the debuger publisher
        marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
        e_p_pub = nh.advertise<geometry_msgs::Vector3>("e_p", 10);
        e_v_pub = nh.advertise<geometry_msgs::Vector3>("e_v", 10);
        e_r_pub = nh.advertise<geometry_msgs::Vector3>("e_r", 10);
        e_omega_pub = nh.advertise<geometry_msgs::Vector3>("e_omega", 10);
        b_v_mes_pub = nh.advertise<geometry_msgs::Vector3>("b_v_mes", 10); 
        b_w_mes_pub = nh.advertise<geometry_msgs::Vector3>("b_w_mes", 10);
        q_wb_pub = nh.advertise<geometry_msgs::Quaternion>("q_wb", 10);
        wrench_force_pub = nh.advertise<geometry_msgs::Vector3>("wrench_force", 10);
        wrench_torque_pub = nh.advertise<geometry_msgs::Vector3>("wrench_torque", 10);
        e_p_xy_norm_pub = nh.advertise<std_msgs::Float32>("e_p_xy_norm", 10);
        wrench_force_xy_norm_pub = nh.advertise<std_msgs::Float32>("wrench_force_xy_norm", 10);
        w_desired_velocity_pub = nh.advertise<geometry_msgs::Vector3>("w_desired_velocity", 10);
        w_velocity_pub = nh.advertise<geometry_msgs::Vector3>("w_velocity", 10);
        filtered_velocity_pub = nh.advertise<geometry_msgs::Vector3>("filtered_velocity", 10);

        if(all_parameters_read){
            ROS_INFO_STREAM("\033[1;32m" << "Wrench controller sucessfully initialized" << "\033[0m");
        }else{
            ROS_ERROR("Could not read all parameters for AveroController");
            ros::shutdown();
        }
}

    

    void update_ep_epi(){
        //Eigen::Matrix3d R_bw = q_bw.toRotationMatrix();
        e_p =  q_bw * (w_desired_position - w_position);
        e_p = limitErrorVector(e_p, max_e_p);


        // //Integrator
        // if (first_call){
        //     last_call_p = std::chrono::high_resolution_clock::now();
        //     e_pi.setZero();
        //     return;
        // }


        // time_now_p = std::chrono::high_resolution_clock::now();
        // delta_t_p = std::chrono::duration_cast<std::chrono::duration<double>>(time_now_p - last_call_p).count();

        // e_pi << e_pi + e_p * delta_t_p;
        // last_call_p = time_now_p;
    }

    void update_e_v(){
        
        Eigen::Matrix3d R_bw = q_bw.toRotationMatrix();

        //if length_maf_velocities is 1, no filter is used

        // if(length_maf_velocities > 1){
        //     Eigen::MatrixXd last_w_velocitiescopy(3, length_maf_velocities-1);
        //     Eigen::Vector3d w_velocity_filtered;

        //     last_w_velocitiescopy = last_w_velocities.block(0, 0, 3, length_maf_velocities-1);

        //     last_w_velocities << w_velocity, last_w_velocitiescopy;
        //     w_velocity_filtered = last_w_velocities.rowwise().mean();
        //     e_v = R_bw * (w_desired_velocity - w_velocity_filtered);
            
        //     // publish the filtered velocity
        //     geometry_msgs::Vector3 filtered_velocity_msg;
        //     filtered_velocity_msg.x = w_velocity_filtered(0);
        //     filtered_velocity_msg.y = w_velocity_filtered(1);
        //     filtered_velocity_msg.z = w_velocity_filtered(2);
        //     filtered_velocity_pub.publish(filtered_velocity_msg);
            
        //     //std::cout << last_w_velocities << std::endl;
            
        e_v = R_bw * (w_desired_velocity - w_velocity);
        e_v = limitErrorVector(e_v, max_e_v);
        
    }

    void update_er_eri(){
        Eigen::Quaterniond q_error = q_bw * desired_q_wb;
        e_r = q_error.vec();

        // //Integrator
        // if (first_call){
        //     last_call_r = std::chrono::high_resolution_clock::now();
        //     e_ri.setZero();
        //     return;
        // }

        // time_now_r = std::chrono::high_resolution_clock::now();
        // delta_t_r = std::chrono::duration_cast<std::chrono::duration<double>>(time_now_r - last_call_r).count();

        // e_ri << (e_ri + e_r * delta_t_r);
        // last_call_r = time_now_r;


        //add the sign
        e_r = sign(&q_error.w()) * e_r;
        e_r = limitErrorVector(e_r, max_e_r);
    }    

    void update_e_omega(){
        
        e_omega = q_bw * (desired_q_wb * (q_bw * w_desired_angular_velocity)) - q_bw*w_angular_velocity;
        e_omega = limitErrorVector(e_omega, max_e_omega);
        
    }

    void update_b_des_u_dot(){
        b_des_u_dot << k_pp * e_p + k_pd * e_v + k_pi * e_pi,
                        k_rp * e_r + k_rd * e_omega + k_ri * e_ri;
    }

    void update_C_g_eqm(){
        C_eqm << skewSymmetric(q_bw * w_angular_velocity)*mass, Eigen::Matrix3d::Zero(),
                Eigen::Matrix3d::Zero(), -skewSymmetric(inertia*(q_bw*w_angular_velocity));

        g_eqm << -mass*(q_bw*w_gravitation),
                 Eigen::Vector3d::Zero();

    }

    void set_wrench(){
        
        wrench = M_eqm * b_des_u_dot + C_eqm * b_lin_angular_velocity  + g_eqm;
        
        // saturate forces in world frame
        w_wrench_forces = q_wb * wrench.head(3);
        w_wrench_forces = limitErrorVector(w_wrench_forces, max_wrench.head(3));
        
        // saturate torques in body frame
        wrench_torques = wrench.tail(3);
        wrench_torques = limitErrorVector(wrench_torques, max_wrench.tail(3));
        
        // rotate forces back to body frame
        wrench << q_bw * w_wrench_forces, wrench_torques;
        
    }

    void print_debugging() {
        std::cout << "next print---------------------------------------------------" << std::endl;

        std::cout << "w_position: " << w_position.transpose() << std::endl;
    
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time::now();
        marker.ns = "wrench_vector";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05; // Decreased scale for smaller vector
        marker.scale.y = 0.01; // Decreased scale for smaller vector
        marker.scale.z = 0.015; // Decreased scale for smaller vector
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.points.resize(2);
        marker.points[0].x = 0.0;
        marker.points[0].y = 0.0;
        marker.points[0].z = 0.0;
        marker.points[1].x = ((wrench)(0))/10;
        marker.points[1].y = ((wrench)(1))/10;
        marker.points[1].z = ((wrench)(2))/10;
        
        //publish the marker
        marker_pub.publish(marker);

        //publish the error vectors
        geometry_msgs::Vector3 e_p_msg;
        e_p_msg.x = e_p(0);
        e_p_msg.y = e_p(1);
        e_p_msg.z = e_p(2);
        e_p_pub.publish(e_p_msg);

        geometry_msgs::Vector3 e_v_msg;
        e_v_msg.x = e_v(0);
        e_v_msg.y = e_v(1);
        e_v_msg.z = e_v(2);
        e_v_pub.publish(e_v_msg);


        geometry_msgs::Vector3 e_r_msg;
        e_r_msg.x = e_r(0);
        e_r_msg.y = e_r(1);
        e_r_msg.z = e_r(2);
        e_r_pub.publish(e_r_msg);

        geometry_msgs::Vector3 e_omega_msg;
        e_omega_msg.x = e_omega(0);
        e_omega_msg.y = e_omega(1);
        e_omega_msg.z = e_omega(2);
        e_omega_pub.publish(e_omega_msg);

        //b_des_u_dotle for smaller vector
        marker.scale.z = 0.015; // Decreased sc
        geometry_msgs::Wrench b_des_u_dot_v_msg;
        geometry_msgs::Wrench b_des_u_dot_w_msg;
        b_des_u_dot_v_msg.force.x = b_lin_angular_velocity(0);
        b_des_u_dot_v_msg.force.y = b_lin_angular_velocity(1);
        b_des_u_dot_v_msg.force.z = b_lin_angular_velocity(2);
        b_des_u_dot_w_msg.torque.x = b_lin_angular_velocity(3);
        b_des_u_dot_w_msg.torque.y = b_lin_angular_velocity(4);
        b_des_u_dot_w_msg.torque.z = b_lin_angular_velocity(5);
        b_v_mes_pub.publish(b_des_u_dot_v_msg);
        b_w_mes_pub.publish(b_des_u_dot_w_msg);
        
        //publish the q_wb
        geometry_msgs::Quaternion q_wb_msg;
        q_wb_msg.w = q_wb.w();
        q_wb_msg.x = q_wb.x();
        q_wb_msg.y = q_wb.y();
        q_wb_msg.z = q_wb.z();
        q_wb_pub.publish(q_wb_msg);

        //publish the wrench force
        geometry_msgs::Vector3 wrench_force_msg;
        wrench_force_msg.x = wrench(0);
        wrench_force_msg.y = wrench(1);
        wrench_force_msg.z = wrench(2);
        wrench_force_pub.publish(wrench_force_msg);

        //publish the wrench torque
        geometry_msgs::Vector3 wrench_torque_msg;
        wrench_torque_msg.x = wrench(3);
        wrench_torque_msg.y = wrench(4);
        wrench_torque_msg.z = wrench(5);
        wrench_torque_pub.publish(wrench_torque_msg);

        //publish norm of error xy
        std_msgs::Float32 e_p_xy_msg;
        e_p_xy_msg.data = (e_p(0)*e_p(0) + e_p(1)*e_p(1))*100;
        e_p_xy_norm_pub.publish(e_p_xy_msg);

        //publish norm of wrench xy
        std_msgs::Float32 wrench_force_xy_msg;
        wrench_force_xy_msg.data = 0.1*(wrench(0)*wrench(0) + wrench(1)*wrench(1));
        wrench_force_xy_norm_pub.publish(wrench_force_xy_msg);

        //w_desired_vel
        geometry_msgs::Vector3 w_desired_velocity_msg;
        w_desired_velocity_msg.x = w_desired_velocity(0);
        w_desired_velocity_msg.y = w_desired_velocity(1);
        w_desired_velocity_msg.z = w_desired_velocity(2);
        w_desired_velocity_pub.publish(w_desired_velocity_msg);

        //w_velocity
        geometry_msgs::Vector3 w_velocity_msg;
        w_velocity_msg.x = w_velocity(0);
        w_velocity_msg.y = w_velocity(1);
        w_velocity_msg.z = w_velocity(2);
        w_velocity_pub.publish(w_velocity_msg);



    }



    // control function outputs a unit wrench in z
    Vector6d control(const Eigen::Vector3d& w_desired_position_in, const Eigen::Vector3d& w_desired_velocity_in, const Eigen::Quaterniond& desired_q_wb_in, const Eigen::Vector3d& w_desired_angular_velocity_in,
                     const Eigen::Vector3d& w_position_in, const Eigen::Vector3d& w_velocity_in, const Eigen::Quaterniond& q_wb_in, const Eigen::Vector3d& w_angular_velocity_in) {
        
        update_desired(w_desired_position_in, w_desired_velocity_in, desired_q_wb_in, w_desired_angular_velocity_in);
        update_current(w_position_in, w_velocity_in, q_wb_in, w_angular_velocity_in); 
        

        //calculate errors
        update_ep_epi();
        update_e_v();
        update_er_eri();
        update_e_omega();

        //calculate EQM Input
        update_b_des_u_dot();
        update_C_g_eqm();
        set_wrench();

        

        // print_debugging();

        // if((!first_call) && (w_position(2) == 0)){
        //     ROS_ERROR("Crash");
        //     ros::shutdown();
        // }
        
        if (this->first_call){
            this->first_call = false;
            wrench.setZero();
        }
    
        return wrench;
    }

    private: //für debugging
        ros::NodeHandle nh;
        ros::Publisher marker_pub;
        ros::Publisher e_p_pub;
        ros::Publisher e_v_pub;
        ros::Publisher e_r_pub;
        ros::Publisher e_omega_pub;
        ros::Publisher b_v_mes_pub;
        ros::Publisher b_w_mes_pub;
        ros::Publisher q_wb_pub;
        ros::Publisher wrench_force_pub;
        ros::Publisher wrench_torque_pub;
        ros::Publisher e_p_xy_norm_pub;
        ros::Publisher wrench_force_xy_norm_pub;
        ros::Publisher w_desired_velocity_pub;
        ros::Publisher w_velocity_pub;
        ros::Publisher filtered_velocity_pub;
        

};


// test() {
//     auto controllah = new Controller(....gains, inertia, masses....)
//     controller.control(tothemoon)
// }

#endif // AVERO_CONTROLLER_H
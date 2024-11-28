#include <array>
#include <eigen3/Eigen/Dense> // hat to do this, for me it does not find #include <Eigen/Dense> only witheigen3/ infront
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "Helperfunctions.h"
#include <ros/ros.h>
#include "avero_msgs/Vector6d.h"
#include "avero_msgs/Vector9d.h"
#include "AllocationInterface.h"






class DifferentialAllocator : public AllocatorInterface
{

public:
    

    DifferentialAllocator(bool for_sim){

       
        this->for_sim = for_sim;
        
        wrench_now.setZero();
        wrench_int.setZero();
        first_call = true;
        

        if(for_sim){
            uebersetzung = 1.0;
        }else{
            uebersetzung = 1.0; //don't kno the real value yet
        }
        
        desired_setpoints.fill(0.0);
        u.setZero();

        //debugging publishers
        wrench_dot_pub = nh.advertise<avero_msgs::Vector6d>("wrench_dot", 1);
        u_dot_pub = nh.advertise<avero_msgs::Vector9d>("u_dot", 1);
        setpoints_pub = nh.advertise<avero_msgs::Vector9d>("setpoints", 1);
        wrench_int_pub = nh.advertise<avero_msgs::Vector6d>("wrench_int", 1);

        count = 0;

        double w_pwm_c;
        double w_phi1_c;
        double w_phi2_c;
        nh.getParam("pseudo_inv/w_pwm", w_pwm_c);
        nh.getParam("pseudo_inv/w_phi1", w_phi1_c);
        nh.getParam("pseudo_inv/w_phi2", w_phi2_c);
        nh.getParam("pseudo_inv/damping", damping_coefficient);

        pseudo_inverse_weights << w_pwm_c, w_pwm_c, w_pwm_c, w_phi1_c, w_phi2_c, w_phi1_c, w_phi2_c, w_phi1_c, w_phi2_c;

        //moving average Filter setup
        average_length = 100;
        last_wrenches = Eigen::MatrixXd(6, average_length);
        last_wrenches.setZero();

    }

    void update_wrench(const Vector6d& wrench){
        this->wrench_before = this->wrench_now;

        last_wrenches.block(0,1,6,average_length-1) = last_wrenches.block(0,0,6,average_length-1);
        last_wrenches.col(0) = wrench;

        if(count + 1  < average_length){
            Vector6d wrench_sum = last_wrenches.rowwise().sum();
            this->wrench_now = wrench_sum / (count + 1);
            
        }else{
            this->wrench_now = last_wrenches.rowwise().sum() / average_length;
        }
        

        
    }

    void update_omav_state(const Vector9d& omav_state){
        //mit uebersetzung !
        //nicht sicher ob die velicities aus omav_status wirklich stimmen
        //now not implementet via hovery motor plugin but via gazebo plugin
        if (for_sim){
            pwm = pwm_ranger(omav_state.head(3), pwm_min, 100000000000) ;
            phi = omav_state.tail(6) ;

            if(pwm.isZero()){
                pwm << 1.0, 1.0, 1.0;
                std::cout << "pwm was zero now 1.0 1.0 1.0" << std::endl;
            }

        }else{
            //pwm = omav_state.head(3);
            //phi = omav_state.tail(6)*uebersetzung;
        }

        
    }

    void calculate_wrench_dot(){

        //--------------------------------------------------------------------------------!!!
        // ich bin mir nicht ganz sicher was die genauste art ist dass numerisch abzuleiten, 
        // messe ich delta t in nanosekunden und dann durch 10^9 dividieren oder soll ich das
        //  delta t in nanosekunden lassen aber danach wrech dot durch 10^9 dividieren ?
        //--------------------------------------------------------------------------------!!!

        //momentan wird das delta_t direkt in sekunden umgerechnet

        time_now_d = std::chrono::high_resolution_clock::now();
    
        if(first_call){
            last_call_d = time_now_d;
            wrench_dot.setZero();
            return;
        }

        Vector6d delta_wrench = wrench_now - wrench_before;
        double delta_t_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now_d - last_call_d).count();
        double delta_t_seconds = static_cast<double>(delta_t_nanoseconds) / 1'000'000'000.0;
        


        // wrench_dot = delta_wrench / delta_t_seconds;
        wrench_dot = delta_wrench / 0.01;        
        last_call_d = time_now_d;
    }

    void update_A(){
        double pwm_1 = pwm(0);
        double pwm_2 = pwm(1);
        double pwm_3 = pwm(2);

        double phi_1_1 = phi(0);
        double phi_1_2 = phi(1);
        double phi_2_1 = phi(2);
        double phi_2_2 = phi(3);
        double phi_3_1 = phi(4);
        double phi_3_2 = phi(5);

        A.setZero();

        A.row(0) << k_f*pwm_1*(-8.65956056235493e-17*sin(phi_1_1)*sin(phi_1_2) - 0.5*sin(phi_1_1)*cos(phi_1_2) - 0.5*sin(phi_1_1) - 0.707106781186547*sin(phi_1_2)*cos(phi_1_1) + 6.12323399573677e-17*cos(phi_1_1)*cos(phi_1_2) + 6.12323399573677e-17*cos(phi_1_1) + 0.866025403784438*cos(phi_1_2) - 0.866025403784439), 
                    k_f*pwm_2*(-8.65956056235493e-17*sin(phi_2_1)*sin(phi_2_2) - 0.5*sin(phi_2_1)*cos(phi_2_2) - 0.5*sin(phi_2_1) - 0.707106781186547*sin(phi_2_2)*cos(phi_2_1) + 6.12323399573677e-17*cos(phi_2_1)*cos(phi_2_2) + 6.12323399573677e-17*cos(phi_2_1) - 0.866025403784438*cos(phi_2_2) + 0.866025403784439), 
                    k_f*pwm_3*(-8.65956056235493e-17*sin(phi_3_1)*sin(phi_3_2) + 0.866025403784439*sin(phi_3_1)*cos(phi_3_2) + 0.866025403784439*sin(phi_3_1) + 1.22474487139159*sin(phi_3_2)*cos(phi_3_1) + 6.12323399573677e-17*cos(phi_3_1)*cos(phi_3_2) + 6.12323399573677e-17*cos(phi_3_1) - 0.5*cos(phi_3_2) + 0.5), 
                    k_f*std::pow(pwm_1, 2.0)*(0.353553390593274*sin(phi_1_1)*sin(phi_1_2) - 3.06161699786838e-17*sin(phi_1_1)*cos(phi_1_2) - 3.06161699786838e-17*sin(phi_1_1) - 4.32978028117747e-17*sin(phi_1_2)*cos(phi_1_1) - 0.25*cos(phi_1_1)*cos(phi_1_2) - 0.25*cos(phi_1_1)), 
                    k_f*std::pow(pwm_1, 2.0)*(0.25*sin(phi_1_1)*sin(phi_1_2) - 4.32978028117747e-17*sin(phi_1_1)*cos(phi_1_2) - 3.06161699786838e-17*sin(phi_1_2)*cos(phi_1_1) - 0.433012701892219*sin(phi_1_2) - 0.353553390593274*cos(phi_1_1)*cos(phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.353553390593274*sin(phi_2_1)*sin(phi_2_2) - 3.06161699786838e-17*sin(phi_2_1)*cos(phi_2_2) - 3.06161699786838e-17*sin(phi_2_1) - 4.32978028117747e-17*sin(phi_2_2)*cos(phi_2_1) - 0.25*cos(phi_2_1)*cos(phi_2_2) - 0.25*cos(phi_2_1)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.25*sin(phi_2_1)*sin(phi_2_2) - 4.32978028117747e-17*sin(phi_2_1)*cos(phi_2_2) - 3.06161699786838e-17*sin(phi_2_2)*cos(phi_2_1) + 0.433012701892219*sin(phi_2_2) - 0.353553390593274*cos(phi_2_1)*cos(phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(-0.612372435695794*sin(phi_3_1)*sin(phi_3_2) - 3.06161699786838e-17*sin(phi_3_1)*cos(phi_3_2) - 3.06161699786838e-17*sin(phi_3_1) - 4.32978028117747e-17*sin(phi_3_2)*cos(phi_3_1) + 0.433012701892219*cos(phi_3_1)*cos(phi_3_2) + 0.433012701892219*cos(phi_3_1)), 
                    k_f*std::pow(pwm_3, 2.0)*(-0.433012701892219*sin(phi_3_1)*sin(phi_3_2) - 4.32978028117747e-17*sin(phi_3_1)*cos(phi_3_2) - 3.06161699786838e-17*sin(phi_3_2)*cos(phi_3_1) + 0.25*sin(phi_3_2) + 0.612372435695794*cos(phi_3_1)*cos(phi_3_2));

        A.row(1) << k_f*pwm_1*(0.866025403784439*sin(phi_1_1)*cos(phi_1_2) + 0.866025403784439*sin(phi_1_1) + 1.22474487139159*sin(phi_1_2)*cos(phi_1_1) + 0.5*cos(phi_1_2) - 0.5), 
                    k_f*pwm_2*(-0.866025403784439*sin(phi_2_1)*cos(phi_2_2) - 0.866025403784439*sin(phi_2_1) - 1.22474487139159*sin(phi_2_2)*cos(phi_2_1) + 0.5*cos(phi_2_2) - 0.5),
                    k_f*pwm_3*(-0.5*sin(phi_3_1)*cos(phi_3_2) - 0.5*sin(phi_3_1) - 0.707106781186547*sin(phi_3_2)*cos(phi_3_1) - 0.866025403784438*cos(phi_3_2) + 0.866025403784439), 
                    k_f*std::pow(pwm_1, 2.0)*(-0.612372435695794*sin(phi_1_1)*sin(phi_1_2) + 0.433012701892219*cos(phi_1_1)*cos(phi_1_2) + 0.433012701892219*cos(phi_1_1)), 
                    k_f*std::pow(pwm_1, 2.0)*(-0.433012701892219*sin(phi_1_1)*sin(phi_1_2) - 0.25*sin(phi_1_2) + 0.612372435695794*cos(phi_1_1)*cos(phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.612372435695794*sin(phi_2_1)*sin(phi_2_2) - 0.433012701892219*cos(phi_2_1)*cos(phi_2_2) - 0.433012701892219*cos(phi_2_1)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.433012701892219*sin(phi_2_1)*sin(phi_2_2) - 0.25*sin(phi_2_2) - 0.612372435695794*cos(phi_2_1)*cos(phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(0.353553390593274*sin(phi_3_1)*sin(phi_3_2) - 0.25*cos(phi_3_1)*cos(phi_3_2) - 0.25*cos(phi_3_1)), 
                    k_f*std::pow(pwm_3, 2.0)*(0.25*sin(phi_3_1)*sin(phi_3_2) + 0.433012701892219*sin(phi_3_2) - 0.353553390593274*cos(phi_3_1)*cos(phi_3_2));

        A.row(2) << k_f*pwm_1*(-1.41421356237309*sin(phi_1_1)*sin(phi_1_2) + 3.06161699786838e-17*sin(phi_1_1)*cos(phi_1_2) + 3.06161699786838e-17*sin(phi_1_1) + 4.32978028117746e-17*sin(phi_1_2)*cos(phi_1_1) + 1.0*cos(phi_1_1)*cos(phi_1_2) + 1.0*cos(phi_1_1) - 5.30287619362453e-17*cos(phi_1_2) + 5.30287619362454e-17), 
                    k_f*pwm_2*(-1.41421356237309*sin(phi_2_1)*sin(phi_2_2) + 3.06161699786838e-17*sin(phi_2_1)*cos(phi_2_2) + 3.06161699786838e-17*sin(phi_2_1) + 4.32978028117746e-17*sin(phi_2_2)*cos(phi_2_1) + 1.0*cos(phi_2_1)*cos(phi_2_2) + 1.0*cos(phi_2_1) + 5.30287619362453e-17*cos(phi_2_2) - 5.30287619362454e-17), 
                    k_f*pwm_3*(-1.41421356237309*sin(phi_3_1)*sin(phi_3_2) - 5.30287619362453e-17*sin(phi_3_1)*cos(phi_3_2) - 5.30287619362453e-17*sin(phi_3_1) - 7.49939943260923e-17*sin(phi_3_2)*cos(phi_3_1) + 1.0*cos(phi_3_1)*cos(phi_3_2) + 1.0*cos(phi_3_1) + 3.06161699786838e-17*cos(phi_3_2) - 3.06161699786838e-17), 
                    k_f*std::pow(pwm_1, 2.0)*(-2.16489014058873e-17*sin(phi_1_1)*sin(phi_1_2) - 0.5*sin(phi_1_1)*cos(phi_1_2) - 0.5*sin(phi_1_1) - 0.707106781186547*sin(phi_1_2)*cos(phi_1_1) + 1.53080849893419e-17*cos(phi_1_1)*cos(phi_1_2) + 1.53080849893419e-17*cos(phi_1_1)), 
                    k_f*std::pow(pwm_1, 2.0)*(-1.53080849893419e-17*sin(phi_1_1)*sin(phi_1_2) - 0.707106781186547*sin(phi_1_1)*cos(phi_1_2) - 0.5*sin(phi_1_2)*cos(phi_1_1) + 2.65143809681227e-17*sin(phi_1_2) + 2.16489014058873e-17*cos(phi_1_1)*cos(phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(-2.16489014058873e-17*sin(phi_2_1)*sin(phi_2_2) - 0.5*sin(phi_2_1)*cos(phi_2_2) - 0.5*sin(phi_2_1) - 0.707106781186547*sin(phi_2_2)*cos(phi_2_1) + 1.53080849893419e-17*cos(phi_2_1)*cos(phi_2_2) + 1.53080849893419e-17*cos(phi_2_1)), 
                    k_f*std::pow(pwm_2, 2.0)*(-1.53080849893419e-17*sin(phi_2_1)*sin(phi_2_2) - 0.707106781186547*sin(phi_2_1)*cos(phi_2_2) - 0.5*sin(phi_2_2)*cos(phi_2_1) - 2.65143809681227e-17*sin(phi_2_2) + 2.16489014058873e-17*cos(phi_2_1)*cos(phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(3.74969971630462e-17*sin(phi_3_1)*sin(phi_3_2) - 0.5*sin(phi_3_1)*cos(phi_3_2) - 0.5*sin(phi_3_1) - 0.707106781186547*sin(phi_3_2)*cos(phi_3_1) - 2.65143809681227e-17*cos(phi_3_1)*cos(phi_3_2) - 2.65143809681227e-17*cos(phi_3_1)), 
                    k_f*std::pow(pwm_3, 2.0)*(2.65143809681227e-17*sin(phi_3_1)*sin(phi_3_2) - 0.707106781186547*sin(phi_3_1)*cos(phi_3_2) - 0.5*sin(phi_3_2)*cos(phi_3_1) - 1.53080849893419e-17*sin(phi_3_2) - 3.74969971630462e-17*cos(phi_3_1)*cos(phi_3_2));
        
        A.row(3) << -k_f*pwm_1*(-6.50843912306034e-18*sin(phi_1_1) - 0.0269609964658858*sin(phi_1_2) - 1.73472347597681e-18*sin(2*phi_1_2) - 4.81482486096809e-35*sin(phi_1_1 - 2*phi_1_2) + 9.53138842102274e-19*sin(phi_1_1 - phi_1_2) - 1.01574617198139e-17*sin(phi_1_1 + phi_1_2) + 4.81482486096809e-35*sin(phi_1_1 + 2*phi_1_2) + 8.67361737988404e-19*sin(2*phi_1_1 - 2*phi_1_2) - 8.67361737988404e-19*sin(2*phi_1_1 + 2*phi_1_2) - 0.0531454385671997*cos(phi_1_1) - 7.22223729145213e-35*cos(2*phi_1_1) + 9.62964972193618e-35*cos(2*phi_1_2) + 0.00778296928359985*cos(phi_1_1 - phi_1_2) - 0.0829419692835998*cos(phi_1_1 + phi_1_2) - 1.73472347597681e-18*cos(phi_1_1 + 2*phi_1_2) - 9.62964972193618e-35*cos(2*phi_1_1 - phi_1_2) + 9.62964972193618e-35*cos(2*phi_1_1 + phi_1_2) + 9.62964972193618e-35*cos(2*phi_1_1 + 2*phi_1_2) - 1.20370621524202e-34), 
                    k_f*pwm_2*(-6.50843912306034e-18*sin(phi_2_1) + 0.0269609964658858*sin(phi_2_2) + 1.73472347597681e-18*sin(2*phi_2_2) - 4.81482486096809e-35*sin(phi_2_1 - 2*phi_2_2) + 9.53138842102274e-19*sin(phi_2_1 - phi_2_2) - 1.01574617198139e-17*sin(phi_2_1 + phi_2_2) + 4.81482486096809e-35*sin(phi_2_1 + 2*phi_2_2) - 8.67361737988404e-19*sin(2*phi_2_1 - 2*phi_2_2) + 8.67361737988404e-19*sin(2*phi_2_1 + 2*phi_2_2) - 0.0531454385671997*cos(phi_2_1) + 7.22223729145213e-35*cos(2*phi_2_1) - 9.62964972193618e-35*cos(2*phi_2_2) + 0.00778296928359985*cos(phi_2_1 - phi_2_2) - 0.0829419692835998*cos(phi_2_1 + phi_2_2) - 1.73472347597681e-18*cos(phi_2_1 + 2*phi_2_2) + 9.62964972193618e-35*cos(2*phi_2_1 - phi_2_2) - 9.62964972193618e-35*cos(2*phi_2_1 + phi_2_2) - 9.62964972193618e-35*cos(2*phi_2_1 + 2*phi_2_2) + 1.20370621524202e-34), 
                    k_f*pwm_3*(-6.50843912306035e-18*sin(phi_3_1) - 1.73472347597681e-18*sin(2*phi_3_1) + 0.0155659385671997*sin(phi_3_2) + 8.67361737988404e-19*sin(2*phi_3_2) + 4.81482486096809e-35*sin(phi_3_1 - 2*phi_3_2) + 9.53138842102275e-19*sin(phi_3_1 - phi_3_2) - 1.01574617198139e-17*sin(phi_3_1 + phi_3_2) - 1.44444745829043e-34*sin(phi_3_1 + 2*phi_3_2) + 1.30104260698261e-18*sin(2*phi_3_1 - 2*phi_3_2) + 4.33680868994202e-19*sin(2*phi_3_1 + 2*phi_3_2) + 0.0920505997889205*cos(phi_3_1) + 3.85185988877447e-34*cos(phi_3_2) - 0.0134804982329429*cos(phi_3_1 - phi_3_2) + 0.143659704879012*cos(phi_3_1 + phi_3_2) + 3.46944695195361e-18*cos(phi_3_1 + 2*phi_3_2) + 4.81482486096809e-35*cos(2*phi_3_1 - 2*phi_3_2) - 4.81482486096809e-35*cos(2*phi_3_1 + 2*phi_3_2) + 3.85185988877447e-34), 
                    k_f*std::pow(pwm_1, 2.0)*(0.0265727192835999*sin(phi_1_1) + 7.22223729145213e-35*sin(2*phi_1_1) - 0.00389148464179993*sin(phi_1_1 - phi_1_2) + 0.0414709846417999*sin(phi_1_1 + phi_1_2) + 8.67361737988404e-19*sin(phi_1_1 + 2*phi_1_2) - 2.40741243048404e-35*sin(2*phi_1_1 - 2*phi_1_2) + 4.81482486096809e-35*sin(2*phi_1_1 - phi_1_2) - 4.81482486096809e-35*sin(2*phi_1_1 + phi_1_2) - 7.22223729145213e-35*sin(2*phi_1_1 + 2*phi_1_2) - 3.25421956153017e-18*cos(phi_1_1) - 6.01853107621011e-35*cos(phi_1_1 - 2*phi_1_2) + 4.76569421051137e-19*cos(phi_1_1 - phi_1_2) - 5.07873085990693e-18*cos(phi_1_1 + phi_1_2) + 3.61111864572607e-35*cos(phi_1_1 + 2*phi_1_2)), 
                    k_f*std::pow(pwm_1, 2.0)*(-9.62964972193618e-35*sin(2*phi_1_2) + 0.00389148464179993*sin(phi_1_1 - phi_1_2) + 0.0414709846417999*sin(phi_1_1 + phi_1_2) + 1.73472347597681e-18*sin(phi_1_1 + 2*phi_1_2) - 2.40741243048404e-35*sin(2*phi_1_1 - phi_1_2) - 2.40741243048404e-35*sin(2*phi_1_1 + phi_1_2) - 9.62964972193618e-35*sin(2*phi_1_1 + 2*phi_1_2) + 4.81482486096809e-35*cos(phi_1_1) - 0.0134804982329429*cos(phi_1_2) - 1.73472347597681e-18*cos(2*phi_1_2) + 2.40741243048404e-35*cos(phi_1_1 - 2*phi_1_2) - 4.76569421051137e-19*cos(phi_1_1 - phi_1_2) - 5.07873085990693e-18*cos(phi_1_1 + phi_1_2) + 2.40741243048404e-35*cos(phi_1_1 + 2*phi_1_2) - 8.67361737988404e-19*cos(2*phi_1_1 - 2*phi_1_2) - 8.67361737988404e-19*cos(2*phi_1_1 + 2*phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.0265727192835999*sin(phi_2_1) - 7.22223729145213e-35*sin(2*phi_2_1) - 0.00389148464179993*sin(phi_2_1 - phi_2_2) + 0.0414709846417999*sin(phi_2_1 + phi_2_2) + 8.67361737988404e-19*sin(phi_2_1 + 2*phi_2_2) + 2.40741243048404e-35*sin(2*phi_2_1 - 2*phi_2_2) - 4.81482486096809e-35*sin(2*phi_2_1 - phi_2_2) + 4.81482486096809e-35*sin(2*phi_2_1 + phi_2_2) + 7.22223729145213e-35*sin(2*phi_2_1 + 2*phi_2_2) - 3.25421956153017e-18*cos(phi_2_1) + 4.76569421051137e-19*cos(phi_2_1 - phi_2_2) - 5.07873085990693e-18*cos(phi_2_1 + phi_2_2) + 4.81482486096809e-35*cos(phi_2_1 + 2*phi_2_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(-1.20370621524202e-35*sin(phi_2_2) + 9.62964972193618e-35*sin(2*phi_2_2) + 0.00389148464179993*sin(phi_2_1 - phi_2_2) + 0.0414709846417999*sin(phi_2_1 + phi_2_2) + 1.73472347597681e-18*sin(phi_2_1 + 2*phi_2_2) - 2.40741243048404e-35*sin(2*phi_2_1 - 2*phi_2_2) + 1.80555932286303e-35*sin(2*phi_2_1 - phi_2_2) + 3.00926553810506e-35*sin(2*phi_2_1 + phi_2_2) + 7.22223729145213e-35*sin(2*phi_2_1 + 2*phi_2_2) - 4.81482486096809e-35*cos(phi_2_1) + 0.0134804982329429*cos(phi_2_2) + 1.73472347597681e-18*cos(2*phi_2_2) + 7.22223729145213e-35*cos(phi_2_1 - 2*phi_2_2) - 4.76569421051137e-19*cos(phi_2_1 - phi_2_2) - 5.07873085990693e-18*cos(phi_2_1 + phi_2_2) + 7.22223729145213e-35*cos(phi_2_1 + 2*phi_2_2) + 8.67361737988404e-19*cos(2*phi_2_1 - 2*phi_2_2) + 8.67361737988404e-19*cos(2*phi_2_1 + 2*phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(-0.0460252998944602*sin(phi_3_1) + 0.00674024911647145*sin(phi_3_1 - phi_3_2) - 0.0718298524395061*sin(phi_3_1 + phi_3_2) - 1.73472347597681e-18*sin(phi_3_1 + 2*phi_3_2) - 2.40741243048404e-35*sin(2*phi_3_1 - 2*phi_3_2) + 2.40741243048404e-35*sin(2*phi_3_1 + 2*phi_3_2) - 3.25421956153017e-18*cos(phi_3_1) - 1.73472347597681e-18*cos(2*phi_3_1) + 2.40741243048404e-35*cos(phi_3_1 - 2*phi_3_2) + 4.76569421051137e-19*cos(phi_3_1 - phi_3_2) - 5.07873085990693e-18*cos(phi_3_1 + phi_3_2) - 7.22223729145213e-35*cos(phi_3_1 + 2*phi_3_2) + 1.73472347597681e-18*cos(2*phi_3_1 - 2*phi_3_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(-1.92592994438724e-34*sin(phi_3_2) - 0.00674024911647145*sin(phi_3_1 - phi_3_2) - 0.0718298524395061*sin(phi_3_1 + phi_3_2) - 3.46944695195361e-18*sin(phi_3_1 + 2*phi_3_2) + 2.40741243048404e-35*sin(2*phi_3_1 - 2*phi_3_2) + 2.40741243048404e-35*sin(2*phi_3_1 + 2*phi_3_2) - 2.40741243048404e-35*cos(phi_3_1) + 0.00778296928359986*cos(phi_3_2) + 8.67361737988404e-19*cos(2*phi_3_2) - 3.61111864572607e-35*cos(phi_3_1 - 2*phi_3_2) - 4.76569421051137e-19*cos(phi_3_1 - phi_3_2) - 5.07873085990693e-18*cos(phi_3_1 + phi_3_2) - 1.32407683676622e-34*cos(phi_3_1 + 2*phi_3_2) - 1.30104260698261e-18*cos(2*phi_3_1 - 2*phi_3_2) + 4.33680868994202e-19*cos(2*phi_3_1 + 2*phi_3_2));

        A.row(4) << k_f*pwm_1*(3.85185988877447e-34*sin(phi_1_1) - 0.0155659385671997*sin(phi_1_2) - 8.67361737988404e-19*sin(2*phi_1_2) - 9.62964972193618e-35*sin(phi_1_1 - 2*phi_1_2) - 1.92592994438724e-34*sin(phi_1_1 - phi_1_2) + 5.77778983316171e-34*sin(phi_1_1 + phi_1_2) + 9.62964972193618e-35*sin(phi_1_1 + 2*phi_1_2) + 4.33680868994202e-19*sin(2*phi_1_1 - 2*phi_1_2) - 4.33680868994202e-19*sin(2*phi_1_1 + 2*phi_1_2) + 0.0920505997889205*cos(phi_1_1) - 2.16667118743564e-34*cos(2*phi_1_1) - 9.62964972193618e-35*cos(phi_1_2) - 2.64815367353245e-34*cos(2*phi_1_2) - 0.0134804982329429*cos(phi_1_1 - phi_1_2) + 0.143659704879012*cos(phi_1_1 + phi_1_2) + 3.46944695195361e-18*cos(phi_1_1 + 2*phi_1_2) + 1.32407683676622e-34*cos(2*phi_1_1 - 2*phi_1_2) + 4.81482486096809e-35*cos(2*phi_1_1 - phi_1_2) + 4.81482486096809e-35*cos(2*phi_1_1 + phi_1_2) + 1.32407683676622e-34*cos(2*phi_1_1 + 2*phi_1_2) + 2.40741243048404e-35), 
                    k_f*pwm_2*(-3.85185988877447e-34*sin(phi_2_1) - 0.0155659385671997*sin(phi_2_2) - 8.67361737988404e-19*sin(2*phi_2_2) + 9.62964972193618e-35*sin(phi_2_1 - 2*phi_2_2) + 3.85185988877447e-34*sin(phi_2_1 - phi_2_2) - 3.85185988877447e-34*sin(phi_2_1 + phi_2_2) - 9.62964972193618e-35*sin(phi_2_1 + 2*phi_2_2) + 4.33680868994202e-19*sin(2*phi_2_1 - 2*phi_2_2) - 4.33680868994202e-19*sin(2*phi_2_1 + 2*phi_2_2) - 0.0920505997889205*cos(phi_2_1) - 2.16667118743564e-34*cos(2*phi_2_1) - 9.62964972193618e-35*cos(phi_2_2) - 2.64815367353245e-34*cos(2*phi_2_2) + 0.0134804982329429*cos(phi_2_1 - phi_2_2) - 0.143659704879012*cos(phi_2_1 + phi_2_2) - 3.46944695195361e-18*cos(phi_2_1 + 2*phi_2_2) + 1.32407683676622e-34*cos(2*phi_2_1 - 2*phi_2_2) + 4.81482486096809e-35*cos(2*phi_2_1 - phi_2_2) + 4.81482486096809e-35*cos(2*phi_2_1 + phi_2_2) + 1.32407683676622e-34*cos(2*phi_2_1 + 2*phi_2_2) + 2.40741243048404e-35), 
                    k_f*pwm_3*(0.0269609964658858*sin(phi_3_2) + 1.73472347597681e-18*sin(2*phi_3_2) + 9.62964972193618e-35*sin(phi_3_1 - 2*phi_3_2) - 3.85185988877447e-34*sin(phi_3_1 - phi_3_2) - 9.62964972193618e-35*sin(phi_3_1 + 2*phi_3_2) - 8.67361737988404e-19*sin(2*phi_3_1 - 2*phi_3_2) + 8.67361737988404e-19*sin(2*phi_3_1 + 2*phi_3_2) - 0.0531454385671997*cos(phi_3_1) - 2.40741243048404e-34*cos(2*phi_3_1) - 1.44444745829043e-34*cos(2*phi_3_2) + 0.00778296928359986*cos(phi_3_1 - phi_3_2) - 0.0829419692835998*cos(phi_3_1 + phi_3_2) + 1.68518870133883e-34*cos(2*phi_3_1 - 2*phi_3_2) + 3.85185988877447e-34*cos(2*phi_3_1 - phi_3_2) - 3.85185988877447e-34*cos(2*phi_3_1 + phi_3_2) + 1.68518870133883e-34*cos(2*phi_3_1 + 2*phi_3_2) + 2.40741243048404e-34), 
                    k_f*std::pow(pwm_1, 2.0)*(-0.0460252998944602*sin(phi_1_1) + 1.92592994438724e-34*sin(2*phi_1_1) + 0.00674024911647145*sin(phi_1_1 - phi_1_2) - 0.0718298524395061*sin(phi_1_1 + phi_1_2) - 1.73472347597681e-18*sin(phi_1_1 + 2*phi_1_2) - 9.62964972193618e-35*sin(2*phi_1_1 - 2*phi_1_2) - 9.62964972193618e-35*sin(2*phi_1_1 - phi_1_2) - 9.62964972193618e-35*sin(2*phi_1_1 + phi_1_2) - 9.62964972193618e-35*sin(2*phi_1_1 + 2*phi_1_2) + 1.92592994438724e-34*cos(phi_1_1) + 1.51091960431229e-36*cos(2*phi_1_1) - 2.49079348109414e-35*cos(phi_1_2) + 8.80628480508457e-36*cos(2*phi_1_2) - 2.40741243048404e-35*cos(phi_1_1 - 2*phi_1_2) + 9.62964972193618e-35*cos(phi_1_1 - phi_1_2) + 2.88889491658085e-34*cos(phi_1_1 + phi_1_2) + 2.40741243048404e-35*cos(phi_1_1 + 2*phi_1_2) + 8.67361737988404e-19*cos(2*phi_1_1 - 2*phi_1_2) - 1.24539674054707e-35*cos(2*phi_1_1 - phi_1_2) - 1.24539674054707e-35*cos(2*phi_1_1 + phi_1_2) - 8.67361737988404e-19*cos(2*phi_1_1 + 2*phi_1_2) - 3.3714219616026e-35), 
                    k_f*std::pow(pwm_1, 2.0)*(2.40741243048404e-35*sin(phi_1_2) + 1.92592994438724e-34*sin(2*phi_1_2) - 0.00674024911647145*sin(phi_1_1 - phi_1_2) - 0.0718298524395061*sin(phi_1_1 + phi_1_2) - 3.46944695195361e-18*sin(phi_1_1 + 2*phi_1_2) + 1.44444745829043e-34*sin(2*phi_1_1 - 2*phi_1_2) + 1.20370621524202e-35*sin(2*phi_1_1 - phi_1_2) - 1.20370621524202e-35*sin(2*phi_1_1 + phi_1_2) - 1.44444745829043e-34*sin(2*phi_1_1 + 2*phi_1_2) - 0.00778296928359986*cos(phi_1_2) - 8.67361737988404e-19*cos(2*phi_1_2) + 4.81482486096809e-35*cos(phi_1_1 - 2*phi_1_2) + 1.92592994438724e-34*cos(phi_1_1 + phi_1_2) + 4.81482486096809e-35*cos(phi_1_1 + 2*phi_1_2) - 4.33680868994202e-19*cos(2*phi_1_1 - 2*phi_1_2) - 4.33680868994202e-19*cos(2*phi_1_1 + 2*phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(0.0460252998944602*sin(phi_2_1) + 1.92592994438724e-34*sin(2*phi_2_1) - 0.00674024911647145*sin(phi_2_1 - phi_2_2) + 0.0718298524395061*sin(phi_2_1 + phi_2_2) + 1.73472347597681e-18*sin(phi_2_1 + 2*phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 - 2*phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 - phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 + phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 + 2*phi_2_2) - 1.92592994438724e-34*cos(phi_2_1) + 1.51091960431229e-36*cos(2*phi_2_1) - 2.49079348109414e-35*cos(phi_2_2) + 8.80628480508457e-36*cos(2*phi_2_2) + 2.40741243048404e-35*cos(phi_2_1 - 2*phi_2_2) - 9.62964972193618e-35*cos(phi_2_1 - phi_2_2) - 2.88889491658085e-34*cos(phi_2_1 + phi_2_2) - 2.40741243048404e-35*cos(phi_2_1 + 2*phi_2_2) + 8.67361737988404e-19*cos(2*phi_2_1 - 2*phi_2_2) - 1.24539674054707e-35*cos(2*phi_2_1 - phi_2_2) - 1.24539674054707e-35*cos(2*phi_2_1 + phi_2_2) - 8.67361737988404e-19*cos(2*phi_2_1 + 2*phi_2_2) - 3.3714219616026e-35), 
                    k_f*std::pow(pwm_2, 2.0)*(2.40741243048404e-35*sin(phi_2_2) + 1.92592994438724e-34*sin(2*phi_2_2) + 0.00674024911647145*sin(phi_2_1 - phi_2_2) + 0.0718298524395061*sin(phi_2_1 + phi_2_2) + 3.46944695195361e-18*sin(phi_2_1 + 2*phi_2_2) + 1.44444745829043e-34*sin(2*phi_2_1 - 2*phi_2_2) + 1.20370621524202e-35*sin(2*phi_2_1 - phi_2_2) - 1.20370621524202e-35*sin(2*phi_2_1 + phi_2_2) - 1.44444745829043e-34*sin(2*phi_2_1 + 2*phi_2_2) - 0.00778296928359986*cos(phi_2_2) - 8.67361737988404e-19*cos(2*phi_2_2) - 4.81482486096809e-35*cos(phi_2_1 - 2*phi_2_2) - 1.92592994438724e-34*cos(phi_2_1 + phi_2_2) - 4.81482486096809e-35*cos(phi_2_1 + 2*phi_2_2) - 4.33680868994202e-19*cos(2*phi_2_1 - 2*phi_2_2) - 4.33680868994202e-19*cos(2*phi_2_1 + 2*phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(0.0265727192835999*sin(phi_3_1) + 3.85185988877447e-34*sin(2*phi_3_1) - 3.64768260038614e-36*sin(phi_3_1 - 2*phi_3_2) - 0.00389148464179993*sin(phi_3_1 - phi_3_2) + 0.0414709846417999*sin(phi_3_1 + phi_3_2) + 2.12602522105553e-35*sin(phi_3_1 + 2*phi_3_2) - 1.92592994438724e-34*sin(2*phi_3_1 - 2*phi_3_2) - 1.92592994438724e-34*sin(2*phi_3_1 - phi_3_2) - 1.92592994438724e-34*sin(2*phi_3_1 + phi_3_2) - 1.92592994438724e-34*sin(2*phi_3_1 + 2*phi_3_2) - 2.61698952082074e-36*cos(2*phi_3_1) - 4.31418086041641e-35*cos(phi_3_2) + 1.52529327083283e-35*cos(2*phi_3_2) + 2.40741243048404e-35*cos(phi_3_1 - 2*phi_3_2) - 1.92592994438724e-34*cos(phi_3_1 - phi_3_2) - 2.40741243048404e-35*cos(phi_3_1 + 2*phi_3_2) + 2.28793990624924e-35*cos(2*phi_3_1 - 2*phi_3_2) + 2.1570904302082e-35*cos(2*phi_3_1 - phi_3_2) + 2.1570904302082e-35*cos(2*phi_3_1 + phi_3_2) + 2.28793990624924e-35*cos(2*phi_3_1 + 2*phi_3_2) - 5.83947413124924e-35), 
                    k_f*std::pow(pwm_3, 2.0)*(2.49079348109414e-35*sin(phi_3_1) + 4.81482486096809e-35*sin(phi_3_2) + 1.92592994438724e-34*sin(2*phi_3_2) + 1.24539674054707e-35*sin(phi_3_1 - 2*phi_3_2) + 0.00389148464179993*sin(phi_3_1 - phi_3_2) + 0.0414709846417999*sin(phi_3_1 + phi_3_2) + 1.24539674054707e-35*sin(phi_3_1 + 2*phi_3_2) + 9.62964972193618e-35*sin(2*phi_3_1 - 2*phi_3_2) + 2.40741243048404e-35*sin(2*phi_3_1 - phi_3_2) - 2.40741243048404e-35*sin(2*phi_3_1 + phi_3_2) - 9.62964972193618e-35*sin(2*phi_3_1 + 2*phi_3_2) + 0.0134804982329429*cos(phi_3_2) + 1.73472347597681e-18*cos(2*phi_3_2) - 4.81482486096809e-35*cos(phi_3_1 - 2*phi_3_2) + 9.62964972193618e-35*cos(phi_3_1 - phi_3_2) + 9.62964972193618e-35*cos(phi_3_1 + phi_3_2) - 4.81482486096809e-35*cos(phi_3_1 + 2*phi_3_2) + 8.67361737988404e-19*cos(2*phi_3_1 - 2*phi_3_2) + 8.67361737988404e-19*cos(2*phi_3_1 + 2*phi_3_2));

        A.row(5) << k_f*pwm_1*(-0.106290877134399*sin(phi_1_1) + 9.62964972193618e-35*sin(2*phi_1_1) + 1.65088490118851e-18*sin(phi_1_2) + 9.62964972193618e-35*sin(2*phi_1_2) - 8.67361737988404e-19*sin(phi_1_1 - 2*phi_1_2) + 0.0155659385671997*sin(phi_1_1 - phi_1_2) - 0.1658839385672*sin(phi_1_1 + phi_1_2) - 8.67361737988404e-19*sin(phi_1_1 + 2*phi_1_2) + 9.62964972193618e-35*sin(2*phi_1_1 - phi_1_2) + 9.62964972193618e-35*sin(2*phi_1_1 + phi_1_2) + 9.62964972193618e-35*sin(2*phi_1_1 + 2*phi_1_2) + 3.25421956153017e-18*cos(phi_1_1) + 4.33680868994202e-19*cos(2*phi_1_1) + 4.81482486096809e-35*cos(phi_1_1 - 2*phi_1_2) - 4.76569421051136e-19*cos(phi_1_1 - phi_1_2) + 5.07873085990693e-18*cos(phi_1_1 + phi_1_2) + 4.81482486096809e-35*cos(phi_1_1 + 2*phi_1_2) - 4.33680868994202e-19), 
                    k_f*pwm_2*(-0.106290877134399*sin(phi_2_1) - 9.62964972193618e-35*sin(2*phi_2_1) - 1.65088490118851e-18*sin(phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_2) - 8.67361737988404e-19*sin(phi_2_1 - 2*phi_2_2) + 0.0155659385671997*sin(phi_2_1 - phi_2_2) - 0.1658839385672*sin(phi_2_1 + phi_2_2) - 8.67361737988404e-19*sin(phi_2_1 + 2*phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 - phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 + phi_2_2) - 9.62964972193618e-35*sin(2*phi_2_1 + 2*phi_2_2) + 3.25421956153017e-18*cos(phi_2_1) - 4.33680868994202e-19*cos(2*phi_2_1) + 4.81482486096809e-35*cos(phi_2_1 - 2*phi_2_2) - 4.76569421051136e-19*cos(phi_2_1 - phi_2_2) + 5.07873085990693e-18*cos(phi_2_1 + phi_2_2) + 4.81482486096809e-35*cos(phi_2_1 + 2*phi_2_2) + 4.33680868994202e-19), 
                    2*k_f*pwm_3*(-0.0531454385671997*sin(phi_3_1) - 7.22223729145213e-35*sin(2*phi_3_1) - 4.76569421051137e-19*sin(phi_3_2) - 7.22223729145213e-35*sin(2*phi_3_2) - 4.33680868994202e-19*sin(phi_3_1 - 2*phi_3_2) + 0.00778296928359987*sin(phi_3_1 - phi_3_2) - 0.0829419692835999*sin(phi_3_1 + phi_3_2) - 4.33680868994202e-19*sin(phi_3_1 + 2*phi_3_2) + 1.20370621524202e-35*sin(2*phi_3_1 - 2*phi_3_2) + 3.61111864572607e-35*sin(2*phi_3_1 + 2*phi_3_2) - 2.81823680977739e-18*cos(phi_3_1) + 4.12721225297128e-19*cos(phi_3_1 - phi_3_2) - 4.39830994366339e-18*cos(phi_3_1 + phi_3_2) - 9.62964972193618e-35*cos(phi_3_1 + 2*phi_3_2) + 4.33680868994202e-19*cos(2*phi_3_1 - 2*phi_3_2) - 4.33680868994202e-19*cos(2*phi_3_1 + 2*phi_3_2) + 3.46944695195361e-18), 
                    k_f*std::pow(pwm_1, 2.0)*(-1.62710978076509e-18*sin(phi_1_1) - 4.33680868994202e-19*sin(2*phi_1_1) - 4.33680868994202e-19*sin(phi_1_2) - 2.40741243048404e-35*sin(phi_1_1 - 2*phi_1_2) + 2.38284710525568e-19*sin(phi_1_1 - phi_1_2) - 2.53936542995347e-18*sin(phi_1_1 + phi_1_2) - 2.40741243048404e-35*sin(phi_1_1 + 2*phi_1_2) + 6.50521303491303e-19*sin(2*phi_1_1 - phi_1_2) - 6.50521303491303e-19*sin(2*phi_1_1 + phi_1_2) - 0.0531454385671997*cos(phi_1_1) + 9.62964972193618e-35*cos(2*phi_1_1) - 4.33680868994202e-19*cos(phi_1_1 - 2*phi_1_2) + 0.00778296928359986*cos(phi_1_1 - phi_1_2) - 0.0829419692835998*cos(phi_1_1 + phi_1_2) - 4.33680868994202e-19*cos(phi_1_1 + 2*phi_1_2) + 4.81482486096809e-35*cos(2*phi_1_1 - 2*phi_1_2) + 9.62964972193618e-35*cos(2*phi_1_1 - phi_1_2) + 9.62964972193618e-35*cos(2*phi_1_1 + phi_1_2) + 4.81482486096809e-35*cos(2*phi_1_1 + 2*phi_1_2)), 
                    k_f*std::pow(pwm_1, 2.0)*(2.16840434497101e-19*sin(phi_1_2) + 4.81482486096809e-35*sin(phi_1_1 - 2*phi_1_2) - 2.38284710525568e-19*sin(phi_1_1 - phi_1_2) - 2.53936542995347e-18*sin(phi_1_1 + phi_1_2) - 4.81482486096809e-35*sin(phi_1_1 + 2*phi_1_2) - 1.0842021724855e-19*sin(2*phi_1_1 - phi_1_2) - 3.25260651745651e-19*sin(2*phi_1_1 + phi_1_2) + 2.60208521396521e-18*cos(phi_1_1) + 8.25442450594254e-19*cos(phi_1_2) + 9.62964972193618e-35*cos(2*phi_1_2) + 4.33680868994202e-19*cos(phi_1_1 - 2*phi_1_2) - 0.00778296928359986*cos(phi_1_1 - phi_1_2) - 0.0829419692835998*cos(phi_1_1 + phi_1_2) - 1.30104260698261e-18*cos(phi_1_1 + 2*phi_1_2) - 4.81482486096809e-35*cos(2*phi_1_1 - 2*phi_1_2) - 4.81482486096809e-35*cos(2*phi_1_1 - phi_1_2) + 4.81482486096809e-35*cos(2*phi_1_1 + phi_1_2) + 1.44444745829043e-34*cos(2*phi_1_1 + 2*phi_1_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(-1.62710978076509e-18*sin(phi_2_1) + 4.33680868994202e-19*sin(2*phi_2_1) - 2.40741243048404e-35*sin(phi_2_1 - 2*phi_2_2) + 2.38284710525568e-19*sin(phi_2_1 - phi_2_2) - 2.53936542995347e-18*sin(phi_2_1 + phi_2_2) - 2.40741243048404e-35*sin(phi_2_1 + 2*phi_2_2) - 0.0531454385671997*cos(phi_2_1) - 9.62964972193618e-35*cos(2*phi_2_1) - 6.50521303491303e-19*cos(phi_2_1 - 2*phi_2_2) + 0.00778296928359985*cos(phi_2_1 - phi_2_2) - 0.0829419692835999*cos(phi_2_1 + phi_2_2) - 6.50521303491303e-19*cos(phi_2_1 + 2*phi_2_2) - 4.81482486096809e-35*cos(2*phi_2_1 - 2*phi_2_2) - 9.62964972193618e-35*cos(2*phi_2_1 - phi_2_2) - 9.62964972193618e-35*cos(2*phi_2_1 + phi_2_2) - 4.81482486096809e-35*cos(2*phi_2_1 + 2*phi_2_2)), 
                    k_f*std::pow(pwm_2, 2.0)*(4.81482486096809e-35*sin(phi_2_1 - 2*phi_2_2) - 2.38284710525568e-19*sin(phi_2_1 - phi_2_2) - 2.53936542995347e-18*sin(phi_2_1 + phi_2_2) - 4.81482486096809e-35*sin(phi_2_1 + 2*phi_2_2) - 8.25442450594254e-19*cos(phi_2_2) - 9.62964972193618e-35*cos(2*phi_2_2) + 8.67361737988404e-19*cos(phi_2_1 - 2*phi_2_2) - 0.00778296928359985*cos(phi_2_1 - phi_2_2) - 0.0829419692835999*cos(phi_2_1 + phi_2_2) - 8.67361737988404e-19*cos(phi_2_1 + 2*phi_2_2) + 4.81482486096809e-35*cos(2*phi_2_1 - 2*phi_2_2) + 4.81482486096809e-35*cos(2*phi_2_1 - phi_2_2) - 4.81482486096809e-35*cos(2*phi_2_1 + phi_2_2) - 1.44444745829043e-34*cos(2*phi_2_1 + 2*phi_2_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(2.81823680977739e-18*sin(phi_3_1) - 4.12721225297128e-19*sin(phi_3_1 - phi_3_2) + 4.39830994366339e-18*sin(phi_3_1 + phi_3_2) + 9.62964972193618e-35*sin(phi_3_1 + 2*phi_3_2) - 4.33680868994202e-19*sin(2*phi_3_1 - 2*phi_3_2) + 4.33680868994202e-19*sin(2*phi_3_1 + 2*phi_3_2) - 0.0531454385671997*cos(phi_3_1) - 1.44444745829043e-34*cos(2*phi_3_1) - 8.67361737988404e-19*cos(phi_3_1 - 2*phi_3_2) + 0.00778296928359987*cos(phi_3_1 - phi_3_2) - 0.0829419692835999*cos(phi_3_1 + phi_3_2) + 8.67361737988404e-19*cos(phi_3_1 + 2*phi_3_2) + 4.81482486096809e-35*cos(2*phi_3_1 - 2*phi_3_2) + 4.81482486096809e-35*cos(2*phi_3_1 + 2*phi_3_2)), 
                    k_f*std::pow(pwm_3, 2.0)*(4.12721225297128e-19*sin(phi_3_1 - phi_3_2) + 4.39830994366339e-18*sin(phi_3_1 + phi_3_2) + 1.92592994438724e-34*sin(phi_3_1 + 2*phi_3_2) + 8.67361737988404e-19*sin(2*phi_3_1 - 2*phi_3_2) - 4.33680868994202e-19*sin(2*phi_3_1 - phi_3_2) - 4.33680868994202e-19*sin(2*phi_3_1 + phi_3_2) + 8.67361737988404e-19*sin(2*phi_3_1 + 2*phi_3_2) - 4.76569421051137e-19*cos(phi_3_2) - 1.44444745829043e-34*cos(2*phi_3_2) + 8.67361737988404e-19*cos(phi_3_1 - 2*phi_3_2) - 0.00778296928359987*cos(phi_3_1 - phi_3_2) - 0.0829419692835999*cos(phi_3_1 + phi_3_2) - 8.67361737988404e-19*cos(phi_3_1 + 2*phi_3_2) - 2.40741243048404e-35*cos(2*phi_3_1 - 2*phi_3_2) + 7.22223729145213e-35*cos(2*phi_3_1 + 2*phi_3_2));

    
    }


    void calculate_u_dot(){
        //u_dot = (A.transpose() * (A*(A.transpose())).inverse()) * wrench_dot;

        //with weighted pseudo inverse
        Eigen::Matrix <double, 9, 9> W = pseudo_inverse_weights.asDiagonal();
        Eigen::MatrixXd D =Eigen::MatrixXd::Identity(6,6)*damping_coefficient;
        

        if(first_call){
            std::cout << "D: " << std::endl << D << std::endl;
            std::cout << "W: " << std::endl << W << std::endl;
        }
        
        u_dot = (W*A.transpose() * (A*W*A.transpose() + D).inverse()) * wrench_dot;
       
       
    }

    void integrate_u_dot(){

        //--------------------------------------------------------------------------------???
        // is u = integral from start to now of u_dot right ?
        //--------------------------------------------------------------------------------???
        


        time_now_i = std::chrono::high_resolution_clock::now();

        if(first_call){
            last_call_i = time_now_i;
            u.setZero();
            wrench_int = wrench_now;
            std::cout<<"wrench integral initialized at " << std::endl << wrench_int.transpose() << std::endl;
            return;
        }
        double delta_t_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now_i - last_call_i).count();
        double delta_t_seconds = static_cast<double>(delta_t_nanoseconds) / (10e9);

        // u = u + delta_t_seconds * u_dot;
        u = u + 0.01 * u_dot;

        wrench_int = wrench_int + 0.01 * wrench_dot;
        // std::cout<<"wrench now at " << std::endl << wrench_now.transpose() << std::endl;
        // std::cout<<"wrench integral at " << std::endl << wrench_int.transpose() << std::endl;
        last_call_i = time_now_i;
    }

    void transform_u_to_setpoints(){
        Eigen::Vector3d pwm_copy;
        Vector6d nearest_rot_copy;

        if(for_sim){
            pwm_copy = pwm_ranger(u.head(3), pwm_min, pwm_max);
        }else{
            pwm_copy = pwm_ranger(u.head(3), pwm_min, pwm_max);
        }
     
        
        nearest_rot_copy = vec_get_nearest_rotation(u.tail(6), phi)/uebersetzung;

        desired_setpoints[0] = pwm_copy(0);
        desired_setpoints[1] = pwm_copy(1);
        desired_setpoints[2] = pwm_copy(2);
        desired_setpoints[3] = nearest_rot_copy(0);
        desired_setpoints[4] = nearest_rot_copy(1);
        desired_setpoints[5] = nearest_rot_copy(2);
        desired_setpoints[6] = nearest_rot_copy(3);
        desired_setpoints[7] = nearest_rot_copy(4);
        desired_setpoints[8] = nearest_rot_copy(5);

        //nicht so sicher ob die pwm aus dem omav_status wirklich stimmen
        //nur um den nächsten status der propeller auch stimmen
        //pwm = pwm_copy;
        //phi = nearest_rot_copy;

        
    }

    void print_debug(){

        std::cout << "motor status:" << std::endl << pwm.transpose() << " " << phi.transpose() << std::endl;
        
        std::cout << "desired_setpoints: " << desired_setpoints[0] << " " << desired_setpoints[1] << " " << desired_setpoints[2] << " " << desired_setpoints[3] << " " << desired_setpoints[4] << " " << desired_setpoints[5] << " " << desired_setpoints[6] << " " << desired_setpoints[7] << " " << desired_setpoints[8] << std::endl;
        avero_msgs::Vector9d setpoints_msg;
        setpoints_msg.x1 = desired_setpoints[0];
        setpoints_msg.x2 = desired_setpoints[1];
        setpoints_msg.x3 = desired_setpoints[2];
        setpoints_msg.x4 = desired_setpoints[3];
        setpoints_msg.x5 = desired_setpoints[4];
        setpoints_msg.x6 = desired_setpoints[5];
        setpoints_msg.x7 = desired_setpoints[6];
        setpoints_msg.x8 = desired_setpoints[7];
        setpoints_msg.x9 = desired_setpoints[8];
        setpoints_pub.publish(setpoints_msg);
        
        
        std::cout << "A: " << std::endl << A << std::endl;

        std::cout << "A^+: " << std::endl << (A.transpose() * (A*(A.transpose())).inverse()) << std::endl;
        std::cout << "wrench: " << std::endl << wrench_now.transpose() << std::endl;

        std::cout << "wrench_dot: " << std::endl << wrench_dot.transpose() << std::endl;
        avero_msgs::Vector6d wrench_dot_msg;
        wrench_dot_msg.x1 = wrench_dot(0);
        wrench_dot_msg.x2 = wrench_dot(1);
        wrench_dot_msg.x3 = wrench_dot(2);
        wrench_dot_msg.x4 = wrench_dot(3);
        wrench_dot_msg.x5 = wrench_dot(4);
        wrench_dot_msg.x6 = wrench_dot(5);
        wrench_dot_pub.publish(wrench_dot_msg);
        
        

        std::cout << "u_dot: " << std::endl << u_dot.transpose() << std::endl;
        avero_msgs::Vector9d u_dot_msg;
        u_dot_msg.x1 = u_dot(0);
        u_dot_msg.x2 = u_dot(1);
        u_dot_msg.x3 = u_dot(2);
        u_dot_msg.x4 = u_dot(3);
        u_dot_msg.x5 = u_dot(4);
        u_dot_msg.x6 = u_dot(5);
        u_dot_msg.x7 = u_dot(6);
        u_dot_msg.x8 = u_dot(7);
        u_dot_msg.x9 = u_dot(8);
        u_dot_pub.publish(u_dot_msg);

        // std::cout << "wrench: " << std::endl << wrench_now.transpose() << std::endl;
        avero_msgs::Vector9d wrench_int_msg;
        wrench_int_msg.x1 = wrench_int(0);
        wrench_int_msg.x2 = wrench_int(1);
        wrench_int_msg.x3 = wrench_int(2);
        wrench_int_msg.x4 = wrench_int(3);
        wrench_int_msg.x5 = wrench_int(4);
        wrench_int_msg.x6 = wrench_int(5);
        wrench_int_pub.publish(wrench_int_msg);

        if(first_call){
            time_last_call_total = std::chrono::high_resolution_clock::now();
        }else{
            std::chrono::time_point<std::chrono::high_resolution_clock> time_now_total = std::chrono::high_resolution_clock::now();
            double delta_t_nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(time_now_total - time_last_call_total).count();
            
            std::cout << "time since last call in nano seconds: " << delta_t_nanoseconds << std::endl;
            time_last_call_total = time_now_total;
        }
        

    }
    


    std::array<double,9>  allocate(const Vector6d& wrench_now, const Vector9d& omav_state){

        update_omav_state(omav_state);
        update_wrench(wrench_now);
        calculate_wrench_dot();
        update_A();
        calculate_u_dot();
        integrate_u_dot();
        transform_u_to_setpoints();

        print_debug();



        if(first_call){
            first_call = false;
        }

        count += 1;

        // if(count == 5){
        //     ROS_ERROR("run 10 times");
        //     ros::shutdown();
        // }

        return desired_setpoints;
    }

private:
    int count;
    int average_length;
    double damping_coefficient;
    
    Vector6d wrench_before;
    Eigen::MatrixXd last_wrenches;
    Vector6d wrench_int;
    Vector6d wrench_dot;
    Vector9d pseudo_inverse_weights;
    Matrix69d A; //this Matrix still needs to be inverted
    double k_f; //Motor constant
    std::chrono::time_point<std::chrono::high_resolution_clock> last_call_d;
    std::chrono::time_point<std::chrono::high_resolution_clock> last_call_i;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_now_d;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_now_i;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_last_call_total;
    Vector9d u_dot;
    bool first_call;
    double uebersetzung;

    double pwm_min = 50;
    double pwm_max = 900;
    bool for_sim;

    //for debugging
    ros::NodeHandle nh;
    ros::Publisher wrench_dot_pub;
    ros::Publisher u_dot_pub;
    ros::Publisher setpoints_pub;
    ros::Publisher wrench_int_pub;


};




#!/usr/bin/env python3

#######################TO DO#######################
#change maximum wrench scaling MOSTLY DONE? NEED TO VERIFY
#plot determinant of JJ^T DONE
# M instead of weight matrix (optional)
######################################################

import numpy as np     
from allocation_functions_cartesian import derive, evaluate_allocation_matrix, compute_matrix_inverse, normalize_angle, allocation_matrix, compute_forces, compute_torques
import rospy
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float64
import time

from dynamixel_sdk_examples.msg import SetPosition
from mav_msgs.msg import Actuators
from allocation_matrix_import import get_allocation_matrix

####messaage type: Actuators
#float64[] angluar_velocities
#float64[] angles
#??? doesnt matter 
#all matrices are 1x9



class allocator: # takes an array which has the size of the number of dynamixels and  sends it as individual messages to the read_write_node.py
    def __init__(self, rate):
        pub_topic_name = "/command/motor_speed"  
        sub_topic_name = "des_wrench" #topic comming from the deg_to_clicks
        pub_force_topic_name = "current_force"
        pub_torque_topic_name = "current_torque"
        pub_rank_name = "det_A_eq_0"
       
        self.prev_wrench_cmd = None 
        self.wrench_cmd = np.zeros((12,1)) 
        self.initilized = False
        self.pub_to_actuators = rospy.Publisher(pub_topic_name, Actuators, queue_size=1) 
        self.pub_current_force = rospy.Publisher(pub_force_topic_name, Float64MultiArray, queue_size=1)
        self.pub_current_torque = rospy.Publisher(pub_torque_topic_name, Float64MultiArray, queue_size=1)
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Float64MultiArray, self.callback)
        self.set_actuators_msg = Actuators()

        self.pub_rank = rospy.Publisher(pub_rank_name, Float64, queue_size=1)
        self.pub_u_dot = rospy.Publisher("u_dot", Actuators, queue_size=1)
        self.u_dot_msg = Actuators()
        self.pub_actuator_limit = rospy.Publisher("actuator_limit", Float64, queue_size=1)
        self.pub_force_error = rospy.Publisher("force_error", Float64, queue_size=1)
        self.pub_torque_error = rospy.Publisher("torque_error", Float64, queue_size=1)
        #set publishing rate
        self.rate = rate
        

        #platform specific values just for testing
        self.min_rpm = 50
        self.idle_rpm = 50
        self.max_rpm = 900
        self.optimal_omega = 600
        self.optimal_omega_gain = 1
        self.max_phi_dot = 2
        self.omega = np.array([[self.idle_rpm], [self.idle_rpm], [self.idle_rpm]])
        self.omega_dot = np.zeros((3,1))
        self.phi = np.zeros((6,1))
        self.phi_dot = np.zeros((6,1))
     

        self.k_f = 6e-5

        self.dt = 1/self.rate
        #vectors from the center of the platform to the center of the nozzle outlet
        self.x_1_val = 0.187150672078
        self.y_1_val = -0.321475774049
        self.z_1_val = 0.0
        self.x_2_val = -0.188550204038
        self.y_2_val = 0.330919086933
        self.z_2_val = 0.0
        self.x_3_val = -0.373928427696
        self.y_3_val = 0.0
        self.z_3_val = 0.0

        self.W = np.diag([750, 750, 750, 160, 160, 160, 160, 160, 160])
        self.D = np.diag([0.007, 0.007, 0.007, 0.007, 0.007, 0.007])



        self.current_state = np.vstack((self.omega, self.phi))
        # self.current_state_dot = np.vstack((self.omega_dot, self.phi_dot))

        self.w_dot = np.zeros((6,1))

        self.wrench_feedback = None 

        self.start_time = time.time()



        self.rate = rospy.Rate(rate)


    #initialize the first 
    def initilize(self, msg): 
        if self.prev_wrench_cmd is None: 
            #initialize the previous wrench command
            self.prev_wrench_cmd = np.zeros((6,1))

            #evaluate the allocation matrix for the initial state
            A_num = get_allocation_matrix(self.k_f,self.current_state[3], self.current_state[4], self.current_state[5], self.current_state[6], self.current_state[7], self.current_state[8], self.current_state[0], self.current_state[1], self.current_state[2])
            

            #initialize the current/desired wrench command
            self.wrench_cmd = np.array(msg.data).reshape((-1, 1))


           # initialize allocation command wrench
            self.wrench_feedback = np.vstack((compute_forces(self.current_state, self.k_f), compute_torques(self.current_state, self.k_f)))


            #calculate the derivative of the wrench command 
            self.w_dot = derive(self.wrench_cmd, self.wrench_feedback, self.dt, 1)


            #calculate the inverse of the allocation matrix = A.T*(A*A.T)^-1
            A_inv_num = (self.W @ np.transpose(A_num) @ np.linalg.inv(A_num @ self.W @ np.transpose(A_num) + self.D))
            self.current_state_dot = A_inv_num@self.w_dot 

            # set initial state
            init_omega = np.array([[400.0], [400.0], [400.0]])
            init_phi = np.array([[0.0], [0.0], [0.0], [0.0], [0.0], [0.0]])
            init_state = np.vstack([(init_omega), (init_phi)])

            # set initial state
            self.current_state = np.vstack((np.array([[400], [400], [400]]), np.zeros((6,1))))


            print("current_state: Initialized \n", self.current_state)



            self.prev_wrench_cmd = self.wrench_cmd
            self.initilized = True

            
            print("initial publish")

            self.publish_to_actuators()
            

 


    
    def callback(self, msg):

        #make sure that the first wrench command is initialized
        if self.initilized == False: 
            self.initilize(msg)
            

        else:
            print("inside callback")

            #update the command wrench
            self.wrench_cmd = np.array(msg.data).reshape((-1, 1))


            #evaluate the current wrench
            self.wrench_feedback = np.vstack((compute_forces(self.current_state, self.k_f), compute_torques(self.current_state, self.k_f)))

            force_error_msg = Float64()
            torque_error_msg = Float64()
            force_error_msg.data = np.linalg.norm(self.wrench_cmd[0:3] - self.wrench_feedback[0:3])
            torque_error_msg.data = np.linalg.norm(self.wrench_cmd[3:6] - self.wrench_feedback[3:6])
            self.pub_force_error.publish(force_error_msg)
            self.pub_torque_error.publish(torque_error_msg)
            #differentiate to obtain command jerk
            self.w_dot = derive(self.wrench_cmd, self.wrench_feedback, self.dt, 0.1)
            
            
            #evaluate the allocation matrix for the current state
            A_num = get_allocation_matrix(self.k_f,self.current_state[3], self.current_state[4], self.current_state[5], self.current_state[6], self.current_state[7], self.current_state[8], self.current_state[0], self.current_state[1], self.current_state[2])
            
            
            #check the determinant of the allocation matrix
            A_t = np.transpose(A_num)
            rank_msg = Float64()
            # if  np.linalg.det(A_num @ A_t) < 1e-4:   
            #     rank_msg.data = 1
            # else:
            #     rank_msg.data = 0
            rank_msg.data = np.linalg.det(A_num @ A_t)
            self.pub_rank.publish(rank_msg)
     
            
            # calculate the weighted inverse of the allocation matrix = W * A.T * (A * W * A.T + lamda * I)^-1
            A_inv_num = (self.W @ np.transpose(A_num) @ np.linalg.inv(A_num @ self.W @ np.transpose(A_num) + self.D))

            # compute optimal u_dot to maintain secondary objective optimal_omega
            omega_dot_optimal = np.array(self.optimal_omega_gain * (self.optimal_omega - self.current_state[0:3])).reshape((-1, 1))
            u_dot_optimal = np.vstack((omega_dot_optimal, np.zeros((6, 1))))


            # compute u_dot with nullspace allocation u_dot = u_dot_optimal + A^-1 * (w_dot - A * u_dot_optimal)
            self.current_state_dot = u_dot_optimal + A_inv_num @ (self.w_dot - A_num @ u_dot_optimal)
            print("current_state_dot: ", self.current_state_dot)

            # set up msg to check if servos speeds are saturated
            actuator_limit_msg = Float64()
            actuator_limit_msg.data = 0  
            # scale down u_dot according to maximum possible phi_dot
            phi_dot_overspeed = 1

            for i in range(3, 9):
                if abs(self.current_state_dot[i]) / self.max_phi_dot > phi_dot_overspeed:
                    phi_dot_overspeed = abs(self.current_state_dot[i]) / self.max_phi_dot
                    print("overspeed at servo {}: ".format(i), phi_dot_overspeed)
            if(phi_dot_overspeed > 1):
                self.current_state_dot = self.current_state_dot / phi_dot_overspeed
                actuator_limit_msg.data = phi_dot_overspeed
                print("current_state_dot after scaling: ", self.current_state_dot)
            
            self.pub_actuator_limit.publish(actuator_limit_msg)

            # CAN PROBABLY REMOVE THIS
            # check servo velocities
            # for i in range(3, 9):
            #     if abs(self.current_state_dot[i]) >= 0.9*self.max_phi_dot:
            #         print("Servo {} velocity too high: {}".format(i, self.current_state_dot[i]))
            # print("current_state_dot: ", self.current_state_dot)
                    
            # checking the actuator limits work
            self.u_dot_msg.angular_velocities = self.current_state_dot[0:3]
            self.u_dot_msg.angles = self.current_state_dot[3:9]
            self.pub_u_dot.publish(self.u_dot_msg)


            #integrate the current_state_dot to get the current state
            self.current_state += self.current_state_dot*self.dt


            # DO WE NEED UPSCALING AS WELL FOR MINIMUM RPM? WHAT IF THE TWO CONFLICT
            # scale down u according to maximum possible rpm
            rpm_overspeed = 1
            for i in range(0, 3):
                if abs(self.current_state[i]) / self.max_rpm > rpm_overspeed:
                    rpm_overspeed = abs(self.current_state[i]) / self.max_rpm
                    print("overspeed at EDF {}: ".format(i), rpm_overspeed)
            if(rpm_overspeed > 1):
                self.current_state[0:3] = self.current_state[0:3] / rpm_overspeed
                print("current_state after scaling: ", self.current_state)
            
            # NOT SURE IF THIS IS NEEDED STILL ACTUALLY I THINK IT IS
            # Enforce RPM minimums
            self.current_state[0] = max(self.current_state[0], self.min_rpm)
            self.current_state[1] = max(self.current_state[1], self.min_rpm)
            self.current_state[2] = max(self.current_state[2], self.min_rpm)

            print("current_state: ", self.current_state)
            print("start time =", self.start_time)

            #publish the current force and torque
            force_msg = Float64MultiArray() 
            force_msg.data = compute_forces(self.current_state, self.k_f) 
            self.pub_current_force.publish(force_msg)
            torque_msg = Float64MultiArray() 
            torque_msg.data = compute_torques(self.current_state, self.k_f)
            self.pub_current_torque.publish(torque_msg)

            
            #update the previous wrench command            
            self.prev_wrench_cmd = self.wrench_cmd


            #publish the current state to the dynamixels and motors
            self.publish_to_actuators()





    def publish_to_actuators(self):
        
       
        print("publishes...")
        # self.set_actuators_msg.angular_velocities = np.array([0,0,0,0,0,0,0,0,0])
        self.set_actuators_msg.angular_velocities = np.array([self.current_state[0], self.current_state[1], self.current_state[2], 0 ,0 , 0, 0, 0, 0])
        self.set_actuators_msg.angles = np.array([0, 0, 0, self.current_state[3], self.current_state[4], self.current_state[5], self.current_state[6], self.current_state[7], self.current_state[8]])
        self.pub_to_actuators.publish(self.set_actuators_msg)

    def set_to_zero(self):
        print("publishing exit command")
        self.current_state = np.zeros((9,1))
        self.publish_to_actuators()


def main():
    node_name = "allocation_node_python"
    rospy.init_node(node_name)
    
    print("-------------------------")
    allocation = allocator(rate=100) 
    rospy.on_shutdown(allocation.set_to_zero)

    while not rospy.is_shutdown():
        try:
            rospy.spin()

        except rospy.ROSInterruptException:
            pass

    # rospy.signal_shutdown("Shutting down")



if __name__ == '__main__': 
    main()


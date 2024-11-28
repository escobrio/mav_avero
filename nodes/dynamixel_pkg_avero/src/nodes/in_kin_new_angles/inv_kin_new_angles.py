#!/usr/bin/env python3

import rospy
import numpy as np
from std_msgs.msg import Int32MultiArray
from std_msgs.msg import Float32MultiArray
from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des, get_nearest_rotation
#from sympy import symbols, lambdify, pi, cos, sin, asin, acos, sqrt, atan2

from functools import *


#This Node transforms a desired chi vector (phi_des, theta_des) on the endeffector of de "2DSN" to the rotation angles of the two joints.. 


#command to publish on topic: rostopic pub /normalvector_enteffector std_msgs/Float32MultiArray "{data: [0.0, 1.0, 0.0]}"

# Subscribe to "joint_angles" topic and publish to "motor/position"

def callback(msg):   

    global q #initial guess
    global first_time

    q_last = q
    
    #n = np.array(msg.data, dtype=np.float32)
    n = np.array(msg.data, dtype=np.float32)
    


    chi_des = get_chi_des(n, q)


    #message is phi and theta in radians
    ##tuning Parameters for the numerical Calculation in the while-loop
    it_max = 100
    alpha = 0.9 #still needs to be tested for different possible configurations
    damping = 0.01
    acceptable_chi_err = 1/180*np.pi

    
    q1_val, q2_val = q
    chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))
    chi_err = chi_des - chi_now 

    

    if n[0] == 0 and n[1] == 0 and n[2]>0: 
        q[0] = np.pi/2
        q[1] = 0
    elif  n[0] == 0 and n[1] == 0 and n[2]<0:
        q[0] = -np.pi/2
        q[1] = 0
    else:
        for it in range(it_max):
            if np.all(np.abs(chi_err) < acceptable_chi_err):
                break  

            #gets the now estimated values for q
            q1_val, q2_val = q

            chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))
            chi_err = chi_des - chi_now  
            
            numJac = get_numeric_jac(q1_val, q2_val)
            numJac_pinv = pseudoInverseMat(numJac, damping)

            A = ((alpha*numJac_pinv) @ chi_err).T
            q = q + A 

    

    q_normalized = np.array(
        [normalize_angle(q[0]), normalize_angle(q[1]) ]
    )
    rotation_angles_rad = q_normalized

    
    if(not first_time):
        rotation_angles_rad[0] = get_nearest_rotation(rotation_angles_rad[0], q_last[0])


    q = rotation_angles_rad

    rotation_angles_deg = rotation_angles_rad*360/(2*np.pi)
    rotation_angles_deg = np.round(rotation_angles_deg).astype(int)

    first_time = False

    rot_pub.publish(Int32MultiArray(data = rotation_angles_deg ))




if __name__ == '__main__':
    # Initialize node, topics to subscribe and publish to
    node_name = "inverse_kinematics_chi_des_to_q"
    rospy.init_node(node_name, anonymous=True)

    q = np.array([np.pi/4 ,np.pi/4])
    first_time = True


    normal_vector_sub = rospy.Subscriber('chi_des_endeffector', Float32MultiArray, callback)
    rot_pub = rospy.Publisher('joint_angles', Int32MultiArray, queue_size=10)
    rospy.spin()

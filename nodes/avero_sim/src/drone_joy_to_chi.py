#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray
from mav_msgs.msg import Actuators
import math


class converter_joy_to_chi: # Converts /joint_angles in degrees and publishes to the /command/motor_speed in radians
    def __init__(self):

        rospy.init_node(node_name)
        print(f"Initialized {node_name} node")

        self.sub = rospy.Subscriber(sub_topic_name, Joy, self.callback)
        self.pub_1 = rospy.Publisher(pub_topic_name_1, Float32MultiArray, queue_size=10)
        self.pub_2 = rospy.Publisher(pub_topic_name_2, Actuators, queue_size=10) 

        rospy.spin()

    def callback(self, msg):
        
        ## Joy to desired Chi vector ###
        # x²+y²+z²=1    =>  
        x_unit = - msg.axes[1]
        y_unit = 1
        z_unit = - msg.axes[0]

        magnitude_xy = math.sqrt(x_unit**2 + z_unit**2)

        if(magnitude_xy >= 1):
            x_unit /= magnitude_xy
            z_unit /= magnitude_xy
            
        y_unit = math.sqrt(1.0000001 - x_unit**2 - z_unit**2)
        
        print("x,y,z, magnitudexy, magnitude_total: ")
        print(x_unit, y_unit, z_unit, magnitude_xy, x_unit**2 + y_unit**2 + z_unit**2)

        self.chi_msg = Float32MultiArray()
        self.chi_msg.data = [x_unit, y_unit, z_unit]
        # self.pub_1.publish(self.chi_msg)


        ## Joy to motor_speed, Adjust joy_scaler to multiply the joy input ##
        self.Actuators_msg = Actuators()
        joy_scaler = 10
        print(f"Actuators_msg: {[msg.axes[4] * joy_scaler] * 3}")
        self.Actuators_msg.angular_velocities = [msg.axes[4] * joy_scaler] * 3
        self.pub_2.publish(self.Actuators_msg)


if __name__ == '__main__': 
    
    node_name = "joy_to_chi"        
    sub_topic_name = "/joy" 
    pub_topic_name_1 = "/chi_des_endeffector" 
    pub_topic_name_2 = "/command/motor_speed" 


    try:
        converter_joy_to_chi() 
    except rospy.ROSInterruptException:
        pass
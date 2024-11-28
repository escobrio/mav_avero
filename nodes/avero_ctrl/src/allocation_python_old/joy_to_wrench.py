#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray
from mav_msgs.msg import Actuators
import math


class converter_joy_to_wrench: # Converts /joint_angles in degrees and publishes to the /command/motor_speed in radians
    def __init__(self):

        rospy.init_node(node_name)
        print(f"Initialized {node_name} node")

        self.sub = rospy.Subscriber(sub_topic_name, Joy, self.callback)
        self.pub_1 = rospy.Publisher(pub_topic_name_1, Float64MultiArray, queue_size=1)

        rospy.spin()

    def callback(self, msg):
        
        ## Joy to desired wrench vector ###
        # x²+y²+z²=1    =>  
        x_unit = - msg.axes[0]
        y_unit = msg.axes[1]
        z_unit = 1

        magnitude_xy = math.sqrt(x_unit**2 + y_unit**2)

        if(magnitude_xy >= 1):
            x_unit /= magnitude_xy
            y_unit /= magnitude_xy
            
        z_unit = math.sqrt(1.0000001 - x_unit**2 - y_unit**2)
        
        print("x,y,z, magnitudexy, magnitude_total: ")
        print(x_unit, y_unit, z_unit, magnitude_xy, x_unit**2 + y_unit**2 + z_unit**2)

        self.wrench_msg = Float64MultiArray()
        self.wrench_msg.data = [x_unit, y_unit, z_unit, 0, 0, 0]
        self.pub_1.publish(self.wrench_msg)


if __name__ == '__main__': 
    
    node_name = "joy_to_wrench"
    sub_topic_name = "/joy" 
    pub_topic_name_1 = "/des_wrench" 


    try:
        converter_joy_to_wrench() 
    except rospy.ROSInterruptException:
        pass
#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray
from mav_msgs.msg import Actuators
import math


class converter_angles: # Converts /joint_angles in degrees and publishes to the /command/motor_speed in radians
    def __init__(self):

        rospy.init_node(node_name)
        print(f"Initialized {node_name} node")

        self.sub = rospy.Subscriber(sub_topic_name, Int32MultiArray, self.callback)
        self.pub = rospy.Publisher(pub_topic_name, Actuators, queue_size=10) 

        rospy.spin()

    def callback(self, msg):

        # Convert angles from degrees to radians
        angles_in_degrees = msg.data
        print(f"\n angles in degrees: {angles_in_degrees}")
        # To move the wanted swivel nozzle joints the message has to be weirdly shifted to the right idk why so that's why I added the "[0] + " in the next line
        angles_in_radians = [0] + [math.radians(angle) for angle in angles_in_degrees]
        print(f"\n angle in radians: {angles_in_radians}")

        self.Actuators_msg = Actuators()
        self.Actuators_msg.angles = angles_in_radians
        print(f"\n Actuators message: {self.Actuators_msg}")
        self.pub.publish(self.Actuators_msg)


if __name__ == '__main__': 
    
    node_name = "angles_to_motor"        
    pub_topic_name = "/command/motor_speed" 
    sub_topic_name = "/joint_angles" 
    
    try:
        converter_angles() 
    except rospy.ROSInterruptException:
        pass
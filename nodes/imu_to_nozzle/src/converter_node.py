#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
import numpy as np
from world_to_body_to_nozzle import world_to_nozzle



class converter_imu: # takes an array which has the size of the number of dynamixels and  sends it as individual messages to the read_write_node.py
    def __init__(self, rate):
        pub_topic_name = "throttle_topic" 
        sub_topic_name = "/imu/data" #topic comming from the inverse kinematics

        self.pub = rospy.Publisher(pub_topic_name, Float32MultiArray, queue_size=1) 
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Imu, self.callback)
        self.set_inv_kin_msg = Float32MultiArray()
        #set publishing rate
        self.rate = rospy.Rate(rate)

    def callback(self, msg):        
        w = np.float32(msg.orientation.w)
        x = np.float32(msg.orientation.x)
        y = np.float32(msg.orientation.y)
        z = np.float32(msg.orientation.z)
        q = [w,x,y,z]
        v_w=np.array([[0], #points down vector
                      [0],
                      [-1]])
        v_n=world_to_nozzle(q)@v_w 

        print("quaternion is ", q) 

        self.set_inv_kin_msg.data = v_n
        #print(self.set_inv_kin_msg)
        
        self.pub.publish(self.set_inv_kin_msg) 
        #control publishing rate
        self.rate.sleep() 


if __name__ == '__main__': 
    node_name = "converter_node"
    rospy.init_node(node_name)
    converter_node = converter_imu(rate=200) 
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
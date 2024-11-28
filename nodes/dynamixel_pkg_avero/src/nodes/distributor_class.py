#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32MultiArray 
from dynamixel_sdk_examples.msg import SetPosition 

class distributor: # takes an array which has the size of the number of dynamixels and  sends it as individual messages to the read_write_node.py
    def __init__(self, rate):
        pub_topic_name = "set_position" 
        sub_topic_name = "clicks_goal_pos_array" #topic comming from the deg_to_clicks

        self.pub = rospy.Publisher(pub_topic_name, SetPosition, queue_size=10) 
        self.number_subscriber = rospy.Subscriber(sub_topic_name, Int32MultiArray, self.callback)
        self.set_individ_pose_msg = SetPosition()
        #set publishing rate
        self.rate = rospy.Rate(rate)

    
    def callback(self, msg):
        
        for index, element in enumerate(msg.data):  #tranverses through message array
            self.set_individ_pose_msg.id = index + 1
            #because matrices start from 0 index
            self.set_individ_pose_msg.position = element
            self.pub.publish(self.set_individ_pose_msg) 
            #control publishing rate
            self.rate.sleep() 
            

         


if __name__ == '__main__': 
    node_name = "publish_bulk_commands_class"
    rospy.init_node(node_name)
    distributor_node = distributor(rate=200) 
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass



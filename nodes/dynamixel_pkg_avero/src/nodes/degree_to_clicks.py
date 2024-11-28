#!/usr/bin/env python3

#command to use rostopic pub -1 /joint_angles std_msgs/Int32MultiArray "layout:
#   dim:
#   - label: ''
#     size: 0
#     stride: 0
#   data_offset: 0
# data: [90,-90]" <-- fill in with the desired angle 

import rospy
from std_msgs.msg import Int32MultiArray

#This Node transforms a desired input degree comand to the respective clicks. 

#gear ratio=10/51
#0.088 [deg/puls]

#Tube should turn x°->°abtrieb*51/10=°Antrieb °Antrieb*[puls/deg]=puls_abtrieb
gear_ratio=51/10
puls_per_deg=1/0.088
dtc_factor = gear_ratio*puls_per_deg

# Subscribe to "vector/rotation" topic and publish to "motor/position"
def callback(msg):   
    transformed_data = Int32MultiArray() 
    transformed_data.data = [int(value * dtc_factor) for value in msg.data] 
    clicks_pub.publish(transformed_data)

if __name__ == '__main__':
    # Initialize node, topics to subscribe and publish to
    node_name = "degree_to_clicks"
    rospy.init_node(node_name)
    dtc_sub = rospy.Subscriber('joint_angles', Int32MultiArray, callback)
    clicks_pub = rospy.Publisher('clicks_goal_pos_array', Int32MultiArray, queue_size=10)
    rospy.spin()

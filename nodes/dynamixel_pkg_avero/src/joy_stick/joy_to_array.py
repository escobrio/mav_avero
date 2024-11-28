#!/usr/bin/env python
# Software License Agreement (BSD License)

## Node subscribed to /joy topic and publishing on /motor/position topic

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray

def callback(data):
    
    # Define your mapping from joystick input to Twist message here
    linear_scale = 100000  # Adjust as needed

    # Create a Int32MultiArray message
    message = Int32MultiArray()
    message.data = [0, 0]
    message.data[0]= int(linear_scale * data.axes[1])
    message.data[1] = int(linear_scale * data.axes[4])
    message.data = [int(linear_scale * data.axes[1]), int(linear_scale * data.axes[4])]

    print(f"\n left joystick: {data.axes[1]} \n right joystick: {data.axes[4]} \n publishing {message}")

    # Publish the Int32MultiArray message to the 'motor/position' topic
    motor_pos_pub.publish(message)


if __name__ == '__main__':

    #Initialize node, topics to subscribe and publish to    
    rospy.init_node('joy_to_array_node')
    joy_sub = rospy.Subscriber('/joy', Joy, callback)   #callback is a function that get's called when a subscription event occurs
    motor_pos_pub = rospy.Publisher('/motor/position', Int32MultiArray, queue_size=1)
    # intermitent_pos_pub = rospy.Publisher('/motor/position', Int32MultiArray, queue_size=1)

    # message = Int32MultiArray()
    # message.data = [0, 0]
    rate = rospy.Rate(50) #50 Hz 
    current_message = None

    while not rospy.is_shutdown():
        if current_message is not None:
            motor_pos_pub(current_message) 
            rate.sleep()
        else:
            rospy.sleep(0.1)
        
        rospy.spin()        
        rate.sleep()

    rospy.spin()
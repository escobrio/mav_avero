#!/usr/bin/env python3
# license removed for brevity
import rospy
from std_msgs.msg import Float64MultiArray
import random
import numpy as np



def publish_wrench():
    pub = rospy.Publisher('des_wrench', Float64MultiArray, queue_size=10)
    rospy.init_node('set_des_wrench', anonymous=True)
    msg = Float64MultiArray() 
    msg.data = [0, 0, 0, 0 ,0 , 0]
    rate = rospy.Rate(100) # 10hz
    counter = 0
    
    while not rospy.is_shutdown():

        msg.data[2] = 2
        msg.data[1] = 0

        # msg.data[0] = 5*np.sin(counter)
        # msg.data[1] = 15*np.sin(counter)
        # msg.data[2] = 10 - 10*np.cos(counter)
        # msg.data[3] = 2*np.sin(counter*0.5)
        # msg.data[4] = 1.5*np.sin(counter*1.5)
        # msg.data[5] = 1*np.sin(counter)

        # if counter >= 1:
        #     msg.data[2] = 1
        counter = counter + 0.003

        # if counter  <= 45 and counter >= 25:
        #     print("counter: ", counter)
        #     msg.data[1] = 0
        #     msg.data[2] = counter
        #     msg.data[0] = 0
        #     counter = counter + 0.01
        #     pub.publish(msg)
        # else:
        #     counter = 25
        #     msg.data[1] = 0
        #     msg.data[2] = counter
        #     msg.data[0] = 0
        pub.publish(msg)
        print("Published: ", msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publish_wrench ()
    except rospy.ROSInterruptException:
        pass
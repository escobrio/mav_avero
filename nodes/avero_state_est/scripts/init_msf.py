#!/usr/bin/python

# MIT License
#
# Copyright (c) 2022 Rik Baehnemann, ASL, ETH Zurich, Switzerland
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sellpython
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import rospy
import rostopic
from sensor_fusion_comm.srv import InitScale
from sensor_msgs.msg import Imu
import time
import numpy as np


def wait_for_first_msgs(input_class, input_topic, num_messages, logging_timeout):
    msgs_received = 0
    rospy.loginfo('Waiting to receive first %d messages on topic %s', num_messages, input_topic)
    while msgs_received < num_messages:
        try:
            rospy.wait_for_message(input_topic, input_class, logging_timeout)
        except rospy.ROSInterruptException as error:
            return False
        except rospy.ROSException as error:
            rospy.loginfo("Exception: %s" % error)
            continue
        rospy.loginfo_once('Received first message on topic %s.', input_topic)
        msgs_received += 1
    return True


def wait_for_msf_srv(msf_scale_topic, logging_timeout):
    while True:
        rospy.loginfo('Waiting for MSF scale initialization service on topic %s', msf_scale_topic)
        try:
            rospy.wait_for_service(msf_scale_topic, logging_timeout)
            break
        except rospy.ROSInterruptException as error:
            return False
        except rospy.ROSException as error:
            rospy.loginfo("Exception: %s" % error)
            continue
    rospy.loginfo('Found MSF scale service on topic %s.', msf_scale_topic)
    return True


def estimate_imu_biases(imu_topic, n_imu_messages, bias_param_namespace):
    input_class, input_topic, input_fn = rostopic.get_topic_class(imu_topic, blocking=True)
    if wait_for_first_msgs(input_class, input_topic, num_messages, logging_timeout):
        # read first n imu messages and average them
        imu_dict = {"received_imu_msgs": 0, "accel": np.zeros((n_imu_messages, 3)), "ang_vel": np.zeros((n_imu_messages, 3))}

        def imu_callback(msg, args):
            if args["received_imu_msgs"] < args["accel"].shape[0]:
                args["accel"][args["received_imu_msgs"], :] = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
                args["ang_vel"][args["received_imu_msgs"], :] = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            args["received_imu_msgs"] += 1

        sub = rospy.Subscriber(imu_topic, Imu, imu_callback, imu_dict)
        while imu_dict["received_imu_msgs"] < n_imu_messages:
            time.sleep(0.01)

        # we know enough unsubscribe
        sub.unregister()

        # Take average and set biases:
        b_accel = np.mean(imu_dict["accel"], axis=0) - np.array([0.0, 0.0, 9.806])
        b_gyro = np.mean(imu_dict["ang_vel"], axis=0)

        rospy.set_param(bias_param_namespace +"/b_a/x", float(b_accel[0]))
        rospy.set_param(bias_param_namespace +"/b_a/y", float(b_accel[1]))
        rospy.set_param(bias_param_namespace +"/b_a/z", float(b_accel[2]))
        rospy.set_param(bias_param_namespace +"/b_w/x", float(b_gyro[0]))
        rospy.set_param(bias_param_namespace +"/b_w/y", float(b_gyro[1]))
        rospy.set_param(bias_param_namespace +"/b_w/z", float(b_gyro[2]))



if __name__ == '__main__':
    rospy.init_node('init_msf', anonymous=True)
    # Get ROS parameters.
    topic = rospy.get_param('~topic', '/sensor/transform_stamped')
    msf_scale_topic = rospy.get_param('~msf_scale_topic', '/msf/pose_sensor/initialize_msf_scale')
    scale = rospy.get_param('~scale', 1.0)
    logging_timeout = rospy.get_param('~logging_timeout', 5.0)
    num_messages = rospy.get_param('~num_messages', 1)

    init_biases = rospy.get_param('~init_biases', False)
    imu_topic = rospy.get_param('~imu_topic', '~imu/data_raw')
    bias_param_namespace = rospy.get_param('~bias_param_namespace', '/stork/eval/msf_sensorpod/core/init')
    n_imu_messages = rospy.get_param('~num_imu_messages', 100)

    if init_biases:
        # read first n imu messages and average them
        estimate_imu_biases(imu_topic, n_imu_messages, bias_param_namespace)

    # Wait for topic to appear on ROS network.
    input_class, input_topic, input_fn = rostopic.get_topic_class(topic, blocking=True)

    # Initialization sequence.
    has_msg = wait_for_first_msgs(input_class, input_topic, num_messages, logging_timeout)
    if has_msg:
        has_srv = wait_for_msf_srv(msf_scale_topic, logging_timeout)

        if has_srv:
            rospy.loginfo('Initializing MSF with scale %f on topic %s.', scale, msf_scale_topic)
            try:
                srv = rospy.ServiceProxy(msf_scale_topic, InitScale)
                resp = srv(scale)
            except rospy.ServiceException as e:
                rospy.logerr("Service call failed: %s" % e)

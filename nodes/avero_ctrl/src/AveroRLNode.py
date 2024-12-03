#!/usr/bin/env python3
import rospy
from stable_baselines3 import PPO
import numpy as np
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
from omav_msgs.msg import Actuators
from scipy.spatial.transform import Rotation as R
import threading

class RLAgentNode:
    def __init__(self, model_path):
        self.model = PPO.load(model_path)
        rospy.init_node('rl_agent_node', anonymous=True)
        
        self.latest_observation = None
        self.lock = threading.Lock()  # To avoid race conditions
        self.last_action = np.zeros(9)
        
        self.sub_observations = rospy.Subscriber('/gazebo/model_states', ModelStates, self.observation_callback)
        self.pub_command_motorspeed = rospy.Publisher('/command/motor_speed', Actuators, queue_size=1)
        # Publishers for debugging
        self.pub_observations = rospy.Publisher('/debug/observations', Float32MultiArray, queue_size=1)
        self.pub_action = rospy.Publisher('/debug/actions', Float32MultiArray, queue_size=1)
    
        # Timer for publishing observations at a stable rate (e.g., 100 Hz)
        rospy.Timer(rospy.Duration(1.0 / 100), self.publish_observations)
        rospy.loginfo("\033[92mRL Agent Node initialized. Waiting for observations...\033[0m")
        

    def observation_callback(self, msg):
        try: 
            # Get observation
            lin_vel = [msg.twist[2].linear.x, msg.twist[2].linear.y, msg.twist[2].linear.z]
            ang_vel = [msg.twist[2].angular.x, msg.twist[2].angular.y, msg.twist[2].angular.z]
            q = [msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w] 
            orientation = R.from_quat(q)

            # Rotate world-frame vectors into body-frame
            lin_vel = orientation.inv().apply(lin_vel)
            ang_vel = orientation.inv().apply(ang_vel)
            g_bodyframe = orientation.inv().apply(np.array([0, 0, -9.81])) 
            observation = np.concatenate([lin_vel, ang_vel, g_bodyframe, self.last_action])
            
            with self.lock:
                self.latest_observation = observation

        except Exception as e:
            rospy.logerr(f"Error in observation callback: {e}")

    def publish_observations(self, event):
        with self.lock:
            if self.latest_observation is not None:
                msg_obs = Float32MultiArray()
                msg_obs.data = self.latest_observation
                self.pub_observations.publish(msg_obs)

                # Publish action
                action, _ = self.model.predict(self.latest_observation, deterministic=True)
                self.last_action += action
                msg_act = Float32MultiArray()
                msg_act.data = self.last_action
                self.pub_action.publish(msg_act)
                action_msg = Actuators()
                action_msg.angles = [0, 0, 0] + list(self.last_action[3:9])
                # fanspeed action is normalized in [-1, 1], convert as needed
                action_msg.angular_velocities = list((self.last_action[0:3] + 1) * 450) + [0, 0, 0, 0, 0, 0]
                self.pub_command_motorspeed.publish(action_msg)


if __name__ == '__main__':
    try: 
        model_path = '/home/dedi/catkin_ws/src/mav_avero/nodes/avero_ctrl/resources/model_1.zip'

        node = RLAgentNode(model_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RL Agent Node shutting down.")
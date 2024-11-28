#!/usr/bin/env python3
import rospy
from stable_baselines3 import PPO
import numpy as np
from std_msgs.msg import Float32MultiArray
from gazebo_msgs.msg import ModelStates
from omav_msgs.msg import Actuators
from scipy.spatial.transform import Rotation as R

class RLAgentNode:
    def __init__(self, model_path):
        
        self.model = PPO.load(model_path)
        rospy.init_node('rl_agent_node', anonymous=True)
        self.sub_observations = rospy.Subscriber('/gazebo/model_states', ModelStates, self.observation_callback)
        self.pub_command_motorspeed = rospy.Publisher('/command/motor_speed', Actuators, queue_size=10)
        # Publishers for debugging
        self.pub_observations = rospy.Publisher('/debug/observations', Float32MultiArray, queue_size=10)
        self.pub_action = rospy.Publisher('/debug/actions', Float32MultiArray, queue_size=10)
        rospy.loginfo("RL Agent Node initialized. Wating for observations...")

    def observation_callback(self, msg):
        try: 
            # Get observation
            lin_vel = [msg.twist[2].linear.x, msg.twist[2].linear.y, msg.twist[2].linear.z]
            ang_vel = [msg.twist[2].angular.x, msg.twist[2].angular.y, msg.twist[2].angular.z]
            q = [msg.pose[2].orientation.x, msg.pose[2].orientation.y, msg.pose[2].orientation.z, msg.pose[2].orientation.w] 
            orientation = R.from_quat(q)
            g_bodyframe = orientation.inv().apply(np.array([0, 0, -9.81])) 
            observation = np.concatenate([lin_vel, ang_vel, g_bodyframe])
            msg_obs = Float32MultiArray()
            msg_obs.data = observation
            self.pub_observations.publish(msg_obs)
            
            # Publish action
            action, _ = self.model.predict(observation, deterministic=True)
            msg_act = Float32MultiArray()
            msg_act.data = action
            self.pub_action.publish(msg_act)
            action_msg = Actuators()
            action_msg.angles = [0, 0, 0] + list(action[3:9])
            # fanspeed action is normalized in [-1, 1], convert as needed
            action_msg.angular_velocities = list((action[0:3] + 1) * 450) + [0, 0, 0, 0, 0, 0]
            self.pub_command_motorspeed.publish(action_msg)
            rospy.loginfo(f"Received observation: {observation}, Published action: {action}")
        except Exception as e:
            rospy.logerr(f"Error in processing observations: {e}")

if __name__ == '__main__':
    try: 
        model_path = '/home/dedi/BachelorThesis/AveroRL/data/ppo_mav_model.zip'

        node = RLAgentNode(model_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RL Agent Node shutting down.")
#!/usr/bin/env python3
import rospy
from stable_baselines3 import PPO
import numpy as np
from std_msgs.msg import Float32MultiArray

class RLAgentNode:
    def __init__(self, model_path):
        
        self.model = PPO.load(model_path)
        rospy.init_node('rl_agent_node', anonymous=True)
        self.sub_observations = rospy.Subscriber('/observations', Float32MultiArray, self.observation_callback)
        self.pub_actions = rospy.Publisher('/actions', Float32MultiArray, queue_size=10)
        rospy.loginfo("RL Agent Node initialized. Wating for observations...")

    def observation_callback(self, msg):
        try: 
            observation = np.array(msg.data)
            action, _ = self.model.predict(observation, deterministic=True)
            action_msg = Float32MultiArray()
            action_msg.data = action
            self.pub_actions.publish(action_msg)
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
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
        
        self.latest_observation = np.zeros(33)
        self.last_setpoints = np.array([0.6, 0.6, 0.6, 0.80, -1.25, 0.80, -1.25, 0.80, -1.25])
        self.last_action = None
        self.phi_dot_max = 1.0
        self.omega_dot_max = 5.0
        self.k_phi = 6.45
        self.k_omega = 12.253
        
        self.sub_observations = rospy.Subscriber('/gazebo/model_states', ModelStates, self.observation_callback, queue_size=1)
        self.sub_motorspeed = rospy.Subscriber('/gazebo/motor_states', Actuators, self.actuator_callback, queue_size=1)
        self.pub_command_motorspeed = rospy.Publisher('/command/motor_speed', Actuators, queue_size=1)
        # Publishers for debugging
        self.pub_observations = rospy.Publisher('/debug/observations', Float32MultiArray, queue_size=1)
        self.pub_setpoints = rospy.Publisher('/debug/setpoints', Float32MultiArray, queue_size=1)
        self.pub_actions = rospy.Publisher('/debug/actions', Float32MultiArray, queue_size=1)
    
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
            observation = np.concatenate([lin_vel, ang_vel, g_bodyframe])
            
            self.latest_observation[:9] = observation

        except Exception as e:
            rospy.logerr(f"Error in observation callback: {e}")

    def actuator_callback(self, msg):
        try: 
            # Get observation
            fanspeeds = msg.angular_velocities[:3]
            fanspeeds_norm = np.array(fanspeeds) / 900
            nozzleangles = np.array(msg.angles[3:9])
            nozzleangles_sin = np.sin(nozzleangles)
            nozzleangles_cos = np.cos(nozzleangles)

            observation = np.concatenate([fanspeeds_norm, nozzleangles_sin, nozzleangles_cos, self.last_action])
            
            self.latest_observation[9:33] = observation

        except Exception as e:
            rospy.logerr(f"Error in actuator callback: {e}\n {len(observation)}, {observation}")

    def publish_observations(self, event):
        try:
            if self.latest_observation is not None:
                msg_obs = Float32MultiArray()
                msg_obs.data = self.latest_observation
                self.pub_observations.publish(msg_obs)

                # Publish action
                action, _ = self.model.predict(self.latest_observation, deterministic=True)
                self.last_action = action
                # Scale Action
                
                nozzles_state = np.arcsin(self.latest_observation[12:18])
                nozzles_setpoint = nozzles_state + (action[3:] * self.phi_dot_max) / self.k_phi
                fanspeeds_state = self.latest_observation[9:12]
                fanspeeds_setpoint = fanspeeds_state + (action[:3] * self.omega_dot_max) / self.k_omega

                self.last_setpoints = np.concatenate((fanspeeds_setpoint, nozzles_setpoint))
                self.last_setpoints[:3] = self.last_setpoints[:3].clip(0,1)
                self.last_setpoints[3:] = self.last_setpoints[3:].clip(-np.pi, np.pi)

                msg_setpoints = Float32MultiArray()
                msg_setpoints.data = self.last_setpoints
                self.pub_setpoints.publish(msg_setpoints)
                msg_actions = Float32MultiArray()
                msg_actions.data = action
                self.pub_actions.publish(msg_actions)
                action_msg = Actuators()
                action_msg.angles = [0, 0, 0] + list(self.last_setpoints[3:9])
                # fanspeed action is normalized in [-1, 1], convert as needed
                action_msg.angular_velocities = list(self.last_setpoints[0:3] * 900) + [0, 0, 0, 0, 0, 0]
                self.pub_command_motorspeed.publish(action_msg)
        except Exception as e:
            rospy.logerr(f"Error in publish observations: {e}\n {action}")    


if __name__ == '__main__':
    try: 
        model_path = '/home/dedi/catkin_ws/src/mav_avero/nodes/avero_ctrl/resources/model_8.zip'

        node = RLAgentNode(model_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("RL Agent Node shutting down.")
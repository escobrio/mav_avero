#!/usr/bin/env python3

import numpy as np
def rot_from_quat(q): 
    q[1] = -q[1]
    q[2] = -q[2]
    q[3] = -q[3]
    rot_mat= np.array([[q[0]**2+q[1]**2-q[2]**2-q[3]**2, 2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[2]+2*q[1]*q[3]],
                       [2*q[0]*q[3]+2*q[1]*q[2], q[0]**2-q[1]**2+q[2]**2-q[3]**2, 2*q[2]*q[3]-2*q[0]*q[1]], 
                       [2*q[1]*q[3]-2*q[0]*q[2], 2*q[0]*q[1]+2*q[2]*q[3], q[0]**2-q[1]**2-q[2]**2+q[3]**2]])
    return rot_mat
def Body_Frame_to_nozzle(): #tested it works 
    R_nb=np.array([[0,0,-1],
                  [-1, 0, 0],
                  [0,1,0]])
    
    
    return R_nb

def world_frame_to_body_frame(q):
    # Use the Rotation.from_quat method directly
    R_bw= rot_from_quat(q)
    
    return R_bw


def world_to_nozzle(q): 
    R_nb = Body_Frame_to_nozzle() 

    R_bw = world_frame_to_body_frame(q)
    result = R_nb@R_bw
    return result


vector = np.array([[1], 
                   [0], 
                   [0]])

angle_rad = np.radians(90)

""" # Quaternion for 90-degree rotation around the Z-axis
quaternion = np.array([np.cos(angle_rad / 2), 0, 0, np.sin(angle_rad / 2)])
 """
angle_rad = np.radians(180)
quaternion = np.array([np.cos(angle_rad / 2), 0, 0, np.sin(angle_rad / 2)]) 

R_nb = Body_Frame_to_nozzle() 

R_bw = world_frame_to_body_frame(quaternion)
""" result = R_nb@vector """

print("Quaternion:", quaternion)
result = world_to_nozzle(quaternion)@vector 
#vector is given in world frame 
print("Result in nozzle is", world_to_nozzle(quaternion))
print("Result in nozzle is", result)


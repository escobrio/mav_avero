import sys
sys.path.append('/local/home/ebochlogyrou/catkin_ws/src/mav_avero/nodes/dynamixel_pkg_avero/src/nodes/in_kin_new_angles')
from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des, getJacobian_angles_sym

import matplotlib.pyplot as plt
import numpy as np


thrust = get_thrustvector_num(0, 0.5) 
print(thrust)
theta = np.arccos(thrust[2] / np.linalg.norm(thrust))
print(theta-np.pi/2)
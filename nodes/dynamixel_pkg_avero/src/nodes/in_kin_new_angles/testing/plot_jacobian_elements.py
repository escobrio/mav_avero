import sys
sys.path.append('/local/home/ebochlogyrou/catkin_ws/src/mav_avero/nodes/dynamixel_pkg_avero/src/nodes/in_kin_new_angles')
from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des, getJacobian_angles_sym

import matplotlib.pyplot as plt
import numpy as np

q1 = 0
q2_values = np.arange(0, np.pi, 0.001)

# Initialize firstElement and secondElement as empty lists
firstElement = []
secondElement = []

for q2 in q2_values:
    J = get_numeric_jac(q1, q2)
    secondColumn = J[:, 1]
    firstElement.append(secondColumn[0])
    secondElement.append(secondColumn[1])
    print("q2 is equal: %s degrees, second jacobian column is equal to : %s" % (np.degrees(q2), secondColumn))

# Convert lists to numpy arrays for plotting
firstElement = np.array(firstElement)
secondElement = np.array(secondElement)

# Plot firstElement and secondElement against q2_values
plt.figure()
plt.plot(q2_values, firstElement, label='First Element d_phi/dq2', linewidth=6)  # increase line thickness
plt.plot(q2_values, secondElement, label='Second Element d_theta/dq2', linewidth=6)  # increase line thickness
plt.xlabel('q2', fontsize=20)  # increase font size for x-axis label
plt.ylabel('Element Value', fontsize=20)  # increase font size for y-axis label
plt.legend(fontsize=20)  # increase legend font size
plt.xticks(fontsize=20)  # increase font size for x-axis ticks
plt.yticks(fontsize=20)  # increase font size for y-axis ticks
plt.grid(True)  # add grid
plt.show()

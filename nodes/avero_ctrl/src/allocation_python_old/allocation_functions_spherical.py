import numpy as np
import sympy as sp
import sys
sys.path.append("/local/home/ebochlogyrou/catkin_ws/src/mav_avero/nodes/dynamixel_pkg_avero/src/nodes/in_kin_new_angles")

# Now you can import your module
from helper_functions_symb import getJacobian_angles_sym, get_thrustvector_symbolic, getangles_sym, get_T_0E_symbolic
from sympy import IndexedBase, Idx, symbols, Matrix, zeros, Indexed
from sympy import symbols, sqrt, asin, acos,  diff, pi, cos, sin, lambdify, simplify, atan2


def vector_to_skew_symmetric(v):
    return np.array([[0, -v[2], v[1]], 
                     [v[2], 0, -v[0]], 
                     [-v[1], v[0], 0]])


#define the input variables

omega_1, omega_2, omega_3 = symbols('omega_1 omega_2 omega_3')
omega_dot_1, omega_dot_2, omega_dot_3 = symbols('omega_dot_1 omega_dot_2 omega_dot_3')


#phi indices: first comes the nozzle number and then the joint angle number 

phi_1_1, phi_1_2, phi_2_1, phi_2_2, phi_3_1, phi_3_2 = symbols('phi_1_1 phi_1_2 phi_2_1 phi_2_2 phi_3_1 phi_3_2') 
phi_dot_1_1, phi_dot_1_2, phi_dot_2_1, phi_dot_2_2, phi_dot_3_1, phi_dot_3_2 = symbols('phi_dot_1_1 phi_dot_1_2 phi_dot_2_1 phi_dot_2_2 phi_dot_3_1 phi_dot_3_2')

k_f = symbols('k_f')

x_1, y_1, z_1 = symbols('x_1 y_1 z_1')
x_2, y_2, z_2 = symbols('x_2 y_2 z_2')
x_3, y_3, z_3 = symbols('x_3 y_3 z_3')


u_dot = np.array([[omega_dot_1], [omega_dot_2], [omega_dot_3], [phi_dot_1_1], [phi_dot_1_2], [phi_dot_2_1], [phi_dot_2_2], [phi_dot_3_1], [phi_dot_3_2]])
# define thrust vector
q1, q2 = symbols('q1 q2') 
n_1_symb = getangles_sym()
n_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in n_1_symb])

n_2_symb = getangles_sym()
n_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in n_2_symb])

n_3_symb = getangles_sym()
n_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in n_3_symb])
# print(n_3.shape)

T_1_sym = get_thrustvector_symbolic()
T_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in T_1_sym])
T_2_sym = get_thrustvector_symbolic()
T_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in T_2_sym])
T_3_sym = get_thrustvector_symbolic()
T_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in T_3_sym])

# print(T_1)
F_1 = k_f*omega_1*omega_1*T_1
F_2 = k_f*omega_2*omega_2*T_2
F_3 = k_f*omega_3*omega_3*T_3

# print(F_1)


#define jacobian matrix
J_n_1 = getJacobian_angles_sym()
J_n_1 = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in J_n_1]))
J_n_2 = getJacobian_angles_sym()
J_n_2 = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in J_n_2]))
J_n_3 = getJacobian_angles_sym()
J_n_3 = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in J_n_3]))
# print(J_n_3.shape)
#define A_F matrix

a_f_1 = 2*k_f*omega_1*n_1
a_f_2 = 2*k_f*omega_2*n_2
a_f_3 = 2*k_f*omega_3*n_3

b_f_1 = k_f*omega_1*omega_1*J_n_1
b_f_2 = k_f*omega_2*omega_2*J_n_2
b_f_3 = k_f*omega_3*omega_3*J_n_3


A_F_1 = np.column_stack((a_f_1, b_f_1[:,0], b_f_1[:,1]))
A_F_2 = np.column_stack((a_f_2, b_f_2[:,0], b_f_2[:,1]))
A_F_3 = np.column_stack((a_f_3, b_f_3[:,0], b_f_3[:,1]))
# print (A_F_1.shape)
# print (b_f_1[:,0].shape)
                                


A_F = np.array([a_f_1, a_f_2, a_f_3, b_f_1[:,0], b_f_1[:,1], b_f_2[:,0], b_f_2[:, 1], b_f_3[:,0], b_f_3[:,1]])
# print(A_F)

F_dot = A_F*u_dot 

#define the A_tau matrix

    #define r_i


r_1 = np.array([[x_1], [y_1], [z_1]])
r_2 = np.array([[x_2], [y_2], [z_2]])
r_3 = np.array([[x_3], [y_3], [z_3]])


#calculate J_r 
r_BG_1_sym = get_T_0E_symbolic()
r_BG_1_sub = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in r_BG_1_sym]))
r_BG_1 = r_BG_1_sub[0:3, 3].reshape(3,1)

r_BG_2_sym = get_T_0E_symbolic()
r_BG_2_sub = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in r_BG_2_sym]))
r_BG_2 = r_BG_2_sub[0:3, 3].reshape(3,1)

r_BG_3_sym = get_T_0E_symbolic()
r_BG_3_sub = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in r_BG_3_sym]))
r_BG_3 = r_BG_3_sub[0:3, 3].reshape(3,1)
# print(r_BG_1.shape)




J_r_1 = np.array([[diff(r_BG_1[0,0], phi_1_1), diff(r_BG_1[0,0], phi_1_2)],
                  [diff(r_BG_1[1,0], phi_1_1), diff(r_BG_1[1,0], phi_1_2)],
                  [diff(r_BG_1[2,0], phi_1_1), diff(r_BG_1[2,0], phi_1_2)]])

J_r_2 = np.array([[diff(r_BG_2[0,0], phi_2_1), diff(r_BG_2[0,0], phi_2_2)],
                  [diff(r_BG_2[1,0], phi_2_1), diff(r_BG_2[1,0], phi_2_2)],
                  [diff(r_BG_2[2,0], phi_2_1), diff(r_BG_2[2,0], phi_2_2)]])

J_r_3 = np.array([[diff(r_BG_3[0,0], phi_3_1), diff(r_BG_3[0,0], phi_3_2)],
                  [diff(r_BG_3[1,0], phi_3_1), diff(r_BG_3[1,0], phi_3_2)],
                  [diff(r_BG_3[2,0], phi_3_1), diff(r_BG_3[2,0], phi_3_2)]])

print(J_r_3.shape)

# print(J_r_1.shape)






# A_tau_1 = np.array([vector_to_skew_symmetric(r_1)*A_F_1 - [np.zeros(3,1), vector_to_skew_symmetric(F_1)]])







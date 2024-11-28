import numpy as np
import sympy as sp
from sympy import N
import sys
import os
import copy
sys.path.append(f"{os.environ['HOME']}/catkin_ws/src/mav_avero/nodes/dynamixel_pkg_avero/src/nodes/in_kin_new_angles")

# Now you can import your module
from helper_functions_symb_new_sim import getJacobian_angles_sym, get_thrustvector_symbolic, get_thrustvector_symb_base_frame, getangles_sym, get_T_BE_symbolic, getJacobian_non_minimal_sym, pseudoInverseMat, nozzle_i_to_body 
from sympy import IndexedBase, Idx, symbols, Matrix, zeros, Indexed
from sympy import symbols, sqrt, asin, acos,  diff, pi, cos, sin, lambdify, simplify, atan2


# define the input variables

omega_1, omega_2, omega_3 = symbols('omega_1 omega_2 omega_3')
omega_dot_1, omega_dot_2, omega_dot_3 = symbols('omega_dot_1 omega_dot_2 omega_dot_3')


#phi indices: first comes the nozzle number and then the joint angle number 

phi_1_1, phi_1_2, phi_2_1, phi_2_2, phi_3_1, phi_3_2 = symbols('phi_1_1 phi_1_2 phi_2_1 phi_2_2 phi_3_1 phi_3_2') 
phi_dot_1_1, phi_dot_1_2, phi_dot_2_1, phi_dot_2_2, phi_dot_3_1, phi_dot_3_2 = symbols('phi_dot_1_1 phi_dot_1_2 phi_dot_2_1 phi_dot_2_2 phi_dot_3_1 phi_dot_3_2')

# EDF thrust coefficient
k_f = symbols('k_f')

x_1, y_1, z_1 = symbols('x_1 y_1 z_1')
x_2, y_2, z_2 = symbols('x_2 y_2 z_2')
x_3, y_3, z_3 = symbols('x_3 y_3 z_3')

u = np.array([[omega_1], [omega_2], [omega_3], [phi_1_1], [phi_1_2], [phi_2_1], [phi_2_2], [phi_3_1], [phi_3_2]])
u_dot = np.array([[omega_dot_1], [omega_dot_2], [omega_dot_3], [phi_dot_1_1], [phi_dot_1_2], [phi_dot_2_1], [phi_dot_2_2], [phi_dot_3_1], [phi_dot_3_2]])
# define thrust vector
q1, q2 = symbols('q1 q2') 



def normalize_angle(angle):
    two_pi = 2 * np.pi
    normalized_angle = angle % two_pi

    # Ensure the result is within the range [-pi, pi]
    if normalized_angle > np.pi:
        normalized_angle -= two_pi

    return normalized_angle


#for calculating cross product
def vector_to_skew_symmetric(v):
    return np.array([[0, -v[2], v[1]], 
                     [v[2], 0, -v[0]], 
                     [-v[1], v[0], 0]])

#for integrating the u_dot vector

def integrate(velocity, prev_position, timestep):
    result = prev_position
    result += velocity * timestep
    return result


def derive(wrench, prev_wrench_cmd, dt, jerk_gain): 
    wrench_dot = np.zeros((6,1))
    wrench_dot = jerk_gain*(wrench - prev_wrench_cmd)/dt
    #prev_wrench_cmd = wrench
    return wrench_dot 


def allocation_matrix():

    n_1_symb = get_thrustvector_symb_base_frame(1)
    n_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in n_1_symb])
    n_2_symb = get_thrustvector_symb_base_frame(2)
    n_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in n_2_symb])
    n_3_symb = get_thrustvector_symb_base_frame(3)
    n_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in n_3_symb])

    #print(n_3.shape)

    # might need to check forward kinematics
    # what's the difference between this and the one above?
    T_1_sym = get_thrustvector_symb_base_frame(1)
    T_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in T_1_sym])
    T_2_sym = get_thrustvector_symb_base_frame(2)
    T_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in T_2_sym])
    T_3_sym = get_thrustvector_symb_base_frame(3)
    T_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in T_3_sym])
    #print(T_1.shape)

    F_1 = k_f*omega_1*omega_1*T_1
    F_2 = k_f*omega_2*omega_2*T_2
    F_3 = k_f*omega_3*omega_3*T_3

    # print(F_1)

    #define non minimal jacobian matrix

    J_n_minimal_1 = getJacobian_non_minimal_sym(1)
    J_n_minimal_1 = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in J_n_minimal_1]))
    J_n_minimal_2 = getJacobian_non_minimal_sym(2)
    J_n_minimal_2 = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in J_n_minimal_2]))
    J_n_minimal_3 = getJacobian_non_minimal_sym(3)
    J_n_minimal_3 = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in J_n_minimal_3]))
    
    #print("Jacobian nonminimal shape is", J_n_minimal_3.shape)

    #define A_F matrix

    a_f_1 = 2*k_f*omega_1*n_1
    a_f_2 = 2*k_f*omega_2*n_2
    a_f_3 = 2*k_f*omega_3*n_3

    # added missing omega square
    b_f_1 = k_f*omega_1*omega_1*J_n_minimal_1
    b_f_2 = k_f*omega_2*omega_2*J_n_minimal_2
    b_f_3 = k_f*omega_3*omega_3*J_n_minimal_3

    #print("b_f_1 =", b_f_1.shape)


    A_F_1 = np.column_stack((a_f_1, b_f_1[:,0], b_f_1[:,1]))
    #print("A_F_1 is: %s" % (str(A_F_1)))
    A_F_2 = np.column_stack((a_f_2, b_f_2[:,0], b_f_2[:,1]))
    A_F_3 = np.column_stack((a_f_3, b_f_3[:,0], b_f_3[:,1]))
    #print("A_F_1 shape is: %s" % (str(A_F_1.shape)))
    # print (b_f_1[:,0].shape)
                                    


    A_F = np.column_stack([a_f_1, a_f_2, a_f_3, b_f_1[:,0], b_f_1[:,1], b_f_2[:,0], b_f_2[:, 1], b_f_3[:,0], b_f_3[:,1]])
    #print("A_F = ", A_F)

    F_dot = A_F@u_dot 

    #define the A_tau matrix

        #define r_i


    # might need to check T_BE

    #calculate J_r 

    # r_BG_i is the vector from the base of the drone to the outlet of the i-th nozzle --> notation is shit
    r_BG_1_sym = get_T_BE_symbolic(1)
    r_BG_1_sub = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in r_BG_1_sym]))
    r_BG_1 = r_BG_1_sub[0:3, 3].reshape(3,1) #you get the last column which is the translation of the frames

    r_BG_2_sym = get_T_BE_symbolic(2)
    r_BG_2_sub = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in r_BG_2_sym]))
    r_BG_2 = r_BG_2_sub[0:3, 3].reshape(3,1)

    r_BG_3_sym = get_T_BE_symbolic(3)
    r_BG_3_sub = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in r_BG_3_sym]))
    r_BG_3 = r_BG_3_sub[0:3, 3].reshape(3,1)
    # print("r_BG_1: ", r_BG_1)
    # print("shape of r_BG_1: ", r_BG_1.shape)

    # calculate r_i: the vector from nozzle base frame origin to the center of the outlet of the i-th nozzle 
    #r_BG_i: vector from the base of the drone to the outlet of the i-th nozzle 
    #vectors with x,y,z values vector from the base of the drone to the base of the nozzle platform specific have passed it as a parameter in my node/class 

    #r_i is the vector from the base of the nozzle to the i-th nozzle outlet with respect to the body frame --> notation!!!!!!!!!!!!!!!
    r_1 = (-1)*np.array([[x_1], [y_1], [z_1]]) + r_BG_1 
    r_2 = (-1)*np.array([[x_2], [y_2], [z_2]]) + r_BG_2
    r_3 = (-1)*np.array([[x_3], [y_3], [z_3]]) + r_BG_3 

    ###might need to change r_1 --> r_BG_1

    # think this should be r_BG_i but not sure
    J_r_1 = np.array([[diff(r_1[0,0], phi_1_1), diff(r_1[0,0], phi_1_2)],
                    [diff(r_1[1,0], phi_1_1), diff(r_1[1,0], phi_1_2)],
                    [diff(r_1[2,0], phi_1_1), diff(r_1[2,0], phi_1_2)]])

    J_r_2 = np.array([[diff(r_2[0,0], phi_2_1), diff(r_2[0,0], phi_2_2)],
                    [diff(r_2[1,0], phi_2_1), diff(r_2[1,0], phi_2_2)],
                    [diff(r_2[2,0], phi_2_1), diff(r_2[2,0], phi_2_2)]])

    J_r_3 = np.array([[diff(r_3[0,0], phi_3_1), diff(r_3[0,0], phi_3_2)],
                    [diff(r_3[1,0], phi_3_1), diff(r_3[1,0], phi_3_2)],
                    [diff(r_3[2,0], phi_3_1), diff(r_3[2,0], phi_3_2)]])

    #print(J_r_3.shape)

    # print(J_r_1.shape)

    a_tau_1 = vector_to_skew_symmetric(r_BG_1)@A_F_1
    a_tau_2 = vector_to_skew_symmetric(r_BG_2)@A_F_2
    a_tau_3 = vector_to_skew_symmetric(r_BG_3)@A_F_3
    # print("a_tau_1 is: ", type(a_tau_1))



    b_tau_1 = np.hstack((np.zeros((3,1)), vector_to_skew_symmetric(F_1) @ J_r_1))
    b_tau_2 = np.hstack((np.zeros((3,1)), vector_to_skew_symmetric(F_2) @ J_r_2))
    b_tau_3 = np.hstack((np.zeros((3,1)), vector_to_skew_symmetric(F_3) @ J_r_3))
    #print("shape of b_tau_1 is: %s" % (str(b_tau_1.shape)))

    A_tau_1 = np.array(a_tau_1 - b_tau_1)
    A_tau_2 = np.array(a_tau_2 - b_tau_2)
    A_tau_3 = np.array(a_tau_3 - b_tau_3)
    #print("shape of A_tau_1 is: %s" % (str(A_tau_1.shape)))

    A_tau = np.column_stack([A_tau_1[:,0], A_tau_2[:,0], A_tau_3[:,0], A_tau_1[:,1],A_tau_1[:,2], A_tau_2[:,1], A_tau_2[:,2], A_tau_3[:,1], A_tau_3[:,2]]) 
    # print("A_tau", A_tau)

    #print("shape of A_tau is: %s" % (str(A_tau.shape)))

    tau_dot = A_tau@u_dot

    A = np.row_stack([A_F, A_tau])
    #print("shape of A is: %s" % (str(A.shape))) 
    #A_simplified = simplify(A)
    # np. print("A[0,:] is:", simplify(A[0,:]))savetxt('A_simplified.txt', A, delimiter=',')

    # print("A[0,:] is:\n", simplify(A[0,:]))
    # print("A[1,:] is:\n", simplify(A[1,:]))
    # print("A[2,:] is:\n", simplify(A[2,:]))
    # print("A[3,:] is:\n", simplify(A[3,:]))
    # print("A[4,:] is:\n", simplify(A[4,:]))
    # print("A[5,:] is:\n", simplify(A[5,:]))
    return simplify(A) 





    
def evaluate_allocation_matrix(A_symb, omega_1_val, omega_2_val, omega_3_val, phi_1_1_val, phi_1_2_val, phi_2_1_val, phi_2_2_val, phi_3_1_val, phi_3_2_val, k_f_val, x_1_val, y_1_val, z_1_val, x_2_val, y_2_val, z_2_val, x_3_val, y_3_val, z_3_val):
    # Create a new 2D array to store the numerical values of A_symb
    A_num = np.copy(A_symb)

    # Evaluate each element of the matrix at specific values
    for i in range(A_num.shape[0]):
        for j in range(A_num.shape[1]):
            if isinstance(A_num[i, j], (list, np.ndarray)):  # If the element is a list or ndarray
                for k in range(len(A_num[i, j])):
                    A_num[i, j][k] = A_num[i, j][k].subs([(omega_1, omega_1_val), (omega_2, omega_2_val), (omega_3, omega_3_val), 
                                                  (phi_1_1, phi_1_1_val), (phi_1_2, phi_1_2_val), (phi_2_1, phi_2_1_val), 
                                                  (phi_2_2, phi_2_2_val), (phi_3_1, phi_3_1_val), (phi_3_2, phi_3_2_val), 
                                                  (k_f, k_f_val), (x_1, x_1_val), (y_1, y_1_val), (z_1, z_1_val), 
                                                  (x_2, x_2_val), (y_2, y_2_val), (z_2, z_2_val), (x_3, x_3_val), 
                                                  (y_3, y_3_val), (z_3, z_3_val)])
                # Convert the list to a numpy array
                A_num[i, j] = np.array(A_num[i, j], dtype=float)
            else:  # If the element is a single expression
                A_num[i, j] = A_num[i, j].subs([(omega_1, omega_1_val), (omega_2, omega_2_val), (omega_3, omega_3_val), 
                                        (phi_1_1, phi_1_1_val), (phi_1_2, phi_1_2_val), (phi_2_1, phi_2_1_val), 
                                        (phi_2_2, phi_2_2_val), (phi_3_1, phi_3_1_val), (phi_3_2, phi_3_2_val), 
                                        (k_f, k_f_val), (x_1, x_1_val), (y_1, y_1_val), (z_1, z_1_val), 
                                        (x_2, x_2_val), (y_2, y_2_val), (z_2, z_2_val), (x_3, x_3_val), 
                                        (y_3, y_3_val), (z_3, z_3_val)])
                # Convert the sympy expression to a float
                A_num[i, j] = float(A_num[i, j])

    # Convert A_num to a numpy array if it's not already
    A_num = np.array(A_num, dtype=float)
    return A_num
    
    # print("A numerical is:\n", A)
    # print("A shape is:", A.shape)
    return A
   



def compute_matrix_inverse(A):
    A_inv = A.T@(pseudoInverseMat((A@A.T), 0.001))
    #print("A_inv is:\n", A_inv)
    return A_inv
    # print("A_inv is:\n", A_inv)

def print_thrust_symb():
    n_1_symb = get_thrustvector_symb_base_frame(1)
    n_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in n_1_symb])
    n_2_symb = get_thrustvector_symb_base_frame(2)
    n_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in n_2_symb])
    n_3_symb = get_thrustvector_symb_base_frame(3)
    n_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in n_3_symb])

    #print(n_3.shape)

    # might need to check forward kinematics
    # what's the difference between this and the one above?
    T_1_sym = get_thrustvector_symb_base_frame(1)
    T_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in T_1_sym])
    T_2_sym = get_thrustvector_symb_base_frame(2)
    T_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in T_2_sym])
    T_3_sym = get_thrustvector_symb_base_frame(3)
    T_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in T_3_sym])
    #print(T_1.shape)

    F_1 = k_f*omega_1*omega_1*T_1
    F_2 = k_f*omega_2*omega_2*T_2
    F_3 = k_f*omega_3*omega_3*T_3

    F = F_1 + F_2 + F_3 
    # print("F is: \n", simplify(F))
    # print("F shape is: \n", F.shape)

    
    F_1 = simplify(F_1)
    F_2 = simplify(F_2)
    F_3 = simplify(F_3)

    F = simplify(F)

    print("F_1 is: \n", F_1)
    print("F_2 is: \n", F_2)
    print("F_3 is: \n", F_3)
    #print("F is: \n", F)




def print_r_BG():

    r_BG_1_sym = get_T_BE_symbolic(1)
    r_BG_1_sub = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in r_BG_1_sym]))
    r_BG_1 = r_BG_1_sub[0:3, 3].reshape(3,1)

    r_BG_2_sym = get_T_BE_symbolic(2)
    r_BG_2_sub = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in r_BG_2_sym]))
    r_BG_2 = r_BG_2_sub[0:3, 3].reshape(3,1)

    r_BG_3_sym = get_T_BE_symbolic(3)
    r_BG_3_sub = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in r_BG_3_sym]))
    r_BG_3 = r_BG_3_sub[0:3, 3].reshape(3,1)

    print("r_BG_1 is:\n ", simplify(r_BG_1))
    print("r_BG_2 is:\n ", simplify(r_BG_2))
    print("r_BG_3 is:\n ", simplify(r_BG_3))

#UPDATED ALREADY with new stuff   
def forces_torques(): 


    r_BG_1_sym = get_T_BE_symbolic(1)
    r_BG_1_sub = (np.array([[element.subs({'q1': phi_1_1, 'q2': phi_1_2}) for element in row] for row in r_BG_1_sym]))
    r_1 = r_BG_1_sub[0:3, 3].reshape(3,1)

    r_BG_2_sym = get_T_BE_symbolic(2)
    r_BG_2_sub = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in r_BG_2_sym]))
    r_2 = r_BG_2_sub[0:3, 3].reshape(3,1)

    r_BG_3_sym = get_T_BE_symbolic(3)
    r_BG_3_sub = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in r_BG_3_sym]))
    r_3 = r_BG_3_sub[0:3, 3].reshape(3,1)
# 
    # r_1 = np.array([[-0.0375795*sin(phi_1_1)*cos(phi_1_2) - 0.0155659385671997*sin(phi_1_1) - 0.0531454385671997*sin(phi_1_2)*cos(phi_1_1) - 0.15073], 
    #                 [0.0181757399899823*sin(phi_1_1)*sin(phi_1_2) - 0.012852189*cos(phi_1_1)*cos(phi_1_2) - 0.00532355098998231*cos(phi_1_1) - 0.03531345615*cos(phi_1_2) + 0.0323957752284024], 
    #                 [-0.0499407686215976*sin(phi_1_1)*sin(phi_1_2) + 0.03531345615*cos(phi_1_1)*cos(phi_1_2) + 0.0146273124715976*cos(phi_1_1) - 0.012852189*cos(phi_1_2) - 0.0310279289899823]])
    
    # r_2 = np.array([[-0.0157416789036046*sin(phi_2_1)*sin(phi_2_2) + 0.01878975*sin(phi_2_1)*cos(phi_2_2) + 0.00778296928359987*sin(phi_2_1) + 0.0265727192835999*sin(phi_2_2)*cos(phi_2_1) + 0.0111310479*cos(phi_2_1)*cos(phi_2_2) + 0.00461063100360456*cos(phi_2_1) + 0.0305821971*cos(phi_2_2) + 0.0431619550059871], 
    #                 [-0.00908786999499116*sin(phi_2_1)*sin(phi_2_2) - 0.032543847*sin(phi_2_1)*cos(phi_2_2) - 0.013480102799195*sin(phi_2_1) - 0.046023949799195*sin(phi_2_2)*cos(phi_2_1) + 0.0064260945*cos(phi_2_1)*cos(phi_2_2) + 0.00266177549499116*cos(phi_2_1) + 0.0176548491*cos(phi_2_2) - 0.14632742386113], 
    #                 [-0.0499407686215976*sin(phi_2_1)*sin(phi_2_2) + 0.03531345615*cos(phi_2_1)*cos(phi_2_2) + 0.0146273124715976*cos(phi_2_1) - 0.012852189*cos(phi_2_2) - 0.0310279289899823]])

    # r_3 = np.array([[0.0157416789036046*sin(phi_3_1)*sin(phi_3_2) + 0.01878975*sin(phi_3_1)*cos(phi_3_2) + 0.00778296928359987*sin(phi_3_1) + 0.0265727192835999*sin(phi_3_2)*cos(phi_3_1) - 0.0111310479*cos(phi_3_1)*cos(phi_3_2) - 0.00461063100360456*cos(phi_3_1) - 0.0305821971*cos(phi_3_2) + 0.0978680449940129],
    #                 [-0.00908786999499116*sin(phi_3_1)*sin(phi_3_2) + 0.032543847*sin(phi_3_1)*cos(phi_3_2) + 0.013480102799195*sin(phi_3_1) + 0.046023949799195*sin(phi_3_2)*cos(phi_3_1) + 0.0064260945*cos(phi_3_1)*cos(phi_3_2) + 0.00266177549499116*cos(phi_3_1) + 0.0176548491*cos(phi_3_2) + 0.10999257613887],
    #                 [-0.0499407686215976*sin(phi_3_1)*sin(phi_3_2) + 0.03531345615*cos(phi_3_1)*cos(phi_3_2) + 0.0146273124715976*cos(phi_3_1) - 0.012852189*cos(phi_3_2) - 0.0310279289899823]])
    


    T_1_sym = get_thrustvector_symb_base_frame(1)
    T_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in T_1_sym])
    T_2_sym = get_thrustvector_symb_base_frame(2)
    T_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in T_2_sym])
    T_3_sym = get_thrustvector_symb_base_frame(3)
    T_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in T_3_sym])
    #print(T_1.shape)
    F_1 = k_f*omega_1*omega_1*T_1
    F_2 = k_f*omega_2*omega_2*T_2
    F_3 = k_f*omega_3*omega_3*T_3

    # F_1 = np.array([[-k_f*omega_1**2*(0.5*sin(phi_1_1)*cos(phi_1_2) + 0.5*sin(phi_1_1) + 0.707106781186547*sin(phi_1_2)*cos(phi_1_1))], 
    #                 [k_f*omega_1**2*(0.241830519165799*sin(phi_1_1)*sin(phi_1_2) - 0.171*cos(phi_1_1)*cos(phi_1_2) - 0.171*cos(phi_1_1) - 0.46985*cos(phi_1_2) + 0.46985)],
    #                 [k_f*omega_1**2*(-0.664468242280999*sin(phi_1_1)*sin(phi_1_2) + 0.46985*cos(phi_1_1)*cos(phi_1_2) + 0.46985*cos(phi_1_1) - 0.171*cos(phi_1_2) + 0.171)]])
    # F_2 = np.array([[k_f*omega_2**2*(-0.209445028587455*sin(phi_2_1)*sin(phi_2_2) + 0.25*sin(phi_2_1)*cos(phi_2_2) + 0.25*sin(phi_2_1) + 0.353553390593274*sin(phi_2_2)*cos(phi_2_1) + 0.1481*cos(phi_2_1)*cos(phi_2_2) + 0.1481*cos(phi_2_1) + 0.4069*cos(phi_2_2) - 0.4069)], 
    #                 [k_f*omega_2**2*(-0.1209152595829*sin(phi_2_1)*sin(phi_2_2) - 0.433*sin(phi_2_1)*cos(phi_2_2) - 0.433*sin(phi_2_1) - 0.61235447250755*sin(phi_2_2)*cos(phi_2_1) + 0.0855*cos(phi_2_1)*cos(phi_2_2) + 0.0855*cos(phi_2_1) + 0.2349*cos(phi_2_2) - 0.2349)], 
    #                 [k_f*omega_2**2*(-0.664468242280999*sin(phi_2_1)*sin(phi_2_2) + 0.46985*cos(phi_2_1)*cos(phi_2_2) + 0.46985*cos(phi_2_1) - 0.171*cos(phi_2_2) + 0.171)]])
    # F_3 = np.array([[k_f*omega_3**2*(0.209445028587455*sin(phi_3_1)*sin(phi_3_2) + 0.25*sin(phi_3_1)*cos(phi_3_2) + 0.25*sin(phi_3_1) + 0.353553390593274*sin(phi_3_2)*cos(phi_3_1) - 0.1481*cos(phi_3_1)*cos(phi_3_2) - 0.1481*cos(phi_3_1) - 0.4069*cos(phi_3_2) + 0.4069)], 
    #                 [k_f*omega_3**2*(-0.1209152595829*sin(phi_3_1)*sin(phi_3_2) + 0.433*sin(phi_3_1)*cos(phi_3_2) + 0.433*sin(phi_3_1) + 0.61235447250755*sin(phi_3_2)*cos(phi_3_1) + 0.0855*cos(phi_3_1)*cos(phi_3_2) + 0.0855*cos(phi_3_1) + 0.2349*cos(phi_3_2) - 0.2349)], 
    #                 [k_f*omega_3**2*(-0.664468242280999*sin(phi_3_1)*sin(phi_3_2) + 0.46985*cos(phi_3_1)*cos(phi_3_2) + 0.46985*cos(phi_3_1) - 0.171*cos(phi_3_2) + 0.171)]])


    tau_1 = vector_to_skew_symmetric(r_1)@F_1
    tau_2 = vector_to_skew_symmetric(r_2)@F_2
    tau_3 = vector_to_skew_symmetric(r_3)@F_3

    tau = tau_1 + tau_2 + tau_3 
    tau = np.squeeze(tau) 

    F = F_1 + F_2 + F_3
    F = np.squeeze(F)

    print("F is: \n")
    print("1st row\n", (F[0]))
    print("2nd row\n", (F[1]))
    print("3rd row\n", (F[2]))

    print("tau is: \n")
    print("1st row\n", (tau[0]))
    print("2nd row\n", (tau[1]))
    print("3rd row\n", (tau[2]))

    return tau
    

# def compute_forces(current_state, k_f):
    
#     omega_1 = current_state[0]
#     omega_2 = current_state[1]
#     omega_3 = current_state[2]
#     phi_1_1 = current_state[3]
#     phi_1_2 = current_state[4]
#     phi_2_1 = current_state[5]
#     phi_2_2 = current_state[6]
#     phi_3_1 = current_state[7]
#     phi_3_2 = current_state[8]
#     F = np.array([[k_f*(-omega_1**2*(0.5*sin(phi_1_1)*cos(phi_1_2) + 0.5*sin(phi_1_1) + 0.707106781186547*sin(phi_1_2)*cos(phi_1_1)) + omega_2**2*(-0.209445028587455*sin(phi_2_1)*sin(phi_2_2) + 0.25*sin(phi_2_1)*cos(phi_2_2) + 0.25*sin(phi_2_1) + 0.353553390593274*sin(phi_2_2)*cos(phi_2_1) + 0.1481*cos(phi_2_1)*cos(phi_2_2) + 0.1481*cos(phi_2_1) + 0.4069*cos(phi_2_2) - 0.4069) + omega_3**2*(0.209445028587455*sin(phi_3_1)*sin(phi_3_2) + 0.25*sin(phi_3_1)*cos(phi_3_2) + 0.25*sin(phi_3_1) + 0.353553390593274*sin(phi_3_2)*cos(phi_3_1) - 0.1481*cos(phi_3_1)*cos(phi_3_2) - 0.1481*cos(phi_3_1) - 0.4069*cos(phi_3_2) + 0.4069)), k_f*(-omega_1**2*(-0.241830519165799*sin(phi_1_1)*sin(phi_1_2) + 0.171*cos(phi_1_1)*cos(phi_1_2) + 0.171*cos(phi_1_1) + 0.46985*cos(phi_1_2) - 0.46985) - omega_2**2*(0.1209152595829*sin(phi_2_1)*sin(phi_2_2) + 0.433*sin(phi_2_1)*cos(phi_2_2) + 0.433*sin(phi_2_1) + 0.61235447250755*sin(phi_2_2)*cos(phi_2_1) - 0.0855*cos(phi_2_1)*cos(phi_2_2) - 0.0855*cos(phi_2_1) - 0.2349*cos(phi_2_2) + 0.2349) + omega_3**2*(-0.1209152595829*sin(phi_3_1)*sin(phi_3_2) + 0.433*sin(phi_3_1)*cos(phi_3_2) + 0.433*sin(phi_3_1) + 0.61235447250755*sin(phi_3_2)*cos(phi_3_1) + 0.0855*cos(phi_3_1)*cos(phi_3_2) + 0.0855*cos(phi_3_1) + 0.2349*cos(phi_3_2) - 0.2349)), k_f*(omega_1**2*(-0.664468242280999*sin(phi_1_1)*sin(phi_1_2) + 0.46985*cos(phi_1_1)*cos(phi_1_2) + 0.46985*cos(phi_1_1) - 0.171*cos(phi_1_2) + 0.171) + omega_2**2*(-0.664468242280999*sin(phi_2_1)*sin(phi_2_2) + 0.46985*cos(phi_2_1)*cos(phi_2_2) + 0.46985*cos(phi_2_1) - 0.171*cos(phi_2_2) + 0.171) + omega_3**2*(-0.664468242280999*sin(phi_3_1)*sin(phi_3_2) + 0.46985*cos(phi_3_1)*cos(phi_3_2) + 0.46985*cos(phi_3_1) - 0.171*cos(phi_3_2) + 0.171))]])
#     F = np.squeeze(F).reshape(3,1)
#     return F 



# def compute_torques(current_state, k_f):
    
    
#     omega_1 = current_state[0]
#     omega_2 = current_state[1]
#     omega_3 = current_state[2]
#     phi_1_1 = current_state[3]
#     phi_1_2 = current_state[4]
#     phi_2_1 = current_state[5]
#     phi_2_2 = current_state[6]
#     phi_3_1 = current_state[7]
#     phi_3_2 = current_state[8]

#     tau_total = np.array([[k_f*(-0.0404613614328003*omega_1**2*np.cos(phi_1_1) + 1.73472347597681e-18*omega_1**2*np.cos(phi_1_1 - 2*phi_1_2) + 0.011603633761573*omega_1**2*np.cos(phi_1_1 - phi_1_2) - 0.030051433761573*omega_1**2*np.cos(phi_1_1 + phi_1_2) - 5.20417042793042e-18*omega_1**2*np.cos(phi_1_1 + 2*phi_1_2) + 4.77048955893622e-18*omega_2**2*np.sin(2*phi_2_1) + 0.0134804982329429*omega_2**2*np.sin(phi_2_2) + 1.73472347597681e-18*omega_2**2*np.sin(2*phi_2_2) + 4.33680868994202e-19*omega_2**2*np.sin(2*phi_2_1 - 2*phi_2_2) - 2.60208521396521e-18*omega_2**2*np.sin(2*phi_2_1 - phi_2_2) + 1.12757025938492e-17*omega_2**2*np.sin(2*phi_2_1 + phi_2_2) + 2.16840434497101e-18*omega_2**2*np.sin(2*phi_2_1 + 2*phi_2_2) - 0.1203921192836*omega_2**2*np.cos(phi_2_1) - 4.33680868994202e-19*omega_2**2*np.cos(phi_2_1 - 2*phi_2_2) + 0.0233221185886531*omega_2**2*np.cos(phi_2_1 - phi_2_2) - 0.154721018588653*omega_2**2*np.cos(phi_2_1 + phi_2_2) + 4.33680868994202e-19*omega_2**2*np.cos(phi_2_1 + 2*phi_2_2) - 4.77048955893622e-18*omega_3**2*np.sin(2*phi_3_1) - 0.0134804982329429*omega_3**2*np.sin(phi_3_2) - 1.73472347597681e-18*omega_3**2*np.sin(2*phi_3_2) - 4.33680868994202e-19*omega_3**2*np.sin(2*phi_3_1 - 2*phi_3_2) + 2.60208521396521e-18*omega_3**2*np.sin(2*phi_3_1 - phi_3_2) - 1.12757025938492e-17*omega_3**2*np.sin(2*phi_3_1 + phi_3_2) - 2.16840434497101e-18*omega_3**2*np.sin(2*phi_3_1 + 2*phi_3_2) + 0.1608535307164*omega_3**2*np.cos(phi_3_1) - 4.33680868994202e-19*omega_3**2*np.cos(phi_3_1 - 2*phi_3_2) - 0.0349257627055651*omega_3**2*np.cos(phi_3_1 - phi_3_2) + 0.184772512705565*omega_3**2*np.cos(phi_3_1 + phi_3_2) + 4.33680868994202e-19*omega_3**2*np.cos(phi_3_1 + 2*phi_3_2))],
#                         [k_f*(1.73472347597681e-18*omega_1**2*np.sin(2*phi_1_1) - 0.0155659385671997*omega_1**2*np.sin(phi_1_2) - 2.60208521396521e-18*omega_1**2*np.sin(2*phi_1_2) - 4.33680868994202e-19*omega_1**2*np.sin(2*phi_1_1 - 2*phi_1_2) + 4.33680868994202e-19*omega_1**2*np.sin(2*phi_1_1 - phi_1_2) - 1.30104260698261e-18*omega_1**2*np.sin(2*phi_1_1 + phi_1_2) + 4.33680868994202e-19*omega_1**2*np.sin(2*phi_1_1 + 2*phi_1_2) - 0.16237725*omega_1**2*np.cos(phi_1_1) + 0.0336294295854232*omega_1**2*np.cos(phi_1_1 - phi_1_2) - 0.196006679585423*omega_1**2*np.cos(phi_1_1 + phi_1_2) + 3.25260651745651e-18*omega_2**2*np.sin(2*phi_2_1) + 0.00778296928359987*omega_2**2*np.sin(phi_2_2) + 2.60208521396521e-18*omega_2**2*np.sin(2*phi_2_2) - 4.98732999343332e-18*omega_2**2*np.sin(2*phi_2_1 - phi_2_2) + 1.01915004213637e-17*omega_2**2*np.sin(2*phi_2_1 + phi_2_2) + 1.73472347597681e-18*omega_2**2*np.sin(2*phi_2_1 + 2*phi_2_2) + 0.20840074989446*omega_2**2*np.cos(phi_2_1) + 8.67361737988403e-19*omega_2**2*np.cos(phi_2_1 - 2*phi_2_2) - 0.0403693059096886*omega_2**2*np.cos(phi_2_1 - phi_2_2) + 0.267834359232723*omega_2**2*np.cos(phi_2_1 + phi_2_2) + 8.67361737988403e-19*omega_2**2*np.cos(phi_2_1 + 2*phi_2_2) + 3.25260651745651e-18*omega_3**2*np.sin(2*phi_3_1) + 0.00778296928359987*omega_3**2*np.sin(phi_3_2) + 2.60208521396521e-18*omega_3**2*np.sin(2*phi_3_2) - 4.98732999343332e-18*omega_3**2*np.sin(2*phi_3_1 - phi_3_2) + 1.01915004213637e-17*omega_3**2*np.sin(2*phi_3_1 + phi_3_2) + 1.73472347597681e-18*omega_3**2*np.sin(2*phi_3_1 + 2*phi_3_2) - 0.0459025498944602*omega_3**2*np.cos(phi_3_1) - 8.67361737988403e-19*omega_3**2*np.cos(phi_3_1 - 2*phi_3_2) + 0.00671482675908077*omega_3**2*np.cos(phi_3_1 - phi_3_2) - 0.0716816800821154*omega_3**2*np.cos(phi_3_1 + phi_3_2) - 8.67361737988403e-19*omega_3**2*np.cos(phi_3_1 + 2*phi_3_2))],
#                         [k_f*(6.93889390390723e-18*omega_1**2*np.sin(phi_1_1)*np.sin(phi_1_2)**2 - 0.0184478*omega_1**2*np.sin(phi_1_1)*np.cos(phi_1_2) - 0.0404613614328003*omega_1**2*np.sin(phi_1_1) - 1.21430643318376e-17*omega_1**2*np.sin(phi_1_2)*np.cos(phi_1_1)*np.cos(phi_1_2) - 0.0416550675231461*omega_1**2*np.sin(phi_1_2)*np.cos(phi_1_1) - 0.16237725*omega_1**2*np.cos(phi_1_2) + 0.16237725*omega_1**2 - 5.20417042793042e-18*omega_2**2*np.sin(phi_2_1)**2*np.sin(phi_2_2)**2 + 1.73472347597681e-18*omega_2**2*np.sin(phi_2_1)**2*np.cos(phi_2_2) + 1.30104260698261e-18*omega_2**2*np.sin(phi_2_1)**2 + 1.38777878078145e-17*omega_2**2*np.sin(phi_2_1)*np.sin(phi_2_2)*np.cos(phi_2_1) + 0.26268996465093*omega_2**2*np.sin(phi_2_1)*np.cos(phi_2_2) + 0.24067640321813*omega_2**2*np.sin(phi_2_1) + 8.67361737988404e-18*omega_2**2*np.sin(phi_2_2)**2 - 8.67361737988404e-18*omega_2**2*np.sin(phi_2_2)*np.cos(phi_2_1)*np.cos(phi_2_2) + 0.355933772141454*omega_2**2*np.sin(phi_2_2)*np.cos(phi_2_1) + 6.22587678136277e-5*omega_2**2*np.cos(phi_2_2) - 6.22587678136294e-5*omega_2**2 + 5.20417042793042e-18*omega_3**2*np.sin(phi_3_1)**2*np.sin(phi_3_2)**2 - 1.73472347597681e-18*omega_3**2*np.sin(phi_3_1)**2*np.cos(phi_3_2) - 1.30104260698261e-18*omega_3**2*np.sin(phi_3_1)**2 - 1.38777878078145e-17*omega_3**2*np.sin(phi_3_1)*np.sin(phi_3_2)*np.cos(phi_3_1) - 0.0186604296183146*omega_3**2*np.sin(phi_3_1)*np.cos(phi_3_2) - 0.0406739910511148*omega_3**2*np.sin(phi_3_1) - 8.67361737988404e-18*omega_3**2*np.sin(phi_3_2)**2 - 8.67361737988404e-18*omega_3**2*np.sin(phi_3_2)*np.cos(phi_3_1)*np.cos(phi_3_2) - 0.0419557712131289*omega_3**2*np.sin(phi_3_2)*np.cos(phi_3_1) + 0.162254518836053*omega_3**2*np.cos(phi_3_2) - 0.162254518836053*omega_3**2)]])
    
    tau_total = tau_total.reshape(3,1)
    return tau_total

def print_numerical_forces_torques():
    #---------------------------these are the really new ones
    
    k_f = 0.0001
    omega_1 = 100
    omega_2 = 100
    omega_3 = 100
    phi_1_1 = np.pi/2
    phi_1_2 = np.pi
    phi_2_1 = np.pi/2
    phi_2_2 = np.pi
    phi_3_1 = np.pi/2   
    phi_3_2 = np.pi
    

    F_1 = np.array([-k_f*omega_1**2*(0.5*sin(phi_1_1)*cos(phi_1_2) + 0.5*sin(phi_1_1) + 0.707106781186547*sin(phi_1_2)*cos(phi_1_1)), k_f*omega_1**2*(0.241830519165799*sin(phi_1_1)*sin(phi_1_2) - 0.171*cos(phi_1_1)*cos(phi_1_2) - 0.171*cos(phi_1_1) - 0.46985*cos(phi_1_2) + 0.46985), k_f*omega_1**2*(-0.664468242280999*sin(phi_1_1)*sin(phi_1_2) + 0.46985*cos(phi_1_1)*cos(phi_1_2) + 0.46985*cos(phi_1_1) - 0.171*cos(phi_1_2) + 0.171)])
    F_2 = np.array([k_f*omega_2**2*(-0.209445028587455*sin(phi_2_1)*sin(phi_2_2) + 0.25*sin(phi_2_1)*cos(phi_2_2) + 0.25*sin(phi_2_1) + 0.353553390593274*sin(phi_2_2)*cos(phi_2_1) + 0.1481*cos(phi_2_1)*cos(phi_2_2) + 0.1481*cos(phi_2_1) + 0.4069*cos(phi_2_2) - 0.4069), k_f*omega_2**2*(-0.1209152595829*sin(phi_2_1)*sin(phi_2_2) - 0.433*sin(phi_2_1)*cos(phi_2_2) - 0.433*sin(phi_2_1) - 0.61235447250755*sin(phi_2_2)*cos(phi_2_1) + 0.0855*cos(phi_2_1)*cos(phi_2_2) + 0.0855*cos(phi_2_1) + 0.2349*cos(phi_2_2) - 0.2349), k_f*omega_2**2*(-0.664468242280999*sin(phi_2_1)*sin(phi_2_2) + 0.46985*cos(phi_2_1)*cos(phi_2_2) + 0.46985*cos(phi_2_1) - 0.171*cos(phi_2_2) + 0.171)])
    F_3 = np.array([k_f*omega_3**2*(0.209445028587455*sin(phi_3_1)*sin(phi_3_2) + 0.25*sin(phi_3_1)*cos(phi_3_2) + 0.25*sin(phi_3_1) + 0.353553390593274*sin(phi_3_2)*cos(phi_3_1) - 0.1481*cos(phi_3_1)*cos(phi_3_2) - 0.1481*cos(phi_3_1) - 0.4069*cos(phi_3_2) + 0.4069), k_f*omega_3**2*(-0.1209152595829*sin(phi_3_1)*sin(phi_3_2) + 0.433*sin(phi_3_1)*cos(phi_3_2) + 0.433*sin(phi_3_1) + 0.61235447250755*sin(phi_3_2)*cos(phi_3_1) + 0.0855*cos(phi_3_1)*cos(phi_3_2) + 0.0855*cos(phi_3_1) + 0.2349*cos(phi_3_2) - 0.2349), k_f*omega_3**2*(-0.664468242280999*sin(phi_3_1)*sin(phi_3_2) + 0.46985*cos(phi_3_1)*cos(phi_3_2) + 0.46985*cos(phi_3_1) - 0.171*cos(phi_3_2) + 0.171)])
    
    r_BG_1 = np.array([0.020337878845444*sin(phi_1_1)*cos(phi_1_2) + 0.0778620870316996*sin(phi_1_1) + 0.0287621040931278*sin(phi_1_2)*cos(phi_1_1) + 0.15073, -0.00983663959984971*sin(phi_1_1)*sin(phi_1_2) + 0.00695555456514185*cos(phi_1_1)*cos(phi_1_2) + 0.0266288337648413*cos(phi_1_1) + 0.0191115047510637*cos(phi_1_2) - 0.229040012685816, 0.0270277492163122*sin(phi_1_1)*sin(phi_1_2) - 0.0191115047510637*cos(phi_1_1)*cos(phi_1_2) - 0.0731670031836881*cos(phi_1_1) + 0.00695555456514185*cos(phi_1_2) - 0.040539942895125])
    r_BG_2 = np.array([0.00851933523238445*sin(phi_2_1)*sin(phi_2_2) - 0.010168939422722*sin(phi_2_1)*cos(phi_2_2) - 0.0389310435158498*sin(phi_2_1) - 0.0143810520465639*sin(phi_2_2)*cos(phi_2_1) - 0.00602407971402052*cos(phi_2_1)*cos(phi_2_2) - 0.0230627501787894*cos(phi_2_1) - 0.0165509658044223*cos(phi_2_2) + 0.127136098035242, 0.00491831979992485*sin(phi_2_1)*sin(phi_2_2) + 0.0176126030801545*sin(phi_2_1)*cos(phi_2_2) + 0.0674285673694518*sin(phi_2_1) + 0.0249079821446487*sin(phi_2_2)*cos(phi_2_1) - 0.00347777728257093*cos(phi_2_1)*cos(phi_2_2) - 0.0133144168824206*cos(phi_2_1) - 0.0095547354815896*cos(phi_2_2) + 0.244639079450672, 0.0270277492163122*sin(phi_2_1)*sin(phi_2_2) - 0.0191115047510637*cos(phi_2_1)*cos(phi_2_2) - 0.0731670031836881*cos(phi_2_1) + 0.00695555456514185*cos(phi_2_2) - 0.040539942895125])
    r_BG_3 = np.array([-0.00851933523238445*sin(phi_3_1)*sin(phi_3_2) - 0.010168939422722*sin(phi_3_1)*cos(phi_3_2) - 0.0389310435158498*sin(phi_3_1) - 0.0143810520465639*sin(phi_3_2)*cos(phi_3_1) + 0.00602407971402052*cos(phi_3_1)*cos(phi_3_2) + 0.0230627501787894*cos(phi_3_1) + 0.0165509658044223*cos(phi_3_2) - 0.268166098035242, 0.00491831979992485*sin(phi_3_1)*sin(phi_3_2) - 0.0176126030801545*sin(phi_3_1)*cos(phi_3_2) - 0.0674285673694518*sin(phi_3_1) - 0.0249079821446487*sin(phi_3_2)*cos(phi_3_1) - 0.00347777728257093*cos(phi_3_1)*cos(phi_3_2) - 0.0133144168824206*cos(phi_3_1) - 0.0095547354815896*cos(phi_3_2) - 0.0116809205493283, 0.0270277492163122*sin(phi_3_1)*sin(phi_3_2) - 0.0191115047510637*cos(phi_3_1)*cos(phi_3_2) - 0.0731670031836881*cos(phi_3_1) + 0.00695555456514185*cos(phi_3_2) - 0.040539942895125])
    
    
    tau_1 = vector_to_skew_symmetric(r_BG_1)@F_1
    tau_2 = vector_to_skew_symmetric(r_BG_2)@F_2
    tau_3 = vector_to_skew_symmetric(r_BG_3)@F_3
    
    # print("F_1\n", F_1)
    # print("F_2\n", F_2)
    # print("F_3\n", F_3)
    F = F_1 + F_2 + F_3
    print("F\n", F)
    
    # print("tau_1\n", tau_1)
    # print("tau_2\n", tau_2)
    # print("tau_3\n", tau_3)
    tau = tau_1 + tau_2 + tau_3
    print("tau\n", tau)

def main():
    # print(get_T_BE_symbolic(1)
    A = simplify(allocation_matrix())
    print("allocation_matrix.row(0) << ", A[0, :])
    print("allocation_matrix.row(1) << ", A[1, :])
    print("allocation_matrix.row(2) << ", A[2, :])
    print("allocation_matrix.row(3) << ", A[3, :])
    print("allocation_matrix.row(4) << ", A[4, :])
    print("allocation_matrix.row(5) << ", A[5, :])
    



    
if __name__ == "__main__":
    main()










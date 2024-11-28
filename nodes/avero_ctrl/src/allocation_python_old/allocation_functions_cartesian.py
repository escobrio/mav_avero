import numpy as np
import sympy as sp
from sympy import N
import sys
import os
import copy
sys.path.append(f"{os.environ['HOME']}/catkin_ws/src/mav_avero/nodes/dynamixel_pkg_avero/src/nodes/in_kin_new_angles")

# Now you can import your module
from helper_functions_symb import getJacobian_angles_sym, get_thrustvector_symbolic, get_thrustvector_symb_base_frame, getangles_sym, get_T_BE_symbolic, getJacobian_non_minimal_sym, pseudoInverseMat, nozzle_i_to_body 
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
    r_BG_1 = r_BG_1_sub[0:3, 3].reshape(3,1)

    r_BG_2_sym = get_T_BE_symbolic(2)
    r_BG_2_sub = (np.array([[element.subs({'q1': phi_2_1, 'q2': phi_2_2}) for element in row] for row in r_BG_2_sym]))
    r_BG_2 = r_BG_2_sub[0:3, 3].reshape(3,1)

    r_BG_3_sym = get_T_BE_symbolic(3)
    r_BG_3_sub = (np.array([[element.subs({'q1': phi_3_1, 'q2': phi_3_2}) for element in row] for row in r_BG_3_sym]))
    r_BG_3 = r_BG_3_sub[0:3, 3].reshape(3,1)
    #print(r_BG_1.shape)

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
    A_simplified = simplify(A)
    # np.savetxt('A_simplified.txt', A, delimiter=',')
    # print("A[0,:] is:", simplify(A[0,:]))
    # print("A[1,:] is:", simplify(A[1,:]))
    # print("A[2,:] is:", simplify(A[2,:]))
    # print("A[3,:] is:", simplify(A[3,:]))
    # print("A[4,:] is:", simplify(A[4,:]))
    # print("A[5,:] is:", simplify(A[5,:]))
    return A 





    
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

    
    print("F_1 is: \n", F_1)
    print("F_2 is: \n", F_2)
    print("F_3 is: \n", F_3)




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

    print("r_BG_1 is:\n ", r_BG_1)
    print("r_BG_2 is:\n ", r_BG_2)
    print("r_BG_3 is:\n ", r_BG_3)


def symb_torques(): 
    r_1 = np.array([[3.25421956153017e-18*sin(phi_1_1)*sin(phi_1_2) + 0.01878975*sin(phi_1_1)*cos(phi_1_2) + 0.00778296928359987*sin(phi_1_1) + 0.0265727192835999*sin(phi_1_2)*cos(phi_1_1) - 2.3010807194279e-18*cos(phi_1_1)*cos(phi_1_2) - 9.53138842102275e-19*cos(phi_1_1) - 0.0325448016615173*cos(phi_1_2) - 0.0785701015559775],
                    [-0.0325448016615173*sin(phi_1_1)*cos(phi_1_2) - 0.0134804982329429*sin(phi_1_1) - 0.0460252998944602*sin(phi_1_2)*cos(phi_1_1) - 0.01878975*cos(phi_1_2) - 0.0453624692835998],
                    [0.0531454385671997*sin(phi_1_1)*sin(phi_1_2) - 1.15054035971395e-18*sin(phi_1_1)*cos(phi_1_2) - 4.76569421051137e-19*sin(phi_1_1) - 1.62710978076509e-18*sin(phi_1_2)*cos(phi_1_1) - 0.0375795*cos(phi_1_1)*cos(phi_1_2) - 0.0155659385671997*cos(phi_1_1) + 1.99279435918313e-18*cos(phi_1_2) + 4.81103116896052e-18]])
    
    r_2 = np.array([[3.25421956153017e-18*sin(phi_2_1)*sin(phi_2_2) + 0.01878975*sin(phi_2_1)*cos(phi_2_2) + 0.00778296928359987*sin(phi_2_1) + 0.0265727192835999*sin(phi_2_2)*cos(phi_2_1) - 2.3010807194279e-18*cos(phi_2_1)*cos(phi_2_2) - 9.53138842102275e-19*cos(phi_2_1) + 0.0325448016615173*cos(phi_2_2) + 0.0785701015559775],
                    [0.0325448016615173*sin(phi_2_1)*cos(phi_2_2) + 0.0134804982329429*sin(phi_2_1) + 0.0460252998944602*sin(phi_2_2)*cos(phi_2_1) - 0.01878975*cos(phi_2_2) - 0.0453624692835998],
                    [0.0531454385671997*sin(phi_2_1)*sin(phi_2_2) - 1.15054035971395e-18*sin(phi_2_1)*cos(phi_2_2) - 4.76569421051137e-19*sin(phi_2_1) - 1.62710978076509e-18*sin(phi_2_2)*cos(phi_2_1) - 0.0375795*cos(phi_2_1)*cos(phi_2_2) - 0.0155659385671997*cos(phi_2_1) - 1.99279435918313e-18*cos(phi_2_2) - 4.81103116896052e-18]])

    r_3 = np.array([[3.25421956153017e-18*sin(phi_3_1)*sin(phi_3_2) - 0.0325448016615173*sin(phi_3_1)*cos(phi_3_2) - 0.0134804982329429*sin(phi_3_1) - 0.0460252998944602*sin(phi_3_2)*cos(phi_3_1) - 2.3010807194279e-18*cos(phi_3_1)*cos(phi_3_2) - 9.53138842102275e-19*cos(phi_3_1) + 0.01878975*cos(phi_3_2) + 0.0453624692835999],
                    [0.01878975*sin(phi_3_1)*cos(phi_3_2) + 0.00778296928359987*sin(phi_3_1) + 0.0265727192835999*sin(phi_3_2)*cos(phi_3_1) + 0.0325448016615173*cos(phi_3_2) + 0.0785701015559775],
                    [0.0531454385671997*sin(phi_3_1)*sin(phi_3_2) + 1.99279435918313e-18*sin(phi_3_1)*cos(phi_3_2) + 8.25442450594255e-19*sin(phi_3_1) + 2.81823680977739e-18*sin(phi_3_2)*cos(phi_3_1) - 0.0375795*cos(phi_3_1)*cos(phi_3_2) - 0.0155659385671997*cos(phi_3_1) - 1.15054035971395e-18*cos(phi_3_2) - 2.77765014047903e-18]])
    
    F_1 = np.array([[k_f*omega_1**2*(-4.32978028117747e-17*sin(phi_1_1)*sin(phi_1_2) - 0.25*sin(phi_1_1)*cos(phi_1_2) - 0.25*sin(phi_1_1) - 0.353553390593274*sin(phi_1_2)*cos(phi_1_1) + 3.06161699786838e-17*cos(phi_1_1)*cos(phi_1_2) + 3.06161699786838e-17*cos(phi_1_1) + 0.433012701892219*cos(phi_1_2) - 0.433012701892219)],
                    [k_f*omega_1**2*(0.433012701892219*sin(phi_1_1)*cos(phi_1_2) + 0.433012701892219*sin(phi_1_1) + 0.612372435695794*sin(phi_1_2)*cos(phi_1_1) + 0.25*cos(phi_1_2) - 0.25)],
                    [k_f*omega_1**2*(-0.707106781186547*sin(phi_1_1)*sin(phi_1_2) + 1.53080849893419e-17*sin(phi_1_1)*cos(phi_1_2) + 1.53080849893419e-17*sin(phi_1_1) + 2.16489014058873e-17*sin(phi_1_2)*cos(phi_1_1) + 0.5*cos(phi_1_1)*cos(phi_1_2) + 0.5*cos(phi_1_1) - 2.65143809681227e-17*cos(phi_1_2) + 2.65143809681227e-17)]])

    F_2 = np.array([[k_f*omega_2**2*(-4.32978028117747e-17*sin(phi_2_1)*sin(phi_2_2) - 0.25*sin(phi_2_1)*cos(phi_2_2) - 0.25*sin(phi_2_1) - 0.353553390593274*sin(phi_2_2)*cos(phi_2_1) + 3.06161699786838e-17*cos(phi_2_1)*cos(phi_2_2) + 3.06161699786838e-17*cos(phi_2_1) - 0.433012701892219*cos(phi_2_2) + 0.433012701892219)],
                    [k_f*omega_2**2*(-0.433012701892219*sin(phi_2_1)*cos(phi_2_2) - 0.433012701892219*sin(phi_2_1) - 0.612372435695794*sin(phi_2_2)*cos(phi_2_1) + 0.25*cos(phi_2_2) - 0.25)],
                    [k_f*omega_2**2*(-0.707106781186547*sin(phi_2_1)*sin(phi_2_2) + 1.53080849893419e-17*sin(phi_2_1)*cos(phi_2_2) + 1.53080849893419e-17*sin(phi_2_1) + 2.16489014058873e-17*sin(phi_2_2)*cos(phi_2_1) + 0.5*cos(phi_2_1)*cos(phi_2_2) + 0.5*cos(phi_2_1) + 2.65143809681227e-17*cos(phi_2_2) - 2.65143809681227e-17)]])

    F_3 = np.array([[k_f*omega_3**2*(-4.32978028117747e-17*sin(phi_3_1)*sin(phi_3_2) + 0.433012701892219*sin(phi_3_1)*cos(phi_3_2) + 0.433012701892219*sin(phi_3_1) + 0.612372435695794*sin(phi_3_2)*cos(phi_3_1) + 3.06161699786838e-17*cos(phi_3_1)*cos(phi_3_2) + 3.06161699786838e-17*cos(phi_3_1) - 0.25*cos(phi_3_2) + 0.25)],
                    [k_f*omega_3**2*(-0.25*sin(phi_3_1)*cos(phi_3_2) - 0.25*sin(phi_3_1) - 0.353553390593274*sin(phi_3_2)*cos(phi_3_1) - 0.433012701892219*cos(phi_3_2) + 0.433012701892219)],
                    [k_f*omega_3**2*(-0.707106781186547*sin(phi_3_1)*sin(phi_3_2) - 2.65143809681227e-17*sin(phi_3_1)*cos(phi_3_2) - 2.65143809681227e-17*sin(phi_3_1) - 3.74969971630462e-17*sin(phi_3_2)*cos(phi_3_1) + 0.5*cos(phi_3_1)*cos(phi_3_2) + 0.5*cos(phi_3_1) + 1.53080849893419e-17*cos(phi_3_2) - 1.53080849893419e-17)]])


    tau_1 = vector_to_skew_symmetric(r_1)@F_1
    tau_2 = vector_to_skew_symmetric(r_2)@F_2
    tau_3 = vector_to_skew_symmetric(r_3)@F_3

    tau = tau_1 + tau_2 + tau_3 
    tau = np.squeeze(tau) 
    # print("1st row\n", simplify(tau[0]))
    # print("2nd row\n", simplify(tau[1]))
    print("3rd row\n", simplify(tau[2]))

    return tau
    

def compute_forces(current_state, k_f):
    
    omega_1 = current_state[0]
    omega_2 = current_state[1]
    omega_3 = current_state[2]
    phi_1_1 = current_state[3]
    phi_1_2 = current_state[4]
    phi_2_1 = current_state[5]
    phi_2_2 = current_state[6]
    phi_3_1 = current_state[7]
    phi_3_2 = current_state[8]
    F = np.array([[k_f*omega_1**2*(-4.32978028117747e-17*np.sin(phi_1_1)*np.sin(phi_1_2) - 0.25*np.sin(phi_1_1)*np.cos(phi_1_2) - 0.25*np.sin(phi_1_1) - 0.353553390593274*np.sin(phi_1_2)*np.cos(phi_1_1) + 3.06161699786838e-17*np.cos(phi_1_1)*np.cos(phi_1_2) + 3.06161699786838e-17*np.cos(phi_1_1) + 0.433012701892219*np.cos(phi_1_2) - 0.433012701892219) + k_f*omega_2**2*(-4.32978028117747e-17*np.sin(phi_2_1)*np.sin(phi_2_2) - 0.25*np.sin(phi_2_1)*np.cos(phi_2_2) - 0.25*np.sin(phi_2_1) - 0.353553390593274*np.sin(phi_2_2)*np.cos(phi_2_1) + 3.06161699786838e-17*np.cos(phi_2_1)*np.cos(phi_2_2) + 3.06161699786838e-17*np.cos(phi_2_1) - 0.433012701892219*np.cos(phi_2_2) + 0.433012701892219) + k_f*omega_3**2*(-4.32978028117747e-17*np.sin(phi_3_1)*np.sin(phi_3_2) + 0.433012701892219*np.sin(phi_3_1)*np.cos(phi_3_2) + 0.433012701892219*np.sin(phi_3_1) + 0.612372435695794*np.sin(phi_3_2)*np.cos(phi_3_1) + 3.06161699786838e-17*np.cos(phi_3_1)*np.cos(phi_3_2) + 3.06161699786838e-17*np.cos(phi_3_1) - 0.25*np.cos(phi_3_2) + 0.25)],
        [k_f*omega_1**2*(0.433012701892219*np.sin(phi_1_1)*np.cos(phi_1_2) + 0.433012701892219*np.sin(phi_1_1) + 0.612372435695794*np.sin(phi_1_2)*np.cos(phi_1_1) + 0.25*np.cos(phi_1_2) - 0.25) + k_f*omega_2**2*(-0.433012701892219*np.sin(phi_2_1)*np.cos(phi_2_2) - 0.433012701892219*np.sin(phi_2_1) - 0.612372435695794*np.sin(phi_2_2)*np.cos(phi_2_1) + 0.25*np.cos(phi_2_2) - 0.25) + k_f*omega_3**2*(-0.25*np.sin(phi_3_1)*np.cos(phi_3_2) - 0.25*np.sin(phi_3_1) - 0.353553390593274*np.sin(phi_3_2)*np.cos(phi_3_1) - 0.433012701892219*np.cos(phi_3_2) + 0.433012701892219)],
        [k_f*omega_1**2*(-0.707106781186547*np.sin(phi_1_1)*np.sin(phi_1_2) + 1.53080849893419e-17*np.sin(phi_1_1)*np.cos(phi_1_2) + 1.53080849893419e-17*np.sin(phi_1_1) + 2.16489014058873e-17*np.sin(phi_1_2)*np.cos(phi_1_1) + 0.5*np.cos(phi_1_1)*np.cos(phi_1_2) + 0.5*np.cos(phi_1_1) - 2.65143809681227e-17*np.cos(phi_1_2) + 2.65143809681227e-17) + k_f*omega_2**2*(-0.707106781186547*np.sin(phi_2_1)*np.sin(phi_2_2) + 1.53080849893419e-17*np.sin(phi_2_1)*np.cos(phi_2_2) + 1.53080849893419e-17*np.sin(phi_2_1) + 2.16489014058873e-17*np.sin(phi_2_2)*np.cos(phi_2_1) + 0.5*np.cos(phi_2_1)*np.cos(phi_2_2) + 0.5*np.cos(phi_2_1) + 2.65143809681227e-17*np.cos(phi_2_2) - 2.65143809681227e-17) + k_f*omega_3**2*(-0.707106781186547*np.sin(phi_3_1)*np.sin(phi_3_2) - 2.65143809681227e-17*np.sin(phi_3_1)*np.cos(phi_3_2) - 2.65143809681227e-17*np.sin(phi_3_1) - 3.74969971630462e-17*np.sin(phi_3_2)*np.cos(phi_3_1) + 0.5*np.cos(phi_3_1)*np.cos(phi_3_2) + 0.5*np.cos(phi_3_1) + 1.53080849893419e-17*np.cos(phi_3_2) - 1.53080849893419e-17)]])
    # print('Fshape is', F.shape)

    F = np.squeeze(F).reshape(3,1)
    return F 



def compute_torques(current_state, k_f):
    
    
    omega_1 = current_state[0]
    omega_2 = current_state[1]
    omega_3 = current_state[2]
    phi_1_1 = current_state[3]
    phi_1_2 = current_state[4]
    phi_2_1 = current_state[5]
    phi_2_2 = current_state[6]
    phi_3_1 = current_state[7]
    phi_3_2 = current_state[8]

    r_1 = np.array([[3.25421956153017e-18*np.sin(phi_1_1)*np.sin(phi_1_2) + 0.01878975*np.sin(phi_1_1)*np.cos(phi_1_2) + 0.00778296928359987*np.sin(phi_1_1) + 0.0265727192835999*np.sin(phi_1_2)*np.cos(phi_1_1) - 2.3010807194279e-18*np.cos(phi_1_1)*np.cos(phi_1_2) - 9.53138842102275e-19*np.cos(phi_1_1) - 0.0325448016615173*np.cos(phi_1_2) - 0.0785701015559775],
                    [-0.0325448016615173*np.sin(phi_1_1)*np.cos(phi_1_2) - 0.0134804982329429*np.sin(phi_1_1) - 0.0460252998944602*np.sin(phi_1_2)*np.cos(phi_1_1) - 0.01878975*np.cos(phi_1_2) - 0.0453624692835998],
                    [0.0531454385671997*np.sin(phi_1_1)*np.sin(phi_1_2) - 1.15054035971395e-18*np.sin(phi_1_1)*np.cos(phi_1_2) - 4.76569421051137e-19*np.sin(phi_1_1) - 1.62710978076509e-18*np.sin(phi_1_2)*np.cos(phi_1_1) - 0.0375795*np.cos(phi_1_1)*np.cos(phi_1_2) - 0.0155659385671997*np.cos(phi_1_1) + 1.99279435918313e-18*np.cos(phi_1_2) + 4.81103116896052e-18]])
    
    r_2 = np.array([[3.25421956153017e-18*np.sin(phi_2_1)*np.sin(phi_2_2) + 0.01878975*np.sin(phi_2_1)*np.cos(phi_2_2) + 0.00778296928359987*np.sin(phi_2_1) + 0.0265727192835999*np.sin(phi_2_2)*np.cos(phi_2_1) - 2.3010807194279e-18*np.cos(phi_2_1)*np.cos(phi_2_2) - 9.53138842102275e-19*np.cos(phi_2_1) + 0.0325448016615173*np.cos(phi_2_2) + 0.0785701015559775],
                    [0.0325448016615173*np.sin(phi_2_1)*np.cos(phi_2_2) + 0.0134804982329429*np.sin(phi_2_1) + 0.0460252998944602*np.sin(phi_2_2)*np.cos(phi_2_1) - 0.01878975*np.cos(phi_2_2) - 0.0453624692835998],
                    [0.0531454385671997*np.sin(phi_2_1)*np.sin(phi_2_2) - 1.15054035971395e-18*np.sin(phi_2_1)*np.cos(phi_2_2) - 4.76569421051137e-19*np.sin(phi_2_1) - 1.62710978076509e-18*np.sin(phi_2_2)*np.cos(phi_2_1) - 0.0375795*np.cos(phi_2_1)*np.cos(phi_2_2) - 0.0155659385671997*np.cos(phi_2_1) - 1.99279435918313e-18*np.cos(phi_2_2) - 4.81103116896052e-18]])

    r_3 = np.array([[3.25421956153017e-18*np.sin(phi_3_1)*np.sin(phi_3_2) - 0.0325448016615173*np.sin(phi_3_1)*np.cos(phi_3_2) - 0.0134804982329429*np.sin(phi_3_1) - 0.0460252998944602*np.sin(phi_3_2)*np.cos(phi_3_1) - 2.3010807194279e-18*np.cos(phi_3_1)*np.cos(phi_3_2) - 9.53138842102275e-19*np.cos(phi_3_1) + 0.01878975*np.cos(phi_3_2) + 0.0453624692835999],
                    [0.01878975*np.sin(phi_3_1)*np.cos(phi_3_2) + 0.00778296928359987*np.sin(phi_3_1) + 0.0265727192835999*np.sin(phi_3_2)*np.cos(phi_3_1) + 0.0325448016615173*np.cos(phi_3_2) + 0.0785701015559775],
                    [0.0531454385671997*np.sin(phi_3_1)*np.sin(phi_3_2) + 1.99279435918313e-18*np.sin(phi_3_1)*np.cos(phi_3_2) + 8.25442450594255e-19*np.sin(phi_3_1) + 2.81823680977739e-18*np.sin(phi_3_2)*np.cos(phi_3_1) - 0.0375795*np.cos(phi_3_1)*np.cos(phi_3_2) - 0.0155659385671997*np.cos(phi_3_1) - 1.15054035971395e-18*np.cos(phi_3_2) - 2.77765014047903e-18]])
    
    # print("r_1 shape is: ", r_1.shape)


    F_1 = np.array([[k_f*omega_1**2*(-4.32978028117747e-17*np.sin(phi_1_1)*np.sin(phi_1_2) - 0.25*np.sin(phi_1_1)*np.cos(phi_1_2) - 0.25*np.sin(phi_1_1) - 0.353553390593274*np.sin(phi_1_2)*np.cos(phi_1_1) + 3.06161699786838e-17*np.cos(phi_1_1)*np.cos(phi_1_2) + 3.06161699786838e-17*np.cos(phi_1_1) + 0.433012701892219*np.cos(phi_1_2) - 0.433012701892219)],
                    [k_f*omega_1**2*(0.433012701892219*np.sin(phi_1_1)*np.cos(phi_1_2) + 0.433012701892219*np.sin(phi_1_1) + 0.612372435695794*np.sin(phi_1_2)*np.cos(phi_1_1) + 0.25*np.cos(phi_1_2) - 0.25)],
                    [k_f*omega_1**2*(-0.707106781186547*np.sin(phi_1_1)*np.sin(phi_1_2) + 1.53080849893419e-17*np.sin(phi_1_1)*np.cos(phi_1_2) + 1.53080849893419e-17*np.sin(phi_1_1) + 2.16489014058873e-17*np.sin(phi_1_2)*np.cos(phi_1_1) + 0.5*np.cos(phi_1_1)*np.cos(phi_1_2) + 0.5*np.cos(phi_1_1) - 2.65143809681227e-17*np.cos(phi_1_2) + 2.65143809681227e-17)]])

    F_2 = np.array([[k_f*omega_2**2*(-4.32978028117747e-17*np.sin(phi_2_1)*np.sin(phi_2_2) - 0.25*np.sin(phi_2_1)*np.cos(phi_2_2) - 0.25*np.sin(phi_2_1) - 0.353553390593274*np.sin(phi_2_2)*np.cos(phi_2_1) + 3.06161699786838e-17*np.cos(phi_2_1)*np.cos(phi_2_2) + 3.06161699786838e-17*np.cos(phi_2_1) - 0.433012701892219*np.cos(phi_2_2) + 0.433012701892219)],
                    [k_f*omega_2**2*(-0.433012701892219*np.sin(phi_2_1)*np.cos(phi_2_2) - 0.433012701892219*np.sin(phi_2_1) - 0.612372435695794*np.sin(phi_2_2)*np.cos(phi_2_1) + 0.25*np.cos(phi_2_2) - 0.25)],
                    [k_f*omega_2**2*(-0.707106781186547*np.sin(phi_2_1)*np.sin(phi_2_2) + 1.53080849893419e-17*np.sin(phi_2_1)*np.cos(phi_2_2) + 1.53080849893419e-17*np.sin(phi_2_1) + 2.16489014058873e-17*np.sin(phi_2_2)*np.cos(phi_2_1) + 0.5*np.cos(phi_2_1)*np.cos(phi_2_2) + 0.5*np.cos(phi_2_1) + 2.65143809681227e-17*np.cos(phi_2_2) - 2.65143809681227e-17)]])

    F_3 = np.array([[k_f*omega_3**2*(-4.32978028117747e-17*np.sin(phi_3_1)*np.sin(phi_3_2) + 0.433012701892219*np.sin(phi_3_1)*np.cos(phi_3_2) + 0.433012701892219*np.sin(phi_3_1) + 0.612372435695794*np.sin(phi_3_2)*np.cos(phi_3_1) + 3.06161699786838e-17*np.cos(phi_3_1)*np.cos(phi_3_2) + 3.06161699786838e-17*np.cos(phi_3_1) - 0.25*np.cos(phi_3_2) + 0.25)],
                    [k_f*omega_3**2*(-0.25*np.sin(phi_3_1)*np.cos(phi_3_2) - 0.25*np.sin(phi_3_1) - 0.353553390593274*np.sin(phi_3_2)*np.cos(phi_3_1) - 0.433012701892219*np.cos(phi_3_2) + 0.433012701892219)],
                    [k_f*omega_3**2*(-0.707106781186547*np.sin(phi_3_1)*np.sin(phi_3_2) - 2.65143809681227e-17*np.sin(phi_3_1)*np.cos(phi_3_2) - 2.65143809681227e-17*np.sin(phi_3_1) - 3.74969971630462e-17*np.sin(phi_3_2)*np.cos(phi_3_1) + 0.5*np.cos(phi_3_1)*np.cos(phi_3_2) + 0.5*np.cos(phi_3_1) + 1.53080849893419e-17*np.cos(phi_3_2) - 1.53080849893419e-17)]])

    # print("F_1 shape is ", F_1.shape)
    # print("shape of squeeze F_1 is: ", (np.squeeze(F_1).reshape(3,1)).shape)


    r_1 = np.squeeze(r_1).reshape((3,1))
    r_2 = np.squeeze(r_2).reshape((3,1))
    r_3 = np.squeeze(r_3).reshape((3,1))
    F_1 = np.squeeze(F_1).reshape((3,1))
    F_2 = np.squeeze(F_2).reshape((3,1))
    F_3 = np.squeeze(F_3).reshape((3,1))    

    

    tau_1 = vector_to_skew_symmetric((r_1))@(F_1) 
    tau_2 = vector_to_skew_symmetric((r_2))@(F_2)
    tau_3 = vector_to_skew_symmetric((r_3))@(F_3)
  

    tau_total = tau_1 + tau_2 + tau_3 
    tau_total = np.array(tau_total, dtype=float)
    
    # print("tau_total shape is: ", tau_total.shape)
    # print("tau_total type is: ", type(tau_total))

    # print("tau_total is: \n", tau_total)

    return tau_total





def main():    
    # A_symb = simplify(allocation_matrix())
    # print("A_symb 1st is:\n", A_symb)    
    # A = evaluate_allocation_matrix(A_symb, 200, 200, 200, 0.00, 0.0, 0.00, 0.0, 0.00, 0.0, 0.01, 0.3247545, -0.1872136, 0.0, -0.3247509, -0.1876388, 0.0, -0.0002455, 0.3748525, 0.0)
    # # A_symb = allocation_matrix()
    # A_inv = compute_matrix_inverse(A) 
    # # print("A_inv is: %s" % (str(A_inv))) 
    # #print("A_inv shape is: %s" % (str(A_inv.shape)))
    # print("A_symb 2nd is:\n", A_symb)
    # # print("A_symb simplified is \n", simplify(A_symb))
    compute_forces(np.array([200, 200, 200, 0.00, 0.0, 0.00, 0.0, 0.00, 0.0]), 1.0)

    compute_torques(np.array([200, 200, 200, 0.00, 0.0, 0.00, 0.0, 0.00, 0.0]), 1.0)
    # print_thrust_symb() 
    # print_r_BG()
def main():
    symb_torques()
if __name__ == "__main__":
    main()










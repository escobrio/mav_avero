#needs to have sympy, scipy installed


import numpy as np
from sympy import symbols, sqrt, asin, acos,  diff, pi, cos, sin, lambdify, simplify, atan2


# def nozzle_i_to_body(nozzle_index): 
#     rot_y = -np.pi/2
#     rot_x = 0 
#     n = np.zeros((4, 1))
#     x_1_val = 0.3247545
#     y_1_val = -0.1872136
#     z_1_val = 0.0
#     x_2_val = -0.3247509
#     y_2_val = -0.1876388
#     z_2_val = 0.0
#     x_3_val = -0.0002455
#     y_3_val = 0.3748525
#     z_3_val = 0.0
#     if nozzle_index == 1: 
#         rot_x =np.pi #180 deg CW
#         n = (-1)* np.array([[x_1_val], [y_1_val], [z_1_val], [-1]])
#     elif nozzle_index ==2:
#         rot_x = -np.pi /6 # 30 deg CCW 
#         n = (-1)* np.array([[x_2_val], [y_2_val], [z_2_val],[-1]])
#     elif nozzle_index == 3:
#         rot_x = -np.pi/3 #60 deg CCW 
#         n = (-1)* np.array([[x_3_val], [y_3_val], [z_3_val],[-1]])
#     else: 
#         print("error nozzle index out of scope")



#     C_2 = np.array([
#         [np.cos(rot_y), 0, np.sin(rot_y)],
#         [0, 1, 0],
#         [-np.sin(rot_y), 0, np.cos(rot_y)], 
#         [0,0,0]
#     ]) 
#     T_1 = np.array([
#         [1, 0, 0, 0],
#         [0, np.cos(rot_x), -np.sin(rot_x), 0],
#         [0, np.sin(rot_x), np.cos(rot_x), 0], 
#         [0, 0, 0, 1]
#     ]) 
#     print("T_1 is : ", T_1)

#     T_2 = np.hstack([C_2, n])
#     T_B_0 = np.dot(T_2, T_1)# B = base of the drone frame, 0 base of the  nozzle frame 
#     return T_B_0



def nozzle_i_to_body(nozzle_index): 
    
    n = np.zeros((4, 1))
    x_1_val = 0.15073
    y_1_val = -0.11765
    z_1_val = 0.0
    x_2_val = 0.03067
    y_2_val = 0.18895
    z_2_val = 0.0
    x_3_val = -0.1717
    y_3_val = -0.06737
    z_3_val = 0.0
    if nozzle_index == 1: 
        n = np.array([[x_1_val], [y_1_val], [z_1_val], [1]])
        C = np.array([[0,0,-1],
                        [0.3420, -0.9397,0],
                        [-0.9397, -0.3420,0],
                        [0,0,0]])

    elif nozzle_index ==2:
        
        n = np.array([[x_2_val], [y_2_val], [z_2_val],[1]])
        C = np.array([[-0.2962, 0.8138, 0.5000],
                        [-0.1710, 0.4698, -0.8660],
                        [-0.9397, -0.3420, 0],
                        [0,0,0]])
    elif nozzle_index == 3:
       
        n = np.array([[x_3_val], [y_3_val], [z_3_val],[1]])
        C = np.array([[0.2962, -0.8138, 0.5000],
                        [-0.1710, 0.4698, 0.8660],
                        [-0.9397, -0.3420, 0],
                        [0,0,0]])
        
    else: 
        print("error nozzle index out of scope")

    T_B_0 = np.hstack([C, n])
    return T_B_0


#NOT SURE WHICH ONE IS CORRECT


# def get_chi_des(n, q_last):
#     x_val = n[0]
#     y_val = n[1]
#     z_val = n[2]


#     r = np.sqrt(x_val**2 + y_val**2 + z_val**2)

#     if((y_val<0.0) or ((r == 0.0))):
#         chi_num = np.array([0, 0])

#     elif((x_val == 0.0) and (y_val == 0.0)):
#         theta = np.arccos(y_val/r)
#         phi = q_last[0]
#         return np.array([phi, theta])

        

#     phi = np.arctan2(z_val,x_val)#needs to have sympy, scipy installed





def cart_to_spherical_coord(cart_coord): 
    #input msg in cartesian coordinates 3d
    #output an array in spherical coordinates [phi, theta]
    sph_coord = np.array([0.0, 0.0])
    x = cart_coord[0]
    y = cart_coord[1] 
    z = cart_coord[2] 
    r = np.sqrt(x**2+ z**2)
    d = np.sqrt(x**2 + y**2+ z**2)

    sph_coord[0] = np.arctan2(-z,x)
    sph_coord[1] = np.arccos(y/d) 


    return sph_coord 

def get_chi_des(n, q_last):
    x_val = n[0]
    y_val = n[1]
    z_val = n[2]


    r = np.sqrt(x_val**2 + y_val**2 + z_val**2)

    if((y_val<0.0) or ((r == 0.0))):
        chi_num = np.array([0, 0])

    elif((x_val == 0.0) and (y_val == 0.0)):
        theta = np.arccos(y_val/r)
        phi = q_last[0]
        return np.array([phi, theta])

        

    phi = np.arctan2(z_val,x_val)
    theta = np.arccos(y_val/r)

    return np.array([phi, theta])

def get_T_0E_symbolic():

    q1, q2 = symbols('q1 q2')

    # Define variables
    e1 = np.pi / 4
    e2 = np.pi / 4
    d = 0.075159
    # r = d * (1/sqrt((2 - np.sqrt(2))))
    # print("r = ", r)
    # Define transformation matrices
    C_01 = np.array([
        [cos(q1), 0, sin(q1)],
        [0, 1, 0],
        [-sin(q1), 0, cos(q1)]
    ])

    o_r_01 = np.zeros((3, 1))

    C_12 = np.array([
        [cos(-e1), -sin(-e1), 0],
        [sin(-e1), cos(-e1), 0],
        [0, 0, 1]
    ])

    i_r_12 = d * (1/np.sqrt(2 - np.sqrt(2))) * np.array([[1 - cos(pi/2 - e1)],
                          [sin(pi/2 - e1)],
                           [ 0]])
    
    
    C_23 = np.array([
        [cos(q2), 0, sin(q2)],
        [0, 1, 0],
        [-sin(q2), 0, cos(q2)]
    ])

    ii_r_23 = np.zeros((3, 1))
    

    C_34 = np.array([
        [cos(-e2), -sin(-e2), 0],
        [sin(-e2), cos(-e2), 0],
        [0, 0, 1]
    ])

    iii_r_34 = d * (1/np.sqrt(2 - np.sqrt(2))) * np.array([[1 - cos(pi/2 - e1)],
                          [sin(pi/2 - e1)],
                           [ 0]])

    # Create transformation matrices
    T_01 = np.vstack([np.hstack([C_01, o_r_01]), np.array([0, 0, 0, 1])])
    T_12 = np.vstack([np.hstack([C_12, i_r_12]), np.array([0, 0, 0, 1])])
    T_23 = np.vstack([np.hstack([C_23, ii_r_23]), np.array([0, 0, 0, 1])])
    T_34 = np.vstack([np.hstack([C_34, iii_r_34]), np.array([0, 0, 0, 1])])

    # Calculate the final transformation matrix
    T_0E = np.dot(np.dot(np.dot(T_01, T_12), T_23), T_34)
    C_0E = T_0E[0:3, 0:3]

    return T_0E

def get_T_BE_symbolic(nozzle_index): 
    T_B0 = nozzle_i_to_body(nozzle_index) 
    T_0E = get_T_0E_symbolic() 
    T_BE = np.dot(T_B0, T_0E)
    return T_BE

def get_thrustvector_symbolic():
    post_transform_sym = get_T_0E_symbolic()
    o_r_ey_sym = -1 * post_transform_sym[:3, 1]  # Assuming 1:3 corresponds to the first three rows
    
    #o_r_ey_sym = simplify(o_r_ey_sym)
    
    return o_r_ey_sym

def get_thrustvector_symb_base_frame(nozzle_index): 
    post_transform_sym = get_T_0E_symbolic()
    o_r_ey_sym = -1 * (nozzle_i_to_body(nozzle_index)@ post_transform_sym)[:3, 1]  # Assuming 1:3 corresponds to the first three rows
    
    #o_r_ey_sym = simplify(o_r_ey_sym)
    
    return o_r_ey_sym



def get_thrustvector_num(q1, q2):
    n1_num = -0.707106781186547*np.sin(q1)*np.sin(q2) + 0.5*np.cos(q1)*np.cos(q2) + 0.5*np.cos(q1)
    n2_num = 0.5 - 0.5*np.cos(q2)
    n3_num = -0.5*np.sin(q1)*np.cos(q2) - 0.5*np.sin(q1) - 0.707106781186547*np.sin(q2)*np.cos(q1)
    return np.array([n1_num, n2_num, n3_num])

def getangles_sym():
    #q1, q2, x, y, z = symbols('q1 q2 x y z') #is this needed ?

    x, y, z = symbols('x y z')

    T = get_thrustvector_symbolic()
    x_val = T[0]
    y_val = T[1]
    z_val = T[2]
    
    r = sqrt(x_val**2 + y_val**2 + z_val**2)
    rp = sqrt(x_val**2 + z_val**2)
    
    phi = atan2(-z_val, x_val)
    theta = acos(y_val / r)
    
    #phi = simplify(phi)
    #theta = simplify(theta)
    
    return phi, theta


# Jacobian in the frame on the base of the nozzle 
def getJacobian_angles_sym():
    q1, q2 = symbols('q1 q2')
    phi, theta = getangles_sym()
    
    I_J_sym = np.array([
        [diff(phi, q1), diff(phi, q2)],
        [diff(theta, q1), diff(theta, q2)]
    ])

    
   # I_J_sym = simplify(I_J_sym)
    
    return I_J_sym


def getJacobian_non_minimal_sym(nozzle_index):
    q1, q2 = symbols('q1 q2')
    T = get_thrustvector_symb_base_frame(nozzle_index)
    x_val = T[0]
    y_val = T[1]
    z_val = T[2]
    I_J_non_minimal_sym = np.array([[diff(x_val, q1), diff(x_val, q2)],
                                   [diff(y_val, q1), diff(y_val, q2)],
                                    [diff(z_val, q1), diff(z_val, q2)]])
    return I_J_non_minimal_sym





def pseudoInverseMat(A, lambda_val):
    # Input: Any m-by-n matrix, and a damping factor.
    # Output: An n-by-m pseudo-inverse of the input according to the Moore-Penrose formula
    
    # Get the number of rows (m) and columns (n) of A
    m, n = A.shape

    # Compute the pseudo-inverse for both left and right cases
    if m > n:
        # Compute the left pseudoinverse.
        pinvA = np.linalg.inv(A.T @ A + lambda_val**2 * np.eye(n)) @ A.T
    elif m <= n:
        # Compute the right pseudoinverse.
        pinvA = A.T @ np.linalg.inv(A @ A.T + lambda_val**2 * np.eye(m))
    
    return pinvA

def get_inverseJacobian():
    damping = 0.001
    I_J = getJacobian_angles_sym()
    
    # Using the pseudo-inverse (pinv) from scipy
    I_J_inv = pseudoInverseMat(I_J, damping)

    return I_J_inv

def get_numeric_jac(q1_val, q2_val):
    
    Jac00 = 1
    Jac01 = 0.707106781186548*(np.cos(q2_val) + 1)/(0.5*np.sin(q2_val)**2 + 1.0*np.cos(q2_val) + 1.0)
    Jac10 = 0
    Jac11 = -0.447213595499958*(8.77708367144175e-17*np.cos(q2_val) + 1.58113883008419)*np.sin(q2_val)/np.sqrt(0.5*np.sin(q2_val)**2 + 1.0*np.cos(q2_val) + 1.0)

    numJac = np.array([
        [Jac00, Jac01],
        [Jac10, Jac11]
    ])

    return numJac





def normalize_angle(angle):
    two_pi = 2 * np.pi
    normalized_angle = angle % two_pi

    # Ensure the result is within the range [-2*pi, 2*pi]
    if normalized_angle > np.pi:
        normalized_angle -= two_pi

    return normalized_angle

def get_nearest_rotation(des_angle, angle_now):
    distance = (des_angle - angle_now)%(2*np.pi)

    if (distance < -np.pi):
        distance += 2*np.pi
    elif (distance > np.pi):
        distance -= 2*np.pi
    
    nearest_rot_to_des_angle = angle_now + distance

    return nearest_rot_to_des_angle

    # T_2 = np.hstack([C_2, n])
    # T_B_0 = T_2 @ T_1 # B = base of the drone frame, 0 base of the  nozzle frame 
    # return T_B_0




def main():
    n_1_symb = get_thrustvector_symb_base_frame(1)
    # n_1 = np.array([element.subs({q1: phi_1_1, q2: phi_1_2}) for element in n_1_symb])
    print(n_1_symb)
    # n_2_symb = get_thrustvector_symb_base_frame(2)
    # n_2 = np.array([element.subs({q1: phi_2_1, q2: phi_2_2}) for element in n_2_symb])
    # n_3_symb = get_thrustvector_symb_base_frame(3)
    # n_3 = np.array([element.subs({q1: phi_3_1, q2: phi_3_2}) for element in n_3_symb])
    # print(nozzle_i_to_body(1))
    # T = get_T_BE_symbolic(1)
    # print(T)


if __name__ == "__main__":
    main()
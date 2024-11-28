import numpy as np
from sympy import symbols, sqrt, asin, acos,  diff, pi, cos, sin, lambdify, simplify, atan2


def nozzle_i_to_body(nozzle_index): 
    
    n = np.zeros((4, 1))
    x_1_val = 0.3247545
    y_1_val = -0.1872136
    z_1_val = 0.0
    x_2_val = -0.3247509
    y_2_val = -0.1876388
    z_2_val = 0.0
    x_3_val = -0.0002455
    y_3_val = 0.3748525
    z_3_val = 0.0
    if nozzle_index == 1: 
        n = (-1)* np.array([[x_1_val], [y_1_val], [z_1_val], [-1]])
        C = np.array([[0,0,-1],
                        [0,-1,0],
                        [-1,0,0],
                        [0,0,0]])

    elif nozzle_index ==2:
        
        n = (-1)* np.array([[x_2_val], [y_2_val], [z_2_val],[-1]])
        C = np.array([[0,np.sqrt(3)/2, 1/2],
                        [0, 1/2, -np.sqrt(3)/2],
                        [-1, 0, 0],
                        [0,0,0]])
    elif nozzle_index == 3:
       
        n = (-1)* np.array([[x_3_val], [y_3_val], [z_3_val],[-1]])
        C = np.array([[0,-np.sqrt(3)/2, 1/2],
                        [0, 1/2, np.sqrt(3)/2],
                        [-1, 0, 0],
                        [0,0,0]])
        
    else: 
        print("error nozzle index out of scope")

    T_B_0 = np.hstack([C, n])
    return T_B_0





def main():
    nozzle_index = 2  # Replace with the desired nozzle index
    T_B_0 = nozzle_i_to_body(nozzle_index)
    print("T_B_0 is : ", T_B_0)

if __name__ == "__main__":
    main()
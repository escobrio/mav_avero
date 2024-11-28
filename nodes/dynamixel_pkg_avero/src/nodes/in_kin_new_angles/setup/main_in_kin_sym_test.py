from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des

from sympy import symbols, lambdify, pi, cos, sin, asin, acos, sqrt, atan2
import numpy as np
import time
import matplotlib.pyplot as plt





n = np.array([-1,0,0])

    

q0 = np.array([np.pi/2 ,np.pi/4]) #initial guess (can be last position of the nozzle)
chi_des = get_chi_des(n, q0)
print("chi des" ,chi_des)


##tuning Parameters for the numerical Calculation in the while-loop
it_max = 100
alpha = 0.9 #still needs to be tested for different possible configurations
damping = 0.01
acceptable_chi_err = 1/180*np.pi

alpha_time = np.empty(1000)
start_time = time.time()

q = q0

for it in range(it_max):
    #print("Iteration",it+1)
    #gets the now estimated values for q
    q1_val, q2_val = q

    
    chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))
    chi_err = chi_des - chi_now
    #print("chi_err", chi_err)    
    
    numJac = get_numeric_jac(q1_val, q2_val)
    numJac_pinv = pseudoInverseMat(numJac, damping)



    # alpha = np.linalg.norm(chi_err)*0.5
    # print("alpha = ", alpha)
    A = ((alpha*numJac_pinv) @ chi_err).T
    q = q + A 
    

    if np.all(np.abs(chi_err) < acceptable_chi_err):
        break   





print("--- %s seconds ---" % (time.time() - start_time))
print("Iteration",it+1)

chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))

print("final error = ", chi_err)

#print(q)

q_normalized = np.array(
    [normalize_angle(q[0]), normalize_angle(q[1]) ]
)

#print(q_normalized)
print("final q", q_normalized)
rotation_angles_deg = q_normalized*360/(2*np.pi)
print("rotation_angles_deg", rotation_angles_deg)

#print(get_thrustvector_symbolic())

#print(simplify(Jac_symb)) simplify insane langsam
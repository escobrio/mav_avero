from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des
import sys
from io import StringIO
from sympy import symbols, lambdify, pi, cos, sin, asin, acos, sqrt, atan2
import numpy as np
import time
import matplotlib.pyplot as plt

 ##tuning Parameters for the numerical Calculation in the while-loop
it_max = 1000
alpha = 0.9 #still needs to be tested for different possible configurations
damping = 0.01
acceptable_chi_err = 1/180*np.pi
global tolerance 
tolerance = 1e-6 


alpha_time = np.empty(1000)
start_time = time.time()
counter_err=0
global max_error
max_error = 0
global error 
error = 1
global q
q=np.array([np.pi/2, np.pi/2])

with open('debugging_file.txt', 'w') as f, StringIO() as buffer: 
    f.write("ERROR = 0.02\n")
    for i in range (-100, 101, 10):
        x = i/100 
        for j in range (0, 101, 10):
            y = j/100
            for k in range (-100, 101, 10): 
                z = k/100

                n_xyz = np.array([x, y, z]) 
                if (x==0 and y==0 and z==0): 
                    continue
                n = (1/np.linalg.norm(n_xyz))*n_xyz
                
            
                
                chi_des = get_chi_des(n, q)


                q1_val, q2_val = q
                chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))
                chi_err = chi_des - chi_now 

                

                for it in range(it_max):
                    if np.all(np.abs(chi_err) < acceptable_chi_err):
                        break  

                    #gets the now estimated values for q
                    q1_val, q2_val = q

                    chi_now = cart_to_spherical_coord(get_thrustvector_num(q1_val, q2_val))
                    chi_err = chi_des - chi_now  
                    
                    numJac = get_numeric_jac(q1_val, q2_val)
                    numJac_pinv = pseudoInverseMat(numJac, damping)

                    A = ((alpha*numJac_pinv) @ chi_err).T
                    q = q + A 


                if x == 0 and y == 0 and z>0: 
                    q[0] = np.pi/2
                    q[1] = 0
                elif x == 0 and y == 0 and z<0: 
                    q[0] = -np.pi/2
                    q[1] = 0


                q_normalized = np.array(
                    [normalize_angle(q[0]), normalize_angle(q[1]) ]
                )

                
                rotation_angles_rad = q_normalized
                rotation_angles_deg = rotation_angles_rad*360/(2*np.pi)
                rotation_angles_deg = np.round(rotation_angles_deg).astype(int)


                forw_kin = get_thrustvector_num(q_normalized[0], q_normalized[1])
                forw_kin[2]=-forw_kin[2] #I dont know if it is correct but it works better than before

                max_error = error
                error = 1 - np.dot(forw_kin, n) # this is the dot product approach of the error--> runs but something is wrong with the output 
                    #error =n-forw_kin ### norm approach of the error
                    #error_norm = np.sqrt(np.sum(np.array(error**2))) ### doesnt want to work --> im stuck here

                if error - max_error>tolerance : 
                    max_error = error


                if error>0.001:      #
                    f.write("error number %s in position %s, error %s, forward kinematics give %s\n" %(counter_err, n_xyz, error, forw_kin))
                    print("error number %s in position %s, error %s, forward kinematics give %s" %(counter_err, n, error, forw_kin))
                    f.write("for position %s the joint angles are %s\n" %(n, rotation_angles_deg))
                    counter_err+=1
           
                    rotation_angles_deg = q_normalized*360/(2*np.pi)
                    #f.write("for position %s the joint angles are %s" %(n_xyz, rotation_angles_deg))

    f.write('maximum error is %s' %max_error)
    print('maximum error is %s' %max_error)

    f.write(buffer.getvalue())
sys.stdout = sys.__stdout__




    #print(get_thrustvector_symbolic())

    #print(simplify(Jac_symb)) simplify insane langsam

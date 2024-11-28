from helper_functions_symb import get_inverseJacobian, getJacobian_angles_sym, get_numeric_jac, pseudoInverseMat, get_thrustvector_symbolic, getangles_sym

from sympy import symbols, lambdify, simplify
import numpy as np
import time



#Jac_symb_inv = get_inverseJacobian() does not work yet
# Jac_symb = simplify(getJacobian_angles_sym())
# phi_symb, theta_symb = simplify(getangles_sym())

# print("Jac_symb")
# print(Jac_symb)

# print("phi")
# print(phi_symb)

# print("theta")
# print(theta_symb)$

thrustvector = simplify(get_thrustvector_symbolic())
print(thrustvector)

# Jac00 = 1
# Jac01 = 0.707106781186548*(cos(q2) + 1)/(0.5*sin(q2)**2 + 1.0*cos(q2) + 1.0)
# Jac10 = 0
# Jac11 = -0.447213595499958*(8.77708367144175e-17*cos(q2) + 1.58113883008419)*sin(q2)/sqrt(0.5*sin(q2)**2 + 1.0*cos(q2) + 1.0)

# phi_symb = atan2(0.5*sin(q1)*cos(q2) + 0.5*sin(q1) + 0.707106781186547*sin(q2)*cos(q1), -0.707106781186547*sin(q1)*sin(q2) + 0.5*cos(q1)*cos(q2) + 0.5*cos(q1))
# theta_symb = acos(0.5 - 0.5*cos(q2))

# n1_symb = -0.707106781186547*sin(q1)*sin(q2) + 0.5*cos(q1)*cos(q2) + 0.5*cos(q1)
# n2_symb = 0.5 - 0.5*cos(q2)
# n3_symb = -0.5*sin(q1)*cos(q2) - 0.5*sin(q1) - 0.707106781186547*sin(q2)*cos(q1)

#thrustvector: 
# n1_num = -0.707106781186547*np.sin(q1)*np.sin(q2) + 0.5*np.cos(q1)*np.cos(q2) + 0.5*np.cos(q1)
# n2_num = 0.5 - 0.5*np.cos(q2)
# n3_num = -0.5*np.sin(q1)*np.cos(q2) - 0.5*np.sin(q1) - 0.707106781186547*np.sin(q2)*np.cos(q1)
from helper_functions_symb import pseudoInverseMat, normalize_angle, cart_to_spherical_coord, get_thrustvector_num, get_numeric_jac, get_chi_des
import sys
from io import StringIO
from sympy import symbols, lambdify, pi, cos, sin, asin, acos, sqrt, atan2
import numpy as np
import time
import matplotlib.pyplot as plt

input_deg = np.array([90, 90])
input_rad = 0.0174533 * input_deg 
thrust_vector = get_thrustvector_num(input_rad[0], input_rad[1])
print(thrust_vector)

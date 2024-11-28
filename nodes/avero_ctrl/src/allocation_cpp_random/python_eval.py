import numpy as np 

q1 = 0
q2 = 0
# T_0E = np.array([[-0.707106781186548*np.sin(q1)*np.sin(q2) + 0.5*np.cos(q1)*np.cos(q2) - 0.5*np.cos(q1),
#   -0.707106781186547*np.sin(q1)*np.sin(q2) + 0.5*np.cos(q1)*np.cos(q2) + 0.5*np.cos(q1),
#   np.sin(q1)*np.cos(q2) + 0.707106781186548*np.sin(q2)*np.cos(q1),
#   -0.0287621040931278*np.sin(q1)*np.sin(q2) + 0.020337878845444*np.cos(q1)*np.cos(q2) + 0.0778620870316996*np.cos(q1)],
#  [-0.5*np.cos(q2) - 0.5, 0.5 - 0.5*np.cos(q2), -0.707106781186547*np.sin(q2),
#   0.118537844722588 - 0.020337878845444*np.cos(q2)],
#  [-0.5*np.sin(q1)*np.cos(q2) + 0.5*np.sin(q1) - 0.707106781186548*np.sin(q2)*np.cos(q1),
#   -0.5*np.sin(q1)*np.cos(q2) - 0.5*np.sin(q1) - 0.707106781186547*np.sin(q2)*np.cos(q1),
#   -0.707106781186548*np.sin(q1)*np.sin(q2) + np.cos(q1)*np.cos(q2),
#   -0.020337878845444*np.sin(q1)*np.cos(q2) - 0.0778620870316996*np.sin(q1) - 0.0287621040931278*np.sin(q2)*np.cos(q1)],
#  [0, 0, 0, 1]])

T_BE = [[0.5*np.sin(q1)*np.cos(q2) - 0.5*np.sin(q1) + 0.707106781186548*np.sin(q2)*np.cos(q1),
  0.5*np.sin(q1)*np.cos(q2) + 0.5*np.sin(q1) + 0.707106781186547*np.sin(q2)*np.cos(q1),
  0.707106781186548*np.sin(q1)*np.sin(q2) - 1.0*np.cos(q1)*np.cos(q2),
  0.020337878845444*np.sin(q1)*np.cos(q2) + 0.0778620870316996*np.sin(q1) + 0.0287621040931278*np.sin(q2)*np.cos(q1) + 0.15073],
 [-0.241830519165799*np.sin(q1)*np.sin(q2) + 0.171*np.cos(q1)*np.cos(q2) - 0.171*np.cos(q1) + 0.46985*np.cos(q2) + 0.46985,
  -0.241830519165799*np.sin(q1)*np.sin(q2) + 0.171*np.cos(q1)*np.cos(q2) + 0.171*np.cos(q1) + 0.46985*np.cos(q2) - 0.46985,
  0.342*np.sin(q1)*np.cos(q2) + 0.241830519165799*np.sin(q2)*np.cos(q1) + 0.664468242280999*np.sin(q2),
  -0.00983663959984971*np.sin(q1)*np.sin(q2) + 0.00695555456514185*np.cos(q1)*np.cos(q2) + 0.0266288337648413*np.cos(q1) + 0.0191115047510637*np.cos(q2) - 0.229040012685816],
 [0.664468242280999*np.sin(q1)*np.sin(q2) - 0.46985*np.cos(q1)*np.cos(q2) + 0.46985*np.cos(q1) + 0.171*np.cos(q2) + 0.171,
  0.664468242280999*np.sin(q1)*np.sin(q2) - 0.46985*np.cos(q1)*np.cos(q2) - 0.46985*np.cos(q1) + 0.171*np.cos(q2) - 0.171,
  -0.9397*np.sin(q1)*np.cos(q2) - 0.664468242280999*np.sin(q2)*np.cos(q1) + 0.241830519165799*np.sin(q2),
  0.0270277492163122*np.sin(q1)*np.sin(q2) - 0.0191115047510637*np.cos(q1)*np.cos(q2) - 0.0731670031836881*np.cos(q1) + 0.00695555456514185*np.cos(q2) - 0.040539942895125],
 [0, 0, 0, 1.00000000000000]]

print(T_BE)
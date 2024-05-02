import numpy as np

#R_error = np.matrix([[1,0,0], [0,-1,0], [0,0,-1]])
R_error = np.matrix([[ 1.000e+00,-1.987e-15, -1.000e-02],[-2.235e-11,  1.000e+00,  1.000e-03],[ 1.000e-02, -1.000e-03,  1.000e+00]])
E = np.arccos((np.trace(R_error)-1)/2)
print(E)
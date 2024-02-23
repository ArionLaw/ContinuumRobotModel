import numpy as np


Rdesired = np.array([[-0.966 , 0.217 , -0.141],
 [ 0.176 , 0.949 , 0.261],
 [ 0.19  , 0.228 , -0.955]])

Rshaft = np.array([[ 0.059 , -0.995 , 0.081],
 [-0.804 , -0.  ,   0.594],
 [-0.591 , -0.1 ,  -0.8  ]])

R_wrist = np.matmul(np.transpose(Rshaft), Rdesired)
print(R_wrist)
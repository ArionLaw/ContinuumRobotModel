from utils import *
from plotting import *
import scipy.spatial.transform.rotation as Rotation 
import numpy as np
import time

'''
Python file for coding wrist model. Assuming a 2DOF (notches at 120 degrees).

Assumptions: 
- Square and unrestricted notches
- Constant Curvature Bending
- Equal contribution to bending at each segment
-  

Key parameters:
- n: number of notches for one bending plane; total number of notches is 3*n
- c: distance between notches
- h: length of notch
- prevStraightLen: distance from starting point to first notch
- postStraightLen: distance from last notch to end of tube
- OD: outer diameter of tube
- ID: inner diameter of tube

Secondary parameters (e.g. for testing, visualization):
- d: number of points around circle
- m: number of points per notch/division

Notes:
- Model does not currently consider the introduced slack in Francis' thesis (when two cables are actuated)
'''

def IK_update(R_desired,roll,gamma,beta,alpha):
    start_time = time.time()
    i=0
    orientation_error = 1
    previous_error = 2
    exit = False
    while (i<25) and (orientation_error>0.005) and exit == False:
        i=i+1
        print("i: ",i)
        orientation_error = get_O_error(R_desired,roll,gamma,beta,alpha)
        print("orientation error: ", orientation_error)

        delta = 0.25*orientation_error
        if (abs(previous_error - orientation_error)) < 0.00001:
            exit = True
        
        d_roll = [get_O_error(R_desired,roll+delta,gamma,beta,alpha),get_O_error(R_desired,roll-delta,gamma,beta,alpha)]
        d_gamma = [get_O_error(R_desired,roll,gamma+delta,beta,alpha),get_O_error(R_desired,roll,gamma-delta,beta,alpha)]
        d_beta = [get_O_error(R_desired,roll,gamma,beta+delta,alpha),get_O_error(R_desired,roll,gamma,beta-delta,alpha)]
        d_alpha = [get_O_error(R_desired,roll,gamma,beta,alpha+delta),get_O_error(R_desired,roll,gamma,beta,alpha-delta)]
        
        roll = d_angle(d_roll,orientation_error,roll,delta)
        gamma = d_angle(d_gamma,orientation_error,gamma,delta)
        beta = d_angle(d_beta,orientation_error,beta,delta)
        alpha = d_angle(d_alpha,orientation_error,alpha,delta)
        previous_error = orientation_error

    joint_angles = [roll,gamma,beta,alpha]
    print("--- %s seconds ---" % (time.time() - start_time))
    return joint_angles

def d_angle(d_theta,orientation_error,theta,delta):
    if (d_theta[0] < d_theta[1]) and (d_theta[0] < orientation_error):
        angle = theta+delta
    elif (d_theta[1] < d_theta[0]) and (d_theta[1] < orientation_error):
        angle = theta-delta
    else:
        angle = theta
    return angle

def get_O_error(R_desired,roll,gamma,beta,alpha):
    angles_error = getEulerAngles(get_R_error(R_desired,roll,gamma,beta,alpha))
    E = np.sqrt(abs(angles_error[0])**2 + abs(angles_error[1])**2 + abs(angles_error[2])**2)
    #print("angle error : " , angles_error)
    #print("orientation error: ", E)
    return E

def get_R_error(R_desired,roll,gamma,beta,alpha):
    R = R_desired@np.transpose(get_R_fullwristmodel(roll,gamma,beta,alpha))
    # if there is no orientation error, matrix should be identity
    return R

def get_R_shaft(psm_yaw,psm_pitch):
    R = RotMtx('x',np.pi/2)@RotMtx('z',(psm_yaw+np.pi/2))@RotMtx('x',-np.pi/2)@RotMtx('z',(psm_pitch-np.pi/2))@RotMtx('x',np.pi/2)
    # derived from modified DH convention
    return R

def get_R_fullwristmodel(roll,gamma,beta,alpha):
    R = RotMtx('z',roll)@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha)
    return R

def get_R_segment3notch(gamma,beta,alpha):
    phase_offset = 120*np.pi/180
    #R = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset);
    # "lazy method"
    
    R = RotMtx('x',(-np.pi/2))@RotMtx('z',(gamma-np.pi/2))@RotMtx('x',phase_offset)@RotMtx('z',beta)@RotMtx('x',phase_offset)@RotMtx('z',alpha)@RotMtx('x',phase_offset)@RotMtx('z',np.pi/2)@RotMtx('x',np.pi/2)
    # based off modified DH-Convention
    return R


"""
# wrist inputs
c1_Displacement = 0 #mm
c2_Displacement = 0 #mm
c3_Displacement = 0 #mm
l = [c1_Displacement,c2_Displacement,c3_Displacement]

# fake Modified DH
a = [0 , h , h , h]
alpha = [0 , -1/2*pi , 2/3*pi , 2/3*pi]
d = [c , 0 , 0 , 0]
"""

## my own code
np.set_printoptions(precision=3)

# wrist parameters
n = 3 # sets of 3 cuts
h = 0.66 #mm notch height
c = 0.66 #mm notch spacing
prevStraightLength = 5 #mm
postStraightLength = 1 #mm
y_ = 0.56 #mm neutral bending plane
g = 1.16 #mm notch depth
OD = 1.37 #mm
ID = 0.94 #mm
r = OD/2
w = r*np.sin(np.radians(30))

shaft_length = 200 #mm

# end effector orientation
vec_x = np.array([1,0,0])
vec_z = np.array([0,0,1])
vec_des = np.array([-1,1,0])#np.array([1,1,0])

angle = getAngleTwoVectors(vec_des,vec_z) #acos(dot(vec_des,vec_z)/norm(vec_z)/norm(vec_des))
angle_degrees = np.degrees(angle) #angle*180/pi
axis = getRotationAxis(vec_des, vec_z, angle)#cross(vec_des,vec_z)/norm(vec_z)/norm(vec_des)/sin(angle)

# azimuth about z axis
phi = vec_des[1]/abs(vec_des[1])*np.arccos(np.dot(vec_des[:2],vec_x[:2])/np.linalg.norm(vec_x[:2])/np.linalg.norm(vec_des[:2]))
phi_degrees = phi*180/np.pi
print("phi(azimuth): \n", round(phi_degrees,3))

# altitude about y axis
proj_des = [np.sqrt(vec_des[0]**2 + vec_des[1]**2) , vec_des[2]]
proj_z = [0,1]
theta = np.arccos(np.dot(proj_des,proj_z)/np.linalg.norm(proj_z)/np.linalg.norm(proj_des))
theta_degrees = theta*180/np.pi
print("theta(elevation): \n", round(theta_degrees,3))

#theta = acos(dot(vec_des([1,3]),vec_z([1,3]))/norm(vec_z([1,3]))/norm(vec_des([1,3])))
#need to implement signed consideration (asin?)
#}

R_desired = RotMtx('z',phi)@RotMtx('y',theta)
print("desired Rotation matrix: \n", R_desired)

## FK
roll = 0*np.pi/180
gamma = 0; #35*np.pi/180; 
beta  = 0; #25*np.pi/180; 
alpha = 0; #0*np.pi/180;

psm_yaw = 0.7854; #0*np.pi/180;
psm_pitch = -0.6155; #0*np.pi/180;
psm_insertion = 43.3013; #100; 

# wrist position FK
EE_x = psm_insertion*np.sin(psm_yaw)*np.cos(psm_pitch)
EE_y = -psm_insertion*np.sin(psm_pitch)
EE_z = -psm_insertion*np.cos(psm_yaw)*np.cos(psm_pitch)
EE_pos_FK = [EE_x,EE_y,EE_z]
print("wrist position: \n", EE_pos_FK)
#magnitude = sqrt(EE_x^2 + EE_y^2 + EE_z^2) #check: magnitude = psm_insertion
#}

# orientation FK
R_shaft = get_R_shaft(psm_yaw,psm_pitch)
R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
R_currentFK = R_shaft@R_wrist
print("shaft orientation: \n", R_shaft)
print("wrist orientation: \n", R_wrist)
print("current EE orientation: \n", R_currentFK)
# wrist position FK


## IK
# wrist position IK
EE_pos_desired = [25,25,-25]
psm_insertion = np.sqrt(EE_pos_desired[0]**2 + EE_pos_desired[1]**2 + EE_pos_desired[2]**2); #magnitude = psm_insertion
psm_pitch = np.arcsin(-EE_pos_desired[1]/psm_insertion)
psm_yaw = np.arcsin(EE_pos_desired[0]/np.cos(psm_pitch)/psm_insertion)
psm_joints = [psm_yaw,psm_pitch,psm_insertion]
print("psm joint angles(yaw,pitch,insertion): \n", psm_joints)

# EE_orientation IK
R_wrist_desired = np.linalg.inv(R_shaft)@R_desired
print("R_desired_wrist: \n", R_wrist_desired)
joint_angles = [roll,gamma,beta,alpha]
print("initial notch joint angles: ",joint_angles)
joint_angles = IK_update(R_wrist_desired,joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
print("notch joint angles(roll, gamma, beta, alpha): \n", joint_angles)
R_wrist_IK = get_R_fullwristmodel(joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
print("R_wrist_IK: \n", R_wrist_IK)
print("R_desired_wrist: \n", R_wrist_desired)
R_updated = R_shaft@R_wrist_IK
#print("R_full_IK: \n", R_updated)
#print("R_desired: \n", R_desired)
#}

## Cable Displacement Calculation per segment
# per set of 3 cuts
R = abs(h/theta)
K = 1/R
if (theta > 0):
    L1 = np.sqrt(2*(R-y_-r)**2*(1-np.cos(theta)))
    L2 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(theta)))
    L3 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(theta)))
else:
    L1 = np.sqrt(2*(R-y_+r)**2*(1-np.cos(theta)))
    L2 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(theta)))
    L3 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(theta)))


displacementL1 = h - L1
displacementL2 = h - L2
displacementL3 = h - L3
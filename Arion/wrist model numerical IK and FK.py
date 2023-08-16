from utils import *
from TaskSpace_to_JointSpace import *
from JointSpace_to_CableSpace import *
from CableSpace_to_DiskSpace import *
from plotting import *
import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
# Notes
#----------------------------------------------------------------------------------------------------------------------------------------------#
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
np.set_printoptions(precision=3)
getCabletoDiskMapping()

#----------------------------------------------------------------------------------------------------------------------------------------------#
# wrist parameters
#----------------------------------------------------------------------------------------------------------------------------------------------#
global n , h , c, prevStraightLength, postStraightLength, y_, g, OD, ID, r, w
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

#----------------------------------------------------------------------------------------------------------------------------------------------#
#desired end effector orientation rotation matrix description
#----------------------------------------------------------------------------------------------------------------------------------------------#
vec_x = np.array([1,0,0])
vec_z = np.array([0,0,1])
pos_desired = np.array([-1,1,-0.5])#np.array([1,1,0])
tip_desired = 0 #in degrees

EE_pos_desired = [0,0,-100] #[25,25,-25] #in mm

#angle = getAngleTwoVectors(pos_des,vec_z) #acos(dot(vec_des,vec_z)/norm(vec_z)/norm(vec_des))
#angle_degrees = np.degrees(angle) #angle*180/pi
#axis = getRotationAxis(pos_des, vec_z, angle)#cross(vec_des,vec_z)/norm(vec_z)/norm(vec_des)/sin(angle)

# azimuth about z axis
phi = pos_desired[1]/abs(pos_desired[1])*np.arccos(np.dot(pos_desired[:2],vec_x[:2])/np.linalg.norm(vec_x[:2])/np.linalg.norm(pos_desired[:2]))
phi_degrees = phi*180/np.pi
print("phi(azimuth): \n", round(phi_degrees,3))

# altitude about y axis
proj_des = [np.sqrt(pos_desired[0]**2 + pos_desired[1]**2) , pos_desired[2]]
proj_z = [0,1]
theta = np.arccos(np.dot(proj_des,proj_z)/np.linalg.norm(proj_z)/np.linalg.norm(proj_des))
theta_degrees = theta*180/np.pi
print("theta(elevation): \n", round(theta_degrees,3))

# tool tip roll about z axis
tip_roll = tip_desired*np.pi/180
print("tool tip roll: \n", round(tip_desired,3))

R_desired = RotMtx('z',phi)@RotMtx('y',theta)@RotMtx('z',tip_roll)
print("desired Rotation matrix: \n", R_desired)

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
# initial joint configurations
""""""
roll = 0*np.pi/180
gamma = 0; #35*np.pi/180; 
beta  = 0; #25*np.pi/180; 
alpha = 0; #0*np.pi/180;

psm_yaw = 0.7854; #0*np.pi/180;
psm_pitch = -0.6155; #0*np.pi/180;
psm_insertion = 43.3013; #100;

disk_positions = [-0.2136763752838734, -1, 0.26000965425093653, 0.12707334917195753] # [roll , EE , Disk 3, Disk 4]
joint_angles = DiskPosition_To_JointSpace(disk_positions)
roll = joint_angles[0]
EE_grip = joint_angles[1]
gamma = joint_angles[2]
beta = joint_angles[3]
alphaa = joint_angles[4]

print("\n FK")
# wrist position FK
EE_pos_FK = get_wristPosition_from_PSMjoints(psm_pitch,psm_yaw,psm_insertion)
print("wrist position: \n", EE_pos_FK)

# orientation FK
R_shaft = get_R_shaft(psm_yaw,psm_pitch)
R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
R_currentFK = R_shaft@R_wrist
print("shaft orientation: \n", R_shaft)
print("wrist orientation: \n", R_wrist)
print("current EE orientation: \n", R_currentFK)

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

print("\n IK")
# wrist position IK
psm_joints = get_PSMjoints_from_wristPosition(EE_pos_desired)
print("psm joint angles(yaw,pitch,insertion): \n", psm_joints)

# EE_orientation IK
R_wrist_desired = np.linalg.inv(R_shaft)@R_desired
print("R_desired_wrist: \n", R_wrist_desired)
joint_angles = [roll,gamma,beta,alpha]
print("initial notch joint angles: ",joint_angles)
joint_angles = IK_update(R_wrist_desired,joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
print("notch joint angles(roll, gamma, beta, alpha): \n", joint_angles)
R_wrist_IK = get_R_fullwristmodel(joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
#print("R_wrist_IK: \n", R_wrist_IK)
#print("R_desired_wrist: \n", R_wrist_desired)
R_updated = R_shaft@R_wrist_IK
#print("R_full_IK: \n", R_updated)
#print("R_desired: \n", R_desired)

deltaCablesGamma = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[1], "0")
deltaCablesBeta = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[2], "120")
deltaCablesAlpha = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[3], "240")
deltaCablesTotal = 3*(deltaCablesGamma + deltaCablesBeta + deltaCablesAlpha)
EE_pull = 1 #place holder value for now, need to obtain from ROS topic
#print("cable deltas for notch 1: ", deltaCablesGamma)
#print("cable deltas for notch 2: ", deltaCablesBeta)
#print("cable deltas for notch 3: ", deltaCablesAlpha)
#print("total cable delta: ", deltaCablesTotal)

DiskAngles = [getDiskAngles(joint_angles[0],EE_pull,-deltaCablesTotal[0],-deltaCablesTotal[1],-deltaCablesTotal[2])] 
# getDiskAngles inputs [roll (joint space), end effector actuation (joint space), cable 1 (cable space), cable 2 (cable space), cable 3 (cable space)]
print("Disk Angles: \n", DiskAngles)


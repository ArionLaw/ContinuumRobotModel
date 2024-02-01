#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
def get_wristPosition_from_PSMjoints(psm_joints,wristlength):
    """
    obtain position of wrist before notches from yaw, pitch and insertion joint values
    joint[0] = yaw
    joint[1] = pitch
    joint[2] = insertion
    """
    EE_x = (psm_joints[2]+wristlength/2)*np.sin(psm_joints[0])*np.cos(psm_joints[1])
    EE_y = (-psm_joints[2]+wristlength/2)*np.sin(psm_joints[1])
    EE_z = (-psm_joints[2]+wristlength/2)*np.cos(psm_joints[0])*np.cos(psm_joints[1])
    return [EE_x,EE_y,EE_z]

def get_R_shaft(psm_joints):
    """
    calculates rotation matrix of instrument prior to roll joint
    derived from daVinci PSM modified DH convention
    """
    R1 = RotMtx('x',np.pi/2)@RotMtx('z',(psm_joints[0]+np.pi/2)) # dVRK DH frame 1
    R2 = RotMtx('x',-np.pi/2)@RotMtx('z',(psm_joints[1]-np.pi/2)) # dVRK DH frame 2
    R3 = RotMtx('x',np.pi/2)@RotMtx('z',0) # dVRK DH frame 3
    R = R1@R2@R3
    return R

def get_R_wrist(outer_roll,pitch_angle,inner_roll):
    """
    calculates rotation matrix of wrist relative to shaft
    based on modified DH convention 
    """
    R4 = RotMtx('x',0)@RotMtx('z',outer_roll) # dVRK DH frame 4
    R5 = RotMtx('x',-np.pi/2)@RotMtx('z',pitch_angle) # dVRK DH frame 5
    R6 = RotMtx('x',np.pi/2)@RotMtx('z',inner_roll) # dVRK DH frame 6
    R7 = RotMtx('z',np.pi/2) # dVRK DH frame 7
    R = R4@R5@R6@R7
    
    return R

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_PSMjoints_from_wristPosition(EE_pos_desired,wristlength):
    """
    obtain initial yaw, pitch and insertion joint values of PSM kinematic chain from desired EE_position
    """
    x = float(EE_pos_desired[0])
    y = float(EE_pos_desired[1])
    z = float(EE_pos_desired[2])
    psm_insertion = np.sqrt(x**2 + y**2 + z**2) - wristlength/2; #magnitude = psm_insertion
    psm_pitch = np.arcsin(-y/psm_insertion)
    psm_yaw = np.arcsin(x/np.cos(psm_pitch)/psm_insertion)
    return [psm_yaw,psm_pitch,psm_insertion]

def IK_update(R_desired,outer_roll,pitch_angle,inner_roll,printout):
    """
    IK numerical soln for taskspace to joint space roll and notch angles
    """
    
    #new geometric IK soln 
    
    return joint_angles
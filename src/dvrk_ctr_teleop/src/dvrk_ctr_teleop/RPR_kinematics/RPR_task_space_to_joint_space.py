#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np
import math

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


def wrist_analytical_ik(R_wrist_desired, R_current, R_previous,q_current, is_first_update_after_a_disable):
    """
    calculates wrist joint solutions given desired rotation matrix for the wrist
    returns 2 possible IK solutions 
    """
    #print('R_wrist_desired PASSED TO ik', R_wrist_desired)

    #R_change = R_current@np.
  
    q5 = math.acos(R_wrist_desired[2,2])
    min_deg_limit = 3
    rotation_add_deg = 10

    # if abs(q5) < min_deg_limit*np.pi/180:
    #     #R_wrist_desired = R_wrist_desired@RotMtx('x',rotation_add_deg*np.pi/180)@RotMtx('y',rotation_add_deg*np.pi/180)
    #     print("current q5", q5)
    #     print("min allowed q5", min_deg_limit*np.pi/180)
    #     raise Exception('Singularity Reached')

    
    r33 = R_wrist_desired[2,2]
    r31 = R_wrist_desired[2,0]
    r32 = R_wrist_desired[2,1]
    r13 = R_wrist_desired[0,2]
    r23 = R_wrist_desired[1,2]



    if r33 == 1:
         print("AT SINGULARITY...OFFSETTING")
         r33 = 0.9995 #small offset to avoid singularity

    q5_1 = math.acos(r33)
    q5_2 = -math.acos(r33)
    

    q4_1 = math.atan2(r23, r13)
    q6_1 = math.atan2(r31, r32)

    q4_2 = math.atan2(-r23, -r13)
    q6_2 = math.atan2(-r31, -r32)

    # q4_3 = q6_1
    # q5_3 = q5_1
    # q6_3 = q4_1

    # first_sol = np.array([q4_1,q5_1,q6_1])

    # diff = np.abs(q_current-first_sol)
    # tol = 10000
    # print("q_current", q_current)
    # print("first_sol", first_sol)
    # if diff[0] > tol and diff[2] > tol:
    #     first_sol = np.array([q6_1, q5_1, q4_1])
    #     print("--------------------------------Flipping_solution-----------------------------------------")
    # if np.any(np.abs(q_current-first_sol) >= 100):
    #     print("USING 3rd Solution")
    #     print("ANGLE DIFF", np.sum(np.abs(first_sol - q_current)))

    #     wrist_ik_sol = np.array([[q4_1,q5_1,q6_1],
    #                         [q4_2, q5_2, q6_2],
    #                         [q4_3,q5_3,q6_3]])
    # else:
    
    if(is_first_update_after_a_disable):
        # print('Clutch Status', is_first_update_after_a_disable)
        # first_sol = np.array([q4_1,q5_1,q6_1])
        # second_sol = np.array([q4_2,q5_2,q6_2])
        # interpolated_sol_1 = q_current + (first_sol - q_current)/2
        # interpolated_sol_2 = q_current + (second_sol -q_current)/2
        # wrist_ik_sol = np.array([interpolated_sol_1,
        #                          interpolated_sol_2])
        
        # print('FIRST SOL', first_sol)
        # print("Second SOl", second_sol)
        # print("CURRENT SOLUTION", q_current)
        # print("Wrist IK SOLUTIONS", wrist_ik_sol)
          
        wrist_ik_sol = np.array([[q4_1,q5_1,q6_1],
                                [q4_2, q5_2, q6_2]])
    else: 
        wrist_ik_sol = np.array([[q4_1,q5_1,q6_1],
                                [q4_2, q5_2, q6_2]])
                
    return wrist_ik_sol

class WristIKSolutionSelector:
    '''selects best IK solution based on wrist pitch joint limits and effort'''

    def __init__(self, wrist_pitch_joint_limits):
        self.joint_limits = np.array(wrist_pitch_joint_limits)
    
    def normalize_angle(self,angles):
        """ Normalize angles to the range [0, 2*pi] """
        return np.mod(angles, 2 * np.pi)
    
    def calculate_effort(self,q_current, q_target):
        """ Calculate the effort required to move from q_current to q_target"""
        q_current = self.normalize_angle(q_current)
        q_target = self.normalize_angle(q_target)

        # Calculate the difference and account for the circular nature of angles
        diff = np.abs(q_current - q_target)
        diff = np.where(diff > np.pi, 2 * np.pi - diff, diff)

        return np.sum(diff)

    def is_within_limits(self, angles):
        """ Check if the wrist pitch joint angle is within the joint limits"""
        q5 = angles[1] 
        # Normalize q5 to be within -pi to pi
        q5 = np.arctan2(np.sin(q5), np.cos(q5))

        return self.joint_limits[0] <= q5 <= self.joint_limits[1]


    def select_best_solution(self, q_current, solutions):
        """ Select the IK solution that requires the smallest change in angles and is within joint limits"""

        q_current = np.array(q_current)
        valid_solutions = [sol for sol in solutions if self.is_within_limits(sol)]

        if not valid_solutions:

            # Set q5 to its nearest joint limit for each solution
            adjusted_solutions = []

            for sol in solutions:
                q5 = sol[1]
                if q5 < self.joint_limits[0]:
                    q5 = self.joint_limits[0]
                elif q5 > self.joint_limits[1]:
                    q5 = self.joint_limits[1]
                adjusted_sol = sol.copy()
                adjusted_sol[1] = q5
                adjusted_solutions.append(adjusted_sol)

            valid_solutions = adjusted_solutions

        efforts = [(self.calculate_effort(q_current, np.array(sol)), sol) for sol in valid_solutions]
        return min(efforts, key=lambda x: x[0])[1]
    




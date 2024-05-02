#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np
import math
from scipy.spatial.transform import Rotation as R

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

def interpolate_angles(q_desired, q_cur, step_rad = 0.1):
    step = np.sign(q_desired - q_cur) * step_rad
    q_stepped = step + q_cur
    abs_joint_diffs = np.absolute(q_desired - q_cur)
    return np.where(abs_joint_diffs < np.absolute(step), q_desired, q_stepped)

class WristIKSolver:
    '''selects best IK solution based on wrist pitch joint limits and effort'''

    def __init__(self, wrist_pitch_joint_limits, min_deg_limit):
        self.joint_limits = np.array(wrist_pitch_joint_limits)
        self.min_deg_limit = min_deg_limit
        self.q_previous = min_deg_limit*np.pi/180
        self.R_previous = RotMtx('x',0)
        self.R_singularity = np.array([[0, -1, 0],
                                    [1, 0, 0],
                                    [0, 0, 1]])
    
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
    
    def wrist_analytical_ik(self, R_wrist_desired, R_shaft):
        """
        calculates wrist joint solutions given desired rotation matrix for the wrist
        returns 2 possible IK solutions 
        """

        q5 = math.acos(R_wrist_desired[2,2])

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

        wrist_ik_sol = np.array([[q4_1,q5_1,q6_1],
                            [q4_2, q5_2, q6_2]])
        
        return wrist_ik_sol
    
    def avoid_singularity(self, R_wrist_desired, q_diff,R_shaft, rotation_add_deg = 10):

        def project_vector_on_plane(n,u):  
            n_norm = np.linalg.norm(n)    
            proj_of_u_on_n = (np.dot(u, n)/n_norm**2)*n 
            return u - proj_of_u_on_n


        R_change = np.transpose(self.R_previous)@R_wrist_desired
        rotation_vec = R.from_matrix(R_change).as_rotvec()
        rotation_vec = rotation_vec/np.linalg.norm(rotation_vec)

        ###vector from singularity to Rprevious
        R_change_2 = np.transpose(R_shaft@self.R_singularity)@self.R_previous
        rotation_vec_2 = R.from_matrix(R_change_2).as_rotvec()
        rotation_vec_2 = rotation_vec_2/np.linalg.norm(rotation_vec_2)

        ###Get Projection Plane####
        shaft_plane_normal = R_shaft@np.array([0,0,1])

        rot_vec_proj = project_vector_on_plane(shaft_plane_normal,rotation_vec)
        rot_vec2_proj = project_vector_on_plane(shaft_plane_normal,rotation_vec_2)

        ##calculate angle
        cross_product = np.cross(rot_vec2_proj,rot_vec_proj)
        direction = cross_product[2]

        # Calculate current q5 from R_wrist_desired
        q5 = math.acos(R_wrist_desired[2, 2])
        min_deg_limit = self.min_deg_limit* np.pi / 180  

         # Handling Zero Direction
        if direction == 0:
            direction = np.sign(rotation_vec[0] * rotation_vec[1])
            if direction == 0:
                # If still zero, default to a positive direction
                direction = 1


        # Refining Angle Calculation
        if q_diff >= 1e-6:
            if direction < 0:
                # Smooth transition for increasing q_diff with negative direction
                rotation_angle = np.pi / 2 + np.pi / 2 * (q5 / min_deg_limit)
            else:
                # Smooth transition for increasing q_diff with positive direction
                rotation_angle = rotation_angle = np.pi/2 - np.pi / 2 * (q5 / min_deg_limit)
        else:
            if direction < 0:
                # Smooth transition for decreasing q_diff with negative direction
                rotation_angle = np.pi / 2 * (1 - q5 / min_deg_limit)
            else:
                # Smooth transition for decreasing q_diff with positive direction
                rotation_angle = np.pi/2 + np.pi / 2 * (1 - q5 / min_deg_limit)

        R_wrist_desired_adjusted = R_wrist_desired @ RotMtx('z', rotation_angle)@RotMtx('x',rotation_add_deg*np.pi/180)
        return R_wrist_desired_adjusted
    




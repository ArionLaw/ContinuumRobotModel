#from RPR_task_space_to_joint_space import *
#from RPR_joint_space_to_cable_space import *
#from RPR_cable_space_to_disk_space import *
#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.RPR_task_space_to_joint_space import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_joint_space_to_cable_space import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_cable_space_to_disk_space import *
from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np
import yaml
import os
import pdb

#----------------------------------------------------------------------------------------------------------------------------------------------#
# Notes
#----------------------------------------------------------------------------------------------------------------------------------------------#
'''

'''


class RPRKinematicsSolver:
#----------------------------------------------------------------------------------------------------------------------------------------------#
# wrist parameters to be placed in YAML
#----------------------------------------------------------------------------------------------------------------------------------------------#
    def __init__(self, config_path):
            self.OD = 1.7 #mm
            self.ID = 1.3 #mm
            self.r = self.OD/2
            self.n = 9 # sets of 3 cuts
            self.h = 0.7 #mm notch height
            self.c = 0.4 #mm notch spacing
            self.y_ = 0.716 #mm neutral bending plane
            self.g = self.r + 0.721 #mm notch depth
            self.w = self.r*np.sin(np.radians(30))
            self.shaft_length = 400 #mm
            self.R_wrist_previous = get_R_wrist(0,0,0)

            new_path = os.path.join(sys.path[0], config_path)
            yaml_file = open(new_path)
            config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)

            self.wrist_length = config_yaml["wrist_length"]
            self.simulation = config_yaml['simulation']
            self.scale = config_yaml['scale']
            self.q4_limits = config_yaml["wrist_outer_roll_joint_limits"]
            self.q6_limits = config_yaml["wrist_inner_roll_joint_limits"]
            self.WristIKSolver = WristIKSolver(config_yaml["wrist_pitch_joint_limits"],
                                               config_yaml["min_deg_limit"])
            self.CableToDiskSpaceSolver = CableToDiskSpaceSolver(config_yaml)

            #self.kinematics_data = PsmKinematicsData(spherical_wrist_tool_params)
            #self.negate_joint_list = spherical_wrist_tool_params.negate_joint_list
#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
    def _compute_all_fk(self, joints):
            """
            compute from disk space angles to cable space displacements
                    from cable space displacements to joint space angles 
                    from joint space angles to task space poses                 
            """        
            psm_joints = joints[0:3]

            if self.simulation:
                  q4 = joints[3] 
                  q5 = joints[4]*6
                  q6 = joints[10] 
                  psm_joints[2] = psm_joints[2]/10 #scale down insertion
                  EE_pinch_angle = joints[11]

            else:      
                  disk_positions = joints[3:]
                 
                  """ from disk space angles to joint space angles """
                  q4, q5, q6, EE_pinch_angle = self.CableToDiskSpaceSolver.disk_position_to_joint_space(disk_positions,self.h,
                                                                                                      self.y_,self.r,self.n)

            """ orientation FK """
            R_shaft = get_R_shaft(psm_joints)
            R_wrist = get_R_wrist(q4,q5,q6)
            R_currentFK = R_shaft@R_wrist

            """ wrist position FK """
            wrist_pos_FK = get_wristPosition_from_PSMjoints(psm_joints, self.wrist_length)
            EE_pos_FK = wrist_pos_FK + self.wrist_length/2*R_wrist@np.array([0,0,1])

            return ConvertToTransformMatrix(R_currentFK,EE_pos_FK), EE_pinch_angle

    def compute_fk(self, joint_positions):
            tf, _ = self._compute_all_fk(joint_positions)
            return tf
        

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
    def compute_ik(self, tf_desired, direct_psm_and_disk_joint_positions, desired_EE_pinch_angle, is_first_update_after_a_disable = False):
            """
            compute from task space poses to joint space angles
                    from joint space angles to cable space displacements
                    from cable space displacements to disk space angles
            """
            ee_position_desired = np.copy(tf_desired[0:3,3])
            R_desired = np.copy(tf_desired[0:3, 0:3])

            if not isinstance(desired_EE_pinch_angle,float):
                desired_EE_pinch_angle = desired_EE_pinch_angle[0]

            if self.simulation:
                q4 = direct_psm_and_disk_joint_positions[3]
                q5 = direct_psm_and_disk_joint_positions[4]*6 #pitch
                q6 = direct_psm_and_disk_joint_positions[10] #inner roll
                current_wrist_angles = [q4,q5,q6]

                if desired_EE_pinch_angle <0.0:
                       desired_EE_pinch_angle = 0.0

            else:
                disk_positions = direct_psm_and_disk_joint_positions[3:]

                """ FK calculate wrist pseudojoints of current pose using """ 
                q4, q5, q6, current_jaw_angle = self.CableToDiskSpaceSolver.DiskPosition_To_JointSpace(disk_positions,self.h,self.y_,self.r,self.n)
                current_wrist_angles = [q4,q5,q6]

            
            """ IK calculate psm joints to obtain wrist cartesian position """
            PSM_wrist_pos_desired = ee_position_desired - self.wrist_length/2*R_desired@np.array([0,0,1])
            psm_joints = get_PSMjoints_from_wristPosition(PSM_wrist_pos_desired,self.wrist_length)

            """ FK calculate current shaft orientation given wrist cartesian position"""
            R_shaft = get_R_shaft(psm_joints)
            R_wrist_current = get_R_wrist(q4,q5,q6)

            """IK calculate wrist joint solutions and select the best solution"""
            R_wrist = np.matmul(np.transpose(R_shaft), R_desired)
            wrist_ik_sols = self.WristIKSolver.wrist_analytical_ik(R_wrist,R_shaft)
            q4q5q6 = self.WristIKSolver.select_best_solution(current_wrist_angles, wrist_ik_sols)

            q4q5q6 = interpolate_angles(q4q5q6, np.array(current_wrist_angles))
            q4 = q4q5q6[0]
            q5 = q4q5q6[1]
            q6 = q4q5q6[2]

        #     # Clip solution to prevent rolling around
            q4 = np.clip(q4,self.q4_limits[0],self.q4_limits[1])
            q6 = np.clip(q6,self.q6_limits[0],self.q6_limits[1])

            if self.simulation: 
                psm_joints.append(q4) #outer_roll
                wrist_joints = []
                for _ in range(0,6):
                    wrist_joints.append(q5/6) #pitch
                wrist_joints.append(q6) #inner_roll
                joints_list = psm_joints + wrist_joints
                joints_list.append(desired_EE_pinch_angle)
                joints_list[2] *=self.scale  #scale up insertion
            else:
                DiskAngles = self.CableToDiskSpaceSolver.get_disk_angles(q4,q5,q6,desired_EE_pinch_angle,
                                                                         current_jaw_angle,self.h,self.y_,
                                                                         self.r,self.w,self.n)
                joints_list = psm_joints + DiskAngles

            return joints_list

#----------------------------------------------------------------------------------------------------------------------------------------------#
### setup ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        
    def get_active_joint_names(self):
            return ["outer_yaw", "outer_pitch", "outer_insertion", \
                    "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw", "jaw"]

    def get_link_names(self):
            return list(self.kinematics_data.link_name_to_dh.keys())
    
    def actuator_to_joint(self, actuator_positions):
            tf,jaw_angle = self._compute_all_fk(actuator_positions)
            joint_positions = np.copy(actuator_positions)
            if self.simulation:
                joint_positions[11] = jaw_angle
            else:
                joint_positions[6] = jaw_angle
            return joint_positions


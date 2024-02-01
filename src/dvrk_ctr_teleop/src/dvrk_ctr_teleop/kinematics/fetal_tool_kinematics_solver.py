import os
import numpy as np
import yaml

from dvrk_ctr_teleop.kinematics.utils import *
from dvrk_ctr_teleop.kinematics.task_space_to_joint_space import *
from dvrk_ctr_teleop.kinematics.joint_space_to_cable_space import *
from dvrk_ctr_teleop.kinematics.cable_space_to_disk_space import *
# from dvrk_ctr_teleop.kinematics.plotting import *
from dvrk_planning.kinematics.kinematics_solver import KinematicsSolver # TODO later
        
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
# np.set_printoptions(precision=3)
printout = False
#getCabletoDiskMapping()
#getEECabletoDisk2Mapping()

class PeterFrancisToolKinematicsSolver(KinematicsSolver):
#----------------------------------------------------------------------------------------------------------------------------------------------#
# wrist parameters to be placed in YAML
#----------------------------------------------------------------------------------------------------------------------------------------------#
        def __init__(self, config_path):
                self.n = 3 # sets of 3 cuts
                self.h = 0.66 #mm notch height
                self.c = 0.66 #mm notch spacing
                self.prevStraightLength = 5 #mm
                self.postStraightLength = 1 #mm
                self.y_ = 0.56 #mm neutral bending plane
                self.g = 1.16 #mm notch depth
                self.OD = 1.37 #mm
                self.ID = 0.94 #mm
                self.r = self.OD/2
                self.w = self.r*np.sin(np.radians(30))
                self.shaft_length = 400 #mm

                # TODO Arion, yaml example
                # model_str = yaml["model"]
                # vscale = yaml["vscale"]

                new_path = os.path.join(sys.path[0], config_path)
                yaml_file = open(new_path)
                config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)

                # print(sys.path[0])
                # print(config_yaml)
                # file_path = sys.path[0]
                # file_path = file_path.replace('\src\dvrk_planning\dvrk_planning\src\dvrk_planning\kinematics','')

                self.cable_to_disk_map = getCabletoDiskMapping(config_yaml["cable_to_disk_map"])
                self.eecable_to_disk_map = getEECabletoDisk2Mapping(config_yaml["ee_cable_to_disk_map"])
                self.wrist_length = config_yaml["wrist_length"]
                self.config_yaml = config_yaml

                #print(file_path)
                #self.cable_to_disk_map.to_csv(file_path +'/dialmapping.csv')
                #self.eecable_to_disk_map.to_csv(file_path +'/EEmapping.csv')

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        def _compute_all_fk(self, joints):
                """
                compute from disk space angles to cable space displacements
                        from cable space displacements to joint space angles 
                        from joint space angles to task space poses                 
                """        
                if printout is True: print("------------------------------------------- FK -------------------------------------------")
                psm_joints = joints[0:3]
                disk_positions = joints[3:]
                #if printout is True: print("joints", joints)

                """ from disk space angles to joint space angles """
                joint_values = DiskPosition_To_JointSpace(disk_positions, self.h, self.y_, self.r,
                                                          self.cable_to_disk_map, self.eecable_to_disk_map,self.config_yaml)
                if printout is True: print("PSM Joint Values(yaw,pitch,insertion): \n",psm_joints)
                if printout is True: print("Instrument Joint Values(roll, EE jaw, gamma, beta, alpha):  \n" , joint_values)
                roll = joint_values[0]
                EE_pinch_angle = joint_values[1]
                gamma = joint_values[2]
                beta = joint_values[3]
                alpha = joint_values[4]

                """ wrist position FK """
                EE_pos_FK = get_wristPosition_from_PSMjoints(psm_joints)
                #if printout is True: print("Wrist Position: \n", EE_pos_FK)

                """ orientation FK """
                R_shaft = get_R_shaft(psm_joints)
                R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
                R_currentFK = R_shaft@R_wrist
                if printout is True: print("Shaft Orientation: \n", R_shaft)
                if printout is True: print("Wrist Orientation: \n", R_wrist)
                if printout is True: print("Current EE Orientation: \n", R_currentFK)

                return ConvertToTransformMatrix(R_currentFK,EE_pos_FK),EE_pinch_angle, joint_values

        def compute_fk(self, joint_positions):
                tf, _, _ = self._compute_all_fk(joint_positions)
                return tf

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        def _compute_all_ik(self, tf_desired, direct_psm_and_disk_joint_positions, desired_EE_pinch_angle):
                """
                compute from task space poses to joint space angles
                        from joint space angles to cable space displacements
                        from cable space displacements to disk space angles
                """
                if printout is True: print("------------------------------------------- IK -------------------------------------------")     
                
                PSM_wrist_pos_desired = tf_desired[0:3, 3]
                #print("tf_p:\n", PSM_wrist_pos_desired)
                R_desired = tf_desired[0:3, 0:3]
                #print("tf_R:\n", R_desired)
                disk_positions = direct_psm_and_disk_joint_positions[3:]
                #print("disk_positions:\n", disk_positions)

                """ FK calculate wrist pseudojoints of current pose using """ 
                joint_values = DiskPosition_To_JointSpace(disk_positions, self.h, self.y_, self.r,
                                                          self.cable_to_disk_map, self.eecable_to_disk_map,self.config_yaml)
                if printout is True: print("Current Wrist Joint Values: \n(roll, EE jaw, gamma, beta, alpha):\n" , joint_values) 
                
                roll = joint_values[0]
                #EE_pinch_angle = joint_values[1]
                gamma = joint_values[2]
                beta = joint_values[3]
                alpha = joint_values[4]

                """ IK calculate psm joints to obtain wrist cartesian position """
                psm_joints = get_PSMjoints_from_wristPosition(PSM_wrist_pos_desired)
                #if printout is True: print("PSM Joint Values(yaw,pitch,insertion):\n", psm_joints)

                """ FK calculate current shaft orientation given wrist cartesian position """
                # R_desired = calc_R_desired(EE_orientation_desired)
                R_shaft = get_R_shaft(psm_joints)

                """ FK calculate current wrist orientation of current pose """
                R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
                R_currentFK = R_shaft@R_wrist
                #if printout is True: print("Desired Orientation: \n", np.around(R_desired,4))
                #if printout is True: print("Shaft Orientation: \n", np.around(R_shaft,4))
                #if printout is True: print("Wrist Orientation: \n", np.around(R_wrist,4))
                if printout is True: print("Current EE Orientation: \n", np.around(R_currentFK,4))

                """ numerical IK calculate pseudojoint values to achieve desired EE_orientation (comparison Rcurrent vs Rdesired)"""
                R_wrist_desired = np.linalg.inv(R_shaft)@R_desired
                #print("R_desired_wrist: \n", R_wrist_desired)
                
                
                
                
                joint_angles = [roll,gamma,beta,alpha] #initial orientation of pseudojoints
                joint_angles = IK_update(R_wrist_desired,joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],printout)
                
                #if printout is True: print("Notch Joint Angles(roll, gamma, beta, alpha): \n", joint_angles,"\n")
                
                R_wrist_IK = get_R_fullwristmodel(joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
                #if printout is True: print("Shaft Orientation: \n", R_shaft)
                #if printout is True: print("R_wrist_current: \n", R_wrist_IK)
                #if printout is True: print("R_desired_wrist: \n", R_wrist_desired)
                R_updated = R_shaft@R_wrist_IK
                if printout is True: print("R_current: \n", np.around(R_updated,4))
                if printout is True: print("R_desired: \n", np.around(R_desired,4))

                """ convert from pseudojoint values to cable displacements """
                deltaCablesGamma = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[1], "0")
                deltaCablesBeta = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[2], "120")
                deltaCablesAlpha = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[3], "240")
                
                deltaCablesTotal = 3*abs(deltaCablesGamma + deltaCablesAlpha + deltaCablesBeta)

                if printout is True: print("cable deltas for notch 1: ", deltaCablesGamma)
                if printout is True: print("cable deltas for notch 2: ", deltaCablesBeta)
                if printout is True: print("cable deltas for notch 3: ", deltaCablesAlpha)
                if printout is True: print("total cable delta: ", deltaCablesTotal)

                """ convert cable displacements to dial positions """
                """ [roll (joint space), EE jaw angle (joint space), cable 1 (cable space), cable 2 (cable space), cable 3 (cable space)] """
                DiskAngles = get_Disk_Angles(joint_angles[0], desired_EE_pinch_angle,
                                             deltaCablesTotal[0], deltaCablesTotal[1], deltaCablesTotal[2],
                                             disk_positions[1],
                                             self.cable_to_disk_map, self.eecable_to_disk_map, self.config_yaml)
                
                joints_list = psm_joints + DiskAngles
                if printout is True: print("Disk Angles: \n", np.around(joints_list,4))
                return joints_list, joint_angles

        def compute_ik(self, T_tip_0_mat, current_joint_positions, ee_metadata = ()):
                if len(ee_metadata) > 0:
                    joints_list, _ = self._compute_all_ik(T_tip_0_mat, current_joint_positions, ee_metadata[0])
                else:
                    joints_list, _ = self._compute_all_ik(T_tip_0_mat, current_joint_positions, None)
                return joints_list

        def actuator_to_joint(self, actuator_positions):
            _, jaw_angle, joint_values = self._compute_all_fk(actuator_positions)
            joint_positions = np.copy(actuator_positions)
            joint_positions[5] = jaw_angle
            # TODO, put rest of joint values in correct place
            return joint_positions

#----------------------------------------------------------------------------------------------------------------------------------------------#
### setup ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        
        def get_active_joint_names(self):
                return ["outer_yaw", "outer_pitch", "outer_insertion", \
                        "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw", "jaw"]

        def get_link_names(self):
                return list(self.kinematics_data.link_name_to_dh.keys())
        

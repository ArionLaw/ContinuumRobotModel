from dvrk_planning.kinematics.utils import *
from dvrk_planning.kinematics.TaskSpace_to_JointSpace import *
from dvrk_planning.kinematics.JointSpace_to_CableSpace import *
from dvrk_planning.kinematics.CableSpace_to_DiskSpace import *
from dvrk_planning.kinematics.plotting import *

from dvrk_planning.kinematics.psm import *
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
printout = True
getCabletoDiskMapping()

class Peter_Francis_tool_Kinematics_Solver:
#----------------------------------------------------------------------------------------------------------------------------------------------#
# wrist parameters to be placed in YAML
#----------------------------------------------------------------------------------------------------------------------------------------------#
        def __init__(self):
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

                #self.kinematics_data = PsmKinematicsData(spherical_wrist_tool_params)
                #self.negate_joint_list = spherical_wrist_tool_params.negate_joint_list
#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        def compute_fk(self, joints):
                """
                compute from disk space angles to cable space displacements
                        from cable space displacements to joint space angles 
                        from joint space angles to task space poses                 
                """        
                if printout is True:
                        print("\n FK")
                psm_joints = joints[0:3]
                disk_positions = joints[3:]
                print("joints", joints)
                # from disk space angles to joint space angles
                joint_values = DiskPosition_To_JointSpace(disk_positions,self.h,self.y_,self.r)
                #print("PSM Joint Values(yaw,pitch,insertion): \n",psm_joints)
                #print("Instrument Joint Values: \n" , joint_values)
                roll = joint_values[0]
                EE_grip = joint_values[1]
                gamma = joint_values[2]
                beta = joint_values[3]
                alpha = joint_values[4]

                # wrist position FK
                EE_pos_FK = get_wristPosition_from_PSMjoints(psm_joints)
                #print("Wrist Position: \n", EE_pos_FK)

                # orientation FK
                R_shaft = get_R_shaft(psm_joints)
                R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
                R_currentFK = R_shaft@R_wrist
                #print("Shaft Orientation: \n", R_shaft)
                #print("Wrist Orientation: \n", R_wrist)
                if printout is True:
                        print("Current EE Orientation: \n", R_currentFK)

                return ConvertToTransformMatrix(R_currentFK,EE_pos_FK)

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
    

        def compute_ik(self, tf_desired, disk_positions):
                """
                compute from task space poses to joint space angles
                        from joint space angles to cable space displacements
                        from cable space displacements to disk space angles
                """
                if printout is True:
                        print("\n IK")     
                PSM_wrist_pos_desired = tf_desired[0:3, 3]
                R_desired = tf_desired[0:3, 0:3]
                ### calculate current wrist pose 
                joint_values = DiskPosition_To_JointSpace(disk_positions,self.h,self.y_,self.r)
                print("Instrument Joint Values: \n" , joint_values)
                
                roll = joint_values[0]
                EE_pinch = joint_values[1]
                gamma = joint_values[2]
                beta = joint_values[3]
                alpha = joint_values[4]

                ### wrist position IK
                psm_joints = get_PSMjoints_from_wristPosition(PSM_wrist_pos_desired)
                #print("PSM Joint Values(yaw,pitch,insertion): \n", psm_joints)

                ### shaft orientation FK for current position
                # R_desired = calc_R_desired(EE_orientation_desired)
                R_shaft = get_R_shaft(psm_joints)

                ### wrist orientation FK for current position
                R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
                R_currentFK = R_shaft@R_wrist
                #print("Desired Orientation: \n", np.around(R_desired,4))
                #print("Shaft Orientation: \n", np.around(R_shaft,4))
                #print("Wrist Orientation: \n", np.around(R_wrist,4))
                #print("Current EE Orientation: \n", np.around(R_currentFK,4))

                ### EE_orientation IK
                R_wrist_desired = np.linalg.inv(R_shaft)@R_desired
                #print("R_desired_wrist: \n", R_wrist_desired)
                joint_angles = [roll,gamma,beta,alpha]
                joint_angles = IK_update(R_wrist_desired,joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3],printout)
                
                print("Notch Joint Angles(roll, gamma, beta, alpha): \n", joint_angles)
                
                R_wrist_IK = get_R_fullwristmodel(joint_angles[0],joint_angles[1],joint_angles[2],joint_angles[3])
                #print("R_wrist_IK: \n", R_wrist_IK)
                #print("R_desired_wrist: \n", R_wrist_desired)
                R_updated = R_shaft@R_wrist_IK
                #print("R_full_IK: \n", R_updated)
                #print("R_desired: \n", R_desired)

                # convert from joint angles to cable displacements
                deltaCablesGamma = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[1], "0")
                deltaCablesBeta = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[2], "240")
                deltaCablesAlpha = get_deltaCable_at_Notch(self.h, self.y_, self.r, self.w, joint_angles[3], "120")
                deltaCablesTotal = 3*abs(deltaCablesGamma + deltaCablesBeta + deltaCablesAlpha)
                
                #EE_pinch = 1 #place holder value for now, need to obtain from ROS topic

                print("cable deltas for notch 1: ", deltaCablesGamma)
                print("cable deltas for notch 2: ", deltaCablesBeta)
                print("cable deltas for notch 3: ", deltaCablesAlpha)
                print("total cable delta: ", deltaCablesTotal)

                # get Disk Angle inputs for PSM tool base 
                # [roll (joint space), end effector actuation (joint space), cable 1 (cable space), cable 2 (cable space), cable 3 (cable space)]
                DiskAngles = getDiskAngles(joint_angles[0],EE_pinch,deltaCablesTotal[0],deltaCablesTotal[1],deltaCablesTotal[2])
                joints_list = psm_joints + DiskAngles
                if printout is True:
                        print("Disk Angles: \n", np.around(DiskAngles,4))
                return joints_list
#----------------------------------------------------------------------------------------------------------------------------------------------#
### setup ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
        
        def get_active_joint_names(self):
                return ["outer_yaw", "outer_pitch", "outer_insertion", \
                        "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw", "jaw"]

        def get_link_names(self):
                return list(self.kinematics_data.link_name_to_dh.keys())
        
#----------------------------------------------------------------------------------------------------------------------------------------------#
### non-Linux Testing ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
# configurations for testing FK and IK
"""
Disk Angles:
 [[3.2833419526133314, -1, 0.1117424042942665, -0.15561920043570215]]
psm_yaw = 0.7854;
psm_pitch = -0.6155;
psm_insertion = 43.3013;
psm_joints = [psm_yaw, psm_pitch, psm_insertion]

result:

PSM_wrist_pos_desired = [25,25,-25] #in mm
psm_yaw = 0.7854;
psm_pitch = -0.6155;
psm_insertion = 43.3013;
"""
# test run
"""
printout = True
EE_orientation_desired = np.array([-0.25,-0.25,-0.5])#np.array([1,1,0])
tip_desired = 0 #in degrees
PSM_wrist_pos_desired = [-25,-25,-25] #[25,25,-25] #in mm
disk_positions = [0,0,0,0]

tool1 = Peter_Francis_tool_Kinematics_Solver()
tool1.compute_ik(PSM_wrist_pos_desired , EE_orientation_desired, tip_desired, disk_positions)

disk_positions = [3.2833419526133314, -1, 0.1117424042942665, -0.15561920043570215]
psm_yaw = 0.7854
psm_pitch = -0.6155
psm_insertion = 43.3013
psm_joints = [psm_yaw, psm_pitch, psm_insertion]

tool1.compute_fk(disk_positions,psm_joints)
"""
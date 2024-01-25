from RPR_task_space_to_joint_space import *
from RPR_joint_space_to_cable_space import *
from RPR_cable_space_to_disk_space import *
from utils import *

#from dvrk_ctr_teleop.kinematics.RPR_task_space_to_joint_space import *
#from dvrk_ctr_teleop.kinematics.RPR_joint_space_to_cable_space import *
#from dvrk_ctr_teleop.kinematics.RPR_cable_space_to_disk_space import *
#from dvrk_ctr_teleop.kinematics.utils import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
# Notes
#----------------------------------------------------------------------------------------------------------------------------------------------#
'''

'''
np.set_printoptions(precision=3)
printout = False

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
    def compute_all_fk(self, joints):
            """
            compute from disk space angles to cable space displacements
                    from cable space displacements to joint space angles 
                    from joint space angles to task space poses                 
            """        
            if printout is True: print("------------------------------------------- FK -------------------------------------------")
            psm_joints = joints[0:3]
            disk_positions = joints[3:]
            #if printout is True: print("joints", joints)

            """ from disk space angles to cable space displacements to joint space angles """

            outer_roll,pitch,inner_roll = get()

            """ wrist position FK """
            EE_pos_FK = get_wristPosition_from_PSMjoints(psm_joints)
            #if printout is True: print("Wrist Position: \n", EE_pos_FK)

            """ orientation FK """
            R_shaft = get_R_shaft(psm_joints)
            R_wrist = get_R_wrist(outer_roll,pitch,inner_roll)
            R_currentFK = R_shaft@R_wrist
            if printout is True: print("Shaft Orientation: \n", R_shaft)
            if printout is True: print("Wrist Orientation: \n", R_wrist)
            if printout is True: print("Current EE Orientation: \n", R_currentFK)

            return ConvertToTransformMatrix(R_currentFK,EE_pos_FK)

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
    def compute_ik(self, tf_desired, direct_psm_and_disk_joint_positions, desired_EE_pinch_angle):
            """
            compute from task space poses to joint space angles
                    from joint space angles to cable space displacements
                    from cable space displacements to disk space angles
            """
            if printout is True: print("------------------------------------------- IK -------------------------------------------")     
            
            PSM_wrist_pos_desired = tf_desired[0:3, 3]
            R_desired = tf_desired[0:3, 0:3]
            disk_positions = direct_psm_and_disk_joint_positions[3:]

            """ FK calculate wrist pseudojoints of current pose using """ 
            joint_values = DiskPositions_To_JointSpace(disk_positions,self.h,self.y_,self.r)

            """ IK calculate psm joints to obtain wrist cartesian position """
            psm_joints = get_PSMjoints_from_wristPosition(PSM_wrist_pos_desired)
            #if printout is True: print("PSM Joint Values(yaw,pitch,insertion):\n", psm_joints)

            """ FK calculate current shaft orientation given wrist cartesian position """
            # R_desired = calc_R_desired(EE_orientation_desired)
            R_shaft = get_R_shaft(psm_joints)

            """ FK calculate current wrist orientation of current pose """
            R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
            R_currentFK = R_shaft@R_wrist

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
### Test Case Debugging ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
# configurations for testing FK and IK

# run test cases
"""
def run_test_cases():
        input_filename = "+rot_x.txt"
        input_current_output_js_list,tf_matrices_list = read_TestCaseFile(input_filename)

        original_stdout = sys.stdout
        log_filename = input_filename + '_testcaselog.txt'
        with open(log_filename,'w') as f:
                sys.stdout = f

                for i in range(len(input_current_output_js_list)):
                        print("============================================================================================================================")
                        print("iteration: ", i)
                        disk_positions = input_current_output_js_list[i]
                        #print("Disk Positions:\n", disk_positions)
                        tf_desired = np.matrix(tf_matrices_list[i])
                        #print("tf Desired:\n",tf_desired)

                        tool1 = Peter_Francis_tool_Kinematics_Solver()
                        
                        #Transform, jaw angle and wrist joint angle as calculated from FK given input_current_output_js
                        Tf, jaw_angle, FK_joint_values = tool1.compute_all_fk(disk_positions)

                        #dial values and wrist joint angle as calculated from IK given log file Tf desired and current joint and dial positions
                        dialvalues, IKpre_joint_values = tool1.compute_all_ik(tf_desired, disk_positions, 30*np.pi/180) 

                        #Transform, jaw angle and wrist joint angle as calculated from FK given dial values calculated from IK
                        Tf, jaw_angle, IKpost_joint_values = tool1.compute_all_fk(dialvalues)
                        
                        input_joint_values = [FK_joint_values[0],FK_joint_values[2],FK_joint_values[3],FK_joint_values[4]]
                        IKpost_joint_values = [IKpost_joint_values[0],IKpost_joint_values[2],IKpost_joint_values[3],IKpost_joint_values[4]]
                        print("____________________________________________________________________________________________________________________________")
                        print("input current output js wrist joint values (roll,gamma,beta,alpha): \n" , input_joint_values)
                        print("IK wrist joint values pre-cable allocation (roll,gamma,beta,alpha): \n" , IKpre_joint_values)
                        print("IK wrist joint values post-cable allocation (roll,gamma,beta,alpha): \n" , IKpost_joint_values)

                        joint_difference = []
                        for i in range(len(IKpost_joint_values)):
                                joint_difference.append( IKpost_joint_values[i] - IKpre_joint_values[i])
                        print("cable allocation error(roll,gamma,beta,alpha): \n" , joint_difference)
                sys.stdout = original_stdout
        print("finished")
"""
#run_test_cases()


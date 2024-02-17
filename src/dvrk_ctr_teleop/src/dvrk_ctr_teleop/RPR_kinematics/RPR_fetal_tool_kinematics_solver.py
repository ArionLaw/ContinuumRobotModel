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

#----------------------------------------------------------------------------------------------------------------------------------------------#
# Notes
#----------------------------------------------------------------------------------------------------------------------------------------------#
'''

'''
np.set_printoptions(precision=3)
printout = False

class Arion_Law_tool_Kinematics_Solver:
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
            self.WristIKSolutionSelector = WristIKSolutionSelector(config_yaml["wrist_pitch_joint_limits"])
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
            if printout is True: print("------------------------------------------- FK -------------------------------------------")
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
                  q4, q5, q6, EE_pinch_angle = self.CableToDiskSpaceSolver.DiskPosition_To_JointSpace(disk_positions,self.h,
                                                                                                      self.y_,self.r,self.n)


            if printout is True: print("Wrist Position: \n", EE_pos_FK)

            """ orientation FK """
            R_shaft = get_R_shaft(psm_joints)
            R_wrist = get_R_wrist(q4,q5,q6)
            R_currentFK = R_shaft@R_wrist
            if printout is True: print("Shaft Orientation: \n", R_shaft)
            if printout is True: print("Wrist Orientation: \n", R_wrist)
            if printout is True: print("Current EE Orientation: \n", R_currentFK)

            """ wrist position FK """
            wrist_pos_FK = get_wristPosition_from_PSMjoints(psm_joints, self.wrist_length)
            EE_pos_FK = wrist_pos_FK + self.wrist_length/2*R_wrist@np.array([0,0,1])

            #print('FK JOINTS:', joints)

            return ConvertToTransformMatrix(R_currentFK,EE_pos_FK), EE_pinch_angle

    def compute_fk(self, joint_positions):
            tf, _ = self._compute_all_fk(joint_positions)
            return tf
        

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
            
            ee_position_desired = tf_desired[0:3,3]
            R_desired = tf_desired[0:3, 0:3]

            if self.simulation:
                q4 = direct_psm_and_disk_joint_positions[3]
                q5 = direct_psm_and_disk_joint_positions[4]*6 #pitch
                q6 = direct_psm_and_disk_joint_positions[10] #inner roll
                current_wrist_angles = [q4,q5,q6]

                if not isinstance(desired_EE_pinch_angle,float):
                       desired_EE_pinch_angle = desired_EE_pinch_angle[0]

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
            #if printout is True: print("PSM Joint Values(yaw,pitch,insertion):\n", psm_joints)

            """ FK calculate current shaft orientation given wrist cartesian position"""
            R_shaft = get_R_shaft(psm_joints)
            R_wrist_current = get_R_wrist(q4,q5,q6)

            """IK calculate wrist joint solutions and select the best solution"""
            R_wrist = np.linalg.inv(R_shaft)@R_desired
            wrist_ik_sols = wrist_analytical_ik(R_wrist,R_wrist_current,self.R_wrist_previous)
            q4,q5,q6= self.WristIKSolutionSelector.select_best_solution(current_wrist_angles, wrist_ik_sols).tolist()
        #     print('wrist_ik',wrist_ik_sols)
        #     print('current_configuration:', current_wrist_angles)
        #     print('best_solution', [q4,q5,q6])

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
                DiskAngles = self.CableToDiskSpaceSolver.get_Disk_Angles(q4,q5,q6,desired_EE_pinch_angle,
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
            joint_positions[11] = jaw_angle
            # TODO, put rest of joint values in correct place
            return joint_positions

        
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


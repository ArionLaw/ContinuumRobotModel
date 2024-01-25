import numpy as np
import sys

from dvrk_ctr_teleop.kinematics.fetal_tool_kinematics_solver import PeterFrancisToolKinematicsSolver
from dvrk_ctr_teleop.RPR_kinematics.RPR_fetal_tool_kinematics_solver import ArionLawToolKinematicsSolver
from test_case_reader import read_TestCaseFile

#----------------------------------------------------------------------------------------------------------------------------------------------#
### Test Case Debugging ###
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

# run test cases
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

                        tool1 = PeterFrancisToolKinematicsSolver()
                        tool1 = ArionLawToolKinematicsSolver()
                        
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

run_test_cases()

"""
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
shaft_length = 400 #mm

# testing from pseudojoints to delta cables to dial values (IK)
print("------------------------------------------- IK -------------------------------------------")  
psm_joints = [0,0,0.1]
roll = 0
desired_EE_pinch_angle = 0
gamma = 20*np.pi/180
beta = 20*np.pi/180
alpha = 0

tool1 = PeterFrancisToolKinematicsSolver()

joint_angles = [roll,gamma,beta,alpha]
print("pseudojoint angles: \n [roll,gamma,beta,alpha]\n",joint_angles)
deltaCablesGamma = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[1], "0")
deltaCablesBeta = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[2], "120")
deltaCablesAlpha = get_deltaCable_at_Notch(h, y_, r, w, joint_angles[3], "240")
deltaCablesTotal = 3*abs(deltaCablesGamma + deltaCablesAlpha + deltaCablesBeta)

if printout is True: print("cable deltas for notch 1: ", deltaCablesGamma)
if printout is True: print("cable deltas for notch 2: ", deltaCablesBeta)
if printout is True: print("cable deltas for notch 3: ", deltaCablesAlpha)
if printout is True: print("total cable delta: ", deltaCablesTotal)

# convert cable displacements to dial positions
# [roll (joint space), EE jaw angle (joint space), cable 1 (cable space), cable 2 (cable space), cable 3 (cable space)]
DiskAngles = get_Disk_Angles(joint_angles[0],desired_EE_pinch_angle,deltaCablesTotal[0],deltaCablesTotal[1],deltaCablesTotal[2])
joints_list = psm_joints + DiskAngles
if printout is True: print("Disk Angles: \n", np.around(joints_list,4))

print("\n------------------------------------------------------------------------------------------")  
print(joints_list)
print("------------------------------------------------------------------------------------------\n")  

""""""
# testing from dial/joint values to cable displacement to pseudojoints (FK)
print("------------------------------------------- FK -------------------------------------------")  
joints = joints_list#[ 0, 0, 0.1, 0, -0.347, 0.158, 0.158]
psm_joints = joints[0:3]
disk_positions = joints[3:]
#print(joints)

# from disk space angles to joint space angles
joint_values = DiskPosition_To_JointSpace(disk_positions,h,y_,r)
#if printout is True: print("PSM Joint Values(yaw,pitch,insertion): \n",psm_joints)
print("Instrument Joint Values(roll, EE jaw, gamma, beta, alpha):  \n" , joint_values, "\n")

WristAngleError = np.array([joint_values[0],joint_values[2],joint_values[3],joint_values[4]]) - np.array(joint_angles)
print("WristAngleError: \n" , WristAngleError)
"""

"""
disk_positions = [3.2833419526133314, -1, 0.1117424042942665, -0.15561920043570215]
psm_yaw = 0.7854
psm_pitch = -0.6155
psm_insertion = 43.3013
psm_joints = [psm_yaw, psm_pitch, psm_insertion]

tool1.compute_fk(disk_positions,psm_joints)
"""

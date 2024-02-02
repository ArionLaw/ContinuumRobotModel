from dvrk_ctr_teleop.RPR_kinematics.utils import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_task_space_to_joint_space import *

import numpy as np
import math

def main():
    
    psm_joints_fk = [np.pi/4, np.pi/8, 0.6] # Example joint values: yaw, pitch, insertion
    q4 = np.pi/2
    q5 = 0.3452
    q6 = 0.9954

    wristlength = 0.10  # Example wrist length

    # Compute FK to get desired end-effector position and orientation
    desired_EE_pos = get_wristPosition_from_PSMjoints(psm_joints_fk, wristlength)
    desired_R_wrist = get_R_wrist(q4,q5,q6)  # Example joint values for outer_roll, pitch_angle, inner_roll
    desired_R_shaft = get_R_shaft(psm_joints_fk)
    print("Desired R_shaft:", desired_R_shaft)
    

    # Use the FK outputs as inputs to the IK functions
    psm_joints_ik = get_PSMjoints_from_wristPosition(desired_EE_pos, wristlength)
    wrist_ik_solutions = wrist_analytical_ik(desired_R_wrist)

    # Define the wrist pitch joint limits for IK
    wrist_pitch_joint_limits = [0, np.pi/2]  # Example joint limits for wrist pitch

    # Initialize the IK solution selector
    selector = WristIKSolutionSelector(wrist_pitch_joint_limits)

    # Select the best IK solution
    q_current = np.array([0,0.20, 0])  # Current joint values
    best_solution = selector.select_best_solution(q_current, wrist_ik_solutions).tolist()

    print("Desired End-Effector Position:", desired_EE_pos)
    print("IK PSM Angles:", psm_joints_ik)
    print("Solution PSM Position:", get_wristPosition_from_PSMjoints(psm_joints_ik, wristlength))
    print("IK Solutions:", wrist_ik_solutions)
    print("Selected Best Solution:", best_solution)
    print("R_Desired:", desired_R_wrist)
    print("R_IK_Solution:", get_R_wrist(best_solution[0], best_solution[1], best_solution[2]))

if __name__ == "__main__":
    main()
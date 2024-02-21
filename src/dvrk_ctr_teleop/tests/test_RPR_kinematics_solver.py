from dvrk_ctr_teleop.RPR_kinematics.utils import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_task_space_to_joint_space import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_fetal_tool_kinematics_solver import Arion_Law_tool_Kinematics_Solver

import numpy as np
import math

def main():
    print("TEST CASE 1>>>>>>>> NO CLUTCH")
    solver_class = Arion_Law_tool_Kinematics_Solver("../config/mapping/psm1_tool_rpr.yaml")
    tf_desired = np.array([[-0.981, -0.028,  0.191,  0.021],
                            [ 0.105,  0.754,  0.648,  0.116],
                            [-0.162,  0.656, -0.737, -0.086],
                            [ 0.,     0.,     0.,     1.   ]])
    
    direct_psm_and_disk_joint_positions = np.array([0.24499228716731508, -0.9257644417192956, 
                                                    0.1450698907261067, 0.04973191502900963, 
                                                    -0.3586000740104184, 0.25598925331853356, -0.0546506512791877])
    
    ee_pinch_angle = 0.7853981633974483

    joints_list = solver_class.compute_ik(tf_desired,direct_psm_and_disk_joint_positions,ee_pinch_angle)
    print(joints_list)

    fk_solution = solver_class.compute_fk(joints_list)
    print('fksolution:', fk_solution)

    print("TEST CASE 2>>>>>>> CLUTCH ON")

    solver_class = Arion_Law_tool_Kinematics_Solver("../config/mapping/psm1_tool_rpr.yaml")
    tf_desired = np.array([[-0.988, -0.029,  0.152,  0.022],
                                                    [ 0.077,  0.762,  0.643,  0.115],
                                                    [-0.135,  0.647, -0.751, -0.084],
                                                    [ 0.,     0.,    0.,    1.   ]])
    
    
    direct_psm_and_disk_joint_positions = np.array([0.24549847238722045, -0.9256248441068788, 0.14500035227578906, 
                                           0.04455738906973605, -0.3578098670726394, 0.25142632478182886, -0.054530223741870186])
    ee_pinch_angle = 0.7853981633974483

    joints_list = solver_class.compute_ik(tf_desired,direct_psm_and_disk_joint_positions,ee_pinch_angle)
    print(joints_list)
    

    fk_solution = solver_class.compute_fk(joints_list)
    print('fksolution:', fk_solution)

    print("TEST CASE 3>>>>>>> CLUTCH ON")

    solver_class = Arion_Law_tool_Kinematics_Solver("../config/mapping/psm1_tool_rpr.yaml")
    tf_desired = np.array([[-0.935, -0.17,   0.313,  0.02 ],
                           [ 0.035,  0.831,  0.555,  0.113],
                           [-0.354,  0.53,  -0.771, -0.086],
                           [ 0.,     0.,     0.,     1.   ]])
    
    direct_psm_and_disk_joint_positions = np.array([0.22616705536368295, -0.9162744562115276, 0.14187200190246138, 
                                           0.6224012941045788, -0.535186471523649, 0.51495766582678, -0.08156241826020402])
    ee_pinch_angle = 0.7853981633974483

    joints_list = solver_class.compute_ik(tf_desired,direct_psm_and_disk_joint_positions,ee_pinch_angle)
    print(joints_list)
    

    fk_solution = solver_class.compute_fk(joints_list)
    print('fksolution:', fk_solution)

    print("TEST CASE 4>>>>>>> CLUTCH ON")

    solver_class = Arion_Law_tool_Kinematics_Solver("../config/mapping/psm1_tool_rpr.yaml")
    tf_desired = np.array([[-0.975, -0.209,  0.078,  0.02],
                           [-0.133,  0.825,  0.549, 0.112],
                           [-0.179,  0.524, -0.832, -0.083],
                           [ 0.,     0.,     0.,     1.]])
    
    direct_psm_and_disk_joint_positions = np.array([0.22619579454811573, -0.9165469536102843, 0.14181929340126126, 0.6216236373597729, -0.5355268440260721, 0.5141577985306195, -0.0816142910295733])
    ee_pinch_angle = 0.7853981633974483

    joints_list = solver_class.compute_ik(tf_desired,direct_psm_and_disk_joint_positions,ee_pinch_angle)
    print(joints_list)
    

    fk_solution = solver_class.compute_fk(joints_list)
    print('fksolution:', fk_solution)
    
if __name__ == "__main__":
    main()
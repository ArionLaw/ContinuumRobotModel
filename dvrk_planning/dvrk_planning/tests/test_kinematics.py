from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
from timer import Timer

import numpy as np

np.set_printoptions(precision = 4, suppress = True)

t = Timer(timer_repeat_times = 100)
p = PsmKinematicsSolver(LND400006())

def test_fk_ik(joint_pos, test_time = False):
    if(test_time):
        output_fk = t.time_average(lambda : p.compute_fk(joint_pos))
        fk_avg_time = t.str_average()
        output_jp = t.time_average(lambda : p.compute_ik(output_fk))
        ik_avg_time = t.str_average()
    else:
        output_fk = p.compute_fk(joint_pos)
        output_jp = p.compute_ik(output_fk)  
      
    # Print results
    print("Input joint pos: \n", joint_pos)
    print("FK: \n", output_fk)
    print("IK: \n", np.array(output_jp))
    if(test_time):
        print("FK time: ", fk_avg_time)
        print("IK time: ", ik_avg_time)

    print("-------------------------------------")

joint_change = 0.1

joint_pos = [0.0, 0.0, joint_change, 0.0, 0.0, 0.0]
test_fk_ik(joint_pos)

joint_pos[1] = -joint_change
test_fk_ik(joint_pos)

joint_pos[0] = joint_change
test_fk_ik(joint_pos)

joint_pos[3] = joint_change
test_fk_ik(joint_pos)

joint_pos[4] = joint_change
test_fk_ik(joint_pos)

joint_pos[5] = joint_change
test_fk_ik(joint_pos, test_time= True)

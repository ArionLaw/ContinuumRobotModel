import numpy as np
from timer import Timer
from dvrk_planning.kinematics.utilities import get_harmonized_joint_positions, TWO_PI

t = Timer(timer_repeat_times = 100)

joint_positions = np.array([1, 2, 3])
reference_joint_positions = np.array([1.1, 2.1 + TWO_PI, 3.2 - TWO_PI])

harmonized_joint_positions = t.time_average(lambda : get_harmonized_joint_positions(joint_positions, reference_joint_positions))

print("joint_positions: ", joint_positions)
print("reference_joint_positions: ", reference_joint_positions)
print("harmonized_joint_positions: ", harmonized_joint_positions)
print(t.str_average())

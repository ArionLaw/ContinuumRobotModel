from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
from timer import Timer
import numpy as np

def print_fk(fk, target_frame):
    print("target_frame: ", target_frame)
    print("FK: \n", fk)

np.set_printoptions(precision = 4, suppress = True)

t = Timer(timer_repeat_times = 1)
p = PsmKinematicsSolver(LND400006())

# 1. Link names
print("link_names:", p.get_link_names())

# 2. Going up the default chain calculate from base
reference_frame = ""

target_frame = "yaw_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print(len(joint_pos))
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

target_frame = "pitch_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

target_frame = "main_insertion_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

target_frame = "tool_roll_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

target_frame = "tool_pitch_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

target_frame = "tool_yaw_link"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
print_fk(p.compute_fk_relative(joint_pos, chain), target_frame)

# 3. For last one, calculate with a timer
target_frame = "tool_tip"
chain = p.get_chain(reference_frame, target_frame)
joint_pos = np.zeros(chain.get_num_of_active_joints())
output_fk = t.time_average(lambda : p.compute_fk_relative(joint_pos, chain))
print_fk(output_fk, target_frame)
print(t.str_average())

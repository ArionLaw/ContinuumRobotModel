import numpy as np

TWO_PI = 2 * np.pi

# Must be same length, not going to check here
def get_harmonized_joint_positions(joint_positions, reference_jps):
    abs_joint_diffs = np.absolute(reference_jps - joint_positions)
    sign_diffs = np.sign(reference_jps - joint_positions)
    shifted_two_pi_jps = sign_diffs * TWO_PI + joint_positions
    shifted_abs_joint_diffs = np.absolute(reference_jps - shifted_two_pi_jps)
    return np.where(abs_joint_diffs < shifted_abs_joint_diffs, joint_positions, shifted_two_pi_jps)

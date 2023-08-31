import time
import numpy as np
from dvrk_planning.controller.teleop_controller import TeleopController, InputType, ControllerType

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3,3]
    return rot, p.flatten()

class JointTeleopController(TeleopController):
    def __init__(self, input_type):
        super().__init__(input_type, ControllerType.JOINT)

    def update(self, *args):
        if(not self._update()):
            return False

        output_js = self._update_impl(args)
        self.output_callback(output_js)
        return True

    def _update_impl(self, args):
        raise NotImplementedError

def interpolate_joint_path(start_jp, goal_jp, increment):
    joint_path = np.arange(start_jp, goal_jp, increment)
    joint_path = np.append(joint_path, goal_jp)
    return joint_path

class JointFollowTeleopController(JointTeleopController):
    def __init__(self, scale = 1.0):
        super().__init__(InputType.FOLLOW)
        self.scale = scale

    def execute_on_output_callback(self, joint_path, joint_state, idx, hz):
        joint_state_copy = np.copy(joint_state)
        for jp in joint_path:
            joint_state_copy[idx] = jp
            self.output_callback(joint_state_copy)
            time.sleep(1.0/hz)
        return joint_state_copy

    def execute_match_joint_states(self, joint_state, match_to_joint_state):
        updated_output_jps = np.copy(joint_state)
        joint_too_far_tol = 0.1
        for i in range(len(match_to_joint_state)):
            if abs(match_to_joint_state[i] - updated_output_jps[i]) > joint_too_far_tol:
                joint_path_inc = interpolate_joint_path(
                    updated_output_jps[i],
                    match_to_joint_state[i],
                    joint_too_far_tol/4)
                updated_output_jps = self.execute_on_output_callback(
                        joint_path_inc, updated_output_jps, i, 50)

    def enable(self, start_input_jps, start_output_jps):
        scaled_input_jps = self.scale * np.array(start_input_jps)
        self.execute_match_joint_states(start_output_jps, scaled_input_jps)
        super()._enable()

    def unclutch(self, start_input_jps, start_output_jps):
        scaled_input_jps = self.scale * np.array(start_input_jps)
        self.execute_match_joint_states(start_output_jps, scaled_input_jps)
        super()._unclutch()

    """
    Parameters
    ----------
    absolute_input_js : ndarray
        1D array containing data with `float` type.
    """
    def __update_input_js(self, absolute_input_jps):
        absolute_output_jps = self.scale * np.array(absolute_input_jps)
        return absolute_output_jps

    def _update_impl(self, args):
        return self.__update_input_js(*args)

class JointIncrementTeleopController(JointTeleopController):
    def __init__(self):
        super().__init__(InputType.INCREMENT)
        self.current_output_jps = np.array([])

    def enable(self, current_output_jps):
        self.current_output_jps = np.copy(current_output_jps)
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    """
    Parameters
    ----------
    increment_js : ndarray
        1D array containing data with `float` type.
        Must have same length as self.current_output_jps from enable
    """
    def __increment_input_js(self, increment_js):
        if(len(increment_js) != len(self.current_output_jps)):
            print("Length input js not the same as current_output_jps")
            return self.current_output_jps
        self.current_output_jps = self.current_output_jps + increment_js
        return self.current_output_jps

    def _update_impl(self, args):
        return self.__increment_input_js(*args)

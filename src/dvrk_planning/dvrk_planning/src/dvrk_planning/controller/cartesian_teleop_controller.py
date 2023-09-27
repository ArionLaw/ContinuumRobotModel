import numpy as np
# Should remove this if kinematics don't use
from PyKDL import Frame, Rotation, Vector

from dvrk_planning.controller.teleop_controller import TeleopController, InputType, ControllerType
from dvrk_planning.utilities import convert_frame_to_mat

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3,3]
    return rot, p.flatten()

class CartesianTeleopController(TeleopController):
    def __init__(self,
            input_type,
            kinematics_solver,
            output_ref_to_input_rot = Rotation.Quaternion(0, 0, 0, 1)):
        super().__init__(input_type, ControllerType.CARTESIAN)
        self.output_ref_to_input_rot = convert_frame_to_mat(Frame(output_ref_to_input_rot, Vector(0.0, 0.0, 0.0)))
        self.kinematics_solver = kinematics_solver
        self.current_output_js = np.array([])

    def _rotation_adjustment(self, input_diff_tf):
        return self.output_ref_to_input_rot[0:3,0:3].dot(input_diff_tf[0:3,3]).flatten()

    def _update_output_tf(self, input_diff_tf, output_tf):
        input_diff_rotated_p = self._rotation_adjustment(input_diff_tf)
        input_diff_rot, _ = get_rot_and_p(input_diff_tf)
        cur_output_rot, cur_output_p = get_rot_and_p(output_tf)

        cur_output_p = cur_output_p + input_diff_rotated_p
        cur_output_rot = np.matmul(cur_output_rot, input_diff_rot)

        output_tf[0:3, 0:3] = cur_output_rot
        output_tf[0:3, 3] = cur_output_p

    def update(self, *args):
        if(not self._update()):
            return False

        absolute_output_tf = self._update_impl(args)
        self.current_output_js = self.kinematics_solver.compute_ik(absolute_output_tf)
        self.output_callback(self.current_output_js)
        return True

    def _update_impl(self, args):
        raise NotImplementedError

class CartesianFollowTeleopController(CartesianTeleopController):
    def __init__(self, kinematics_solver, output_ref_to_input_rot = Rotation.Quaternion(0, 0, 0, 1), position_scale = 1.0):
        super().__init__(InputType.FOLLOW, kinematics_solver, output_ref_to_input_rot)
        self.start_input_tf = np.identity(4)
        self.start_output_tf = np.identity(4)
        self.position_scale = position_scale

    def enable(self, start_input_tf, start_output_js):
        self.start_input_tf = np.copy(start_input_tf)
        self.current_output_js = np.copy(start_output_js)
        self.start_output_tf = self.kinematics_solver.compute_fk(start_output_js)
        super()._enable()

    # When unclutching:
    # Position of output tf stays the same
    # MTM adjusts rotation for current output rotation. Need to check, but that wont work for occulus, or haptic pen.
    # start_output_tf should be the current tf of the output when uncluthing
    def unclutch(self, start_input_tf, start_output_js):
        self.start_input_tf = np.copy(start_input_tf)
        self.current_output_js = np.copy(start_output_js)
        self.start_output_tf = self.kinematics_solver.compute_fk(start_output_js)
        super()._unclutch()

    def __update_input_tf(self, absolute_input_tf):
        input_tf_rot_diff = np.matmul(np.linalg.inv(self.start_input_tf), absolute_input_tf)

        input_tf_difference = np.identity(4)
        input_tf_difference[0:3,0:3] = input_tf_rot_diff[0:3, 0:3]
        input_tf_difference[0:3, 3] = (absolute_input_tf[0:3,3] - self.start_input_tf[0:3,3]) * self.position_scale

        absolute_output_tf = np.copy(self.start_output_tf)
        self._update_output_tf(input_tf_difference, absolute_output_tf)
        return absolute_output_tf

    def _update_impl(self, args):
        return self.__update_input_tf(*args)

class CartesianIncrementTeleopController(CartesianTeleopController):
    def __init__(self, kinematics_solver, output_ref_to_input_rot = Rotation.Quaternion(0, 0, 0, 1)):
        super().__init__(InputType.INCREMENT, kinematics_solver, output_ref_to_input_rot)
        self.current_output_tf = np.identity(4)

    def enable(self, current_output_js):
        self.current_output_js = np.copy(current_output_js)
        self.current_output_tf = self.kinematics_solver.compute_fk(self.current_output_js)
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    def __increment_input_tf(self, inc_position: Vector, inc_quaternion: Rotation):
        input_diff_tf = convert_frame_to_mat(Frame(inc_quaternion, inc_position))
        self._update_output_tf(input_diff_tf, self.current_output_tf)
        return self.current_output_tf

    def _update_impl(self, args):
        return self.__increment_input_tf(*args)

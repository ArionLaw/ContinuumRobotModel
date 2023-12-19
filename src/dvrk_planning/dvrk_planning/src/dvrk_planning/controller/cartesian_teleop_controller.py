import numpy as np
# Should remove this if kinematics don't use
from PyKDL import Frame, Rotation, Vector

from dvrk_planning.controller.teleop_controller import TeleopController, InputType, ControllerType
from dvrk_planning.utilities import convert_frame_to_mat

def get_rot_and_p(tf):
    rot = tf[0:3, 0:3]
    p = tf[0:3, 3]
    return rot, p.flatten('F')

class CartesianTeleopController(TeleopController):
    def __init__(self,
            input_type,
            kinematics_solver,
            input_2_input_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
            output_2_output_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
            desired_jaw_in_kinematics = False):
        super().__init__(input_type, ControllerType.CARTESIAN)
        # m: input base frame (master)
        # h: input reference frame
        # s: output base frame (slave)
        # e: output reference frame

        # p: position
        self.current_output_tf = np.identity(4)

        m2h_transform = convert_frame_to_mat(Frame(input_2_input_reference_rot, Vector(0.0, 0.0, 0.0)))
        self.h2m_transform = np.linalg.inv(m2h_transform)
        self.s2e_transform = convert_frame_to_mat(Frame(output_2_output_reference_rot, Vector(0.0, 0.0, 0.0)))

        self.kinematics_solver = kinematics_solver
        self.current_output_js = np.array([])

        self.desired_jaw_in_kinematics = desired_jaw_in_kinematics

    def _update_output_tf(self, input_diff_tf, current_output_tf):

        input_diff_rot, input_diff_p_wrt_m = get_rot_and_p(input_diff_tf)
        cur_output_rot, cur_output_p_wrt_s = get_rot_and_p(current_output_tf)

        # Solving position wrt input ref frame (h) and output camera frame (e)
        input_diff_p_wrt_h = np.matmul(self.h2m_transform[0:3, 0:3], input_diff_p_wrt_m)
        output_diff_p_wrt_e = input_diff_p_wrt_h
        output_diff_p_wrt_s = np.matmul(self.s2e_transform[0:3, 0:3], output_diff_p_wrt_e)
        output_p_wrt_s = cur_output_p_wrt_s + output_diff_p_wrt_s

        # Post multiply output to rotate about itself
        output_rot = np.matmul(cur_output_rot, input_diff_rot)

        output_tf = np.copy(current_output_tf)
        output_tf[0:3, 0:3] = output_rot
        output_tf[0:3, 3] = output_p_wrt_s

        return output_tf

    def update(self, *args):
        if(not self._update()):
            return False

        desired_jaw = None
        if(self.desired_jaw_in_kinematics):
            desired_jaw = args[-1]
        absolute_output_tf = self._update_impl(args)
        # print("self.input_current_output_js: ", np.around(self.current_output_js, 3))        
        #print("absolute_output_tf: ", np.around(absolute_output_tf, 3))
        self.current_output_js = self.kinematics_solver.compute_ik(absolute_output_tf, self.current_output_js, desired_jaw)
        # print("self.current_output_js: ", np.around(self.current_output_js, 3))
        # print("===============================================================================================================================")
        self.output_callback(self.current_output_js)
        return True

    def _update_impl(self, args):
        raise NotImplementedError

class CartesianFollowTeleopController(CartesianTeleopController):
    def __init__(self, kinematics_solver,
                 input_2_input_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
                 output_2_output_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
                 desired_jaw_in_kinematics = False,
                 position_scale = 1.0):
        super().__init__(InputType.FOLLOW, kinematics_solver,
                         input_2_input_reference_rot, output_2_output_reference_rot,
                         desired_jaw_in_kinematics)
        self.start_input_tf = np.identity(4)
        self.start_output_tf = np.identity(4)
        self.position_scale = position_scale

    def enable(self, start_input_tf, start_output_js):
        self.start_input_tf = np.copy(start_input_tf)
        self.current_output_js = np.copy(start_output_js)
        self.start_output_tf = self.kinematics_solver.compute_fk(start_output_js)
        self.current_output_tf = np.copy(self.start_output_tf)
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
        absolute_output_tf = self._update_output_tf(input_tf_difference, absolute_output_tf)
        self.current_output_tf = absolute_output_tf
        return absolute_output_tf

    def _update_impl(self, args):
        return self.__update_input_tf(args[0])

class CartesianIncrementTeleopController(CartesianTeleopController):
    def __init__(self, kinematics_solver,
                 input_2_input_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
                 output_2_output_reference_rot = Rotation.Quaternion(0, 0, 0, 1),
                 desired_jaw_in_kinematics = False):
        super().__init__(InputType.INCREMENT, kinematics_solver,
                         input_2_input_reference_rot, output_2_output_reference_rot,
                         desired_jaw_in_kinematics)

    def enable(self, current_output_js):
        self.current_output_js = np.copy(current_output_js)
        self.current_output_tf = self.kinematics_solver.compute_fk(self.current_output_js)
        super()._enable()

    def unclutch(self):
        return super()._unclutch()

    def __increment_input_tf(self, inc_position: Vector, inc_quaternion: Rotation):
        input_diff_tf = convert_frame_to_mat(Frame(inc_quaternion, inc_position))

        # print("current_output_tf: ", np.around(self.current_output_tf, 3))
        self.current_output_tf = self._update_output_tf(input_diff_tf, self.current_output_tf)
        return self.current_output_tf

    def _update_impl(self, args):
        return self.__increment_input_tf(args[0], args[1])

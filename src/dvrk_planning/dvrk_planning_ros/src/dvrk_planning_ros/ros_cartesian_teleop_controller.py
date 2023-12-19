import rospy
import tf
import yaml
import numpy as np

from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, TwistStamped

from PyKDL import Rotation, Vector, Wrench

from dvrk_planning.controller.joint_teleop_controller import JointFollowTeleopController, JointIncrementTeleopController
from dvrk_planning.controller.cartesian_teleop_controller import CartesianFollowTeleopController, CartesianIncrementTeleopController, InputType
from dvrk_planning_ros.utils import gm_tf_to_numpy_mat
from dvrk_planning_ros.ros_teleop_controller import RosTeleopController
from dvrk_planning_ros.mtm_device_crtk import MTM # TODO take away notion of mtm

import tf
import rospy

def quat_yaml_to_pykdl(quaternion_yaml):
    x = quaternion_yaml["x"]
    y = quaternion_yaml["y"]
    z = quaternion_yaml["z"]
    w = quaternion_yaml["w"]

    return Rotation.Quaternion(x, y, z, w)

def rotation_from_yaml(reference_rot):
    if "quaternion" in reference_rot:
        return quat_yaml_to_pykdl(reference_rot["quaternion"])
    # TODO elif "transform"
    elif "lookup_tf" in reference_rot:
        lookup_tf_yaml = reference_rot["lookup_tf"]
        t = tf.TransformListener()
        rospy.sleep(1) # Sleep is needed so TransformListener has time to get the tf's
        _, quat_rot  = t.lookupTransform(lookup_tf_yaml["camera_tf"],lookup_tf_yaml["base_tf"], rospy.Time(0))
        return Rotation.Quaternion(quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3])
    else:
        raise KeyError ("output_to_camera in yaml looking for quaternion or lookup_tf")

class RosCartesiansTeleopController(RosTeleopController):
    def __init__(self, controller_yaml, kinematics_solver):

        output_yaml = controller_yaml["output"]
        self.is_debug_output_tf = False
        if("is_debug_output_tf" in output_yaml):
            self.is_debug_output_tf = output_yaml["is_debug_output_tf"]
        if self.is_debug_output_tf:
            self.br = tf.TransformBroadcaster()

        output_2_output_reference_rot = Rotation.Quaternion(0, 0, 0, 1)
        if("output_2_output_reference_rot" in output_yaml):
            output_2_output_reference_rot = rotation_from_yaml(output_yaml["output_2_output_reference_rot"])

        input_yaml = controller_yaml["input"]
        self.is_half_hz = False
        if("is_half_hz" in input_yaml):
            self.is_half_hz = input_yaml["is_half_hz"]
        self._is_half_hz_trigger = False
        self._is_half_hz_trigger_jaw = False

        self.is_mtm_hold_home_off = False
        if("is_mtm_hold_home_off" in input_yaml): # TODO, remove notion of MTM
            self.is_mtm_hold_home_off = input_yaml["is_mtm_hold_home_off"]
            self.mtm_device = MTM(input_yaml["mtm_device_name"])

        input_2_input_reference_rot = Rotation.Quaternion(0, 0, 0, 1)
        if("input_2_input_reference_rot" in input_yaml):
            input_2_input_reference_rot = rotation_from_yaml(input_yaml["input_2_input_reference_rot"])

        self.jaw_sub = None
        self.desired_output_jaw_angle = None
        desired_jaw_in_kinematics = False
        self._jaw_controller = None
        self.kinematics_solver = kinematics_solver
        if input_yaml["type"] == "follow":
            position_scale = 1.0
            if("position_scale" in input_yaml):
                position_scale = input_yaml["position_scale"]
            input_topic_type = TransformStamped
            self._input_callback_impl = self._input_callback_tf
            self._jaw_mimic_controller = None
            if("jaw" in input_yaml):
                self.desired_output_jaw_angle = 0.5
                desired_jaw_in_kinematics = True
                jaw_yaml = input_yaml["jaw"]
                scale = 1.0
                if "scale" in jaw_yaml:
                    scale = jaw_yaml["scale"]
                self._jaw_mimic_controller = JointFollowTeleopController(scale)
                self._jaw_mimic_controller.register(self._output_jaw_callback)
                self.jaw_input_topic = jaw_yaml["input_topic"]
                self.jaw_sub = rospy.Subscriber(self.jaw_input_topic, JointState, self._input_jaw_mimic)
                self._jaw_controller = self._jaw_mimic_controller
            self._teleop_controller = CartesianFollowTeleopController(
                kinematics_solver,
                input_2_input_reference_rot = input_2_input_reference_rot,
                output_2_output_reference_rot = output_2_output_reference_rot,
                desired_jaw_in_kinematics = desired_jaw_in_kinematics,
                position_scale = position_scale)
        elif input_yaml["type"] == "increment":
            self._jaw_inc_controller = None
            if("jaw" in input_yaml):
                self.desired_output_jaw_angle = 0.5
                desired_jaw_in_kinematics = True
                jaw_yaml = input_yaml["jaw"]
                self._jaw_inc_controller = JointIncrementTeleopController()
                self._jaw_inc_controller.register(self._output_jaw_callback_inc)
                self.jaw_input_topic = jaw_yaml["input_topic"]
                self.jaw_sub = rospy.Subscriber(self.jaw_input_topic, JointState, self._input_jaw_increment)
                self._jaw_controller = self._jaw_inc_controller
            self._teleop_controller = CartesianIncrementTeleopController(
                kinematics_solver,
                input_2_input_reference_rot = input_2_input_reference_rot,
                output_2_output_reference_rot = output_2_output_reference_rot,
                desired_jaw_in_kinematics = desired_jaw_in_kinematics)
            input_topic_type = TwistStamped
            self._input_callback_impl = self._input_callback_twist
        else:
            raise KeyError ("controller: type: must be follow or increment")
        super().__init__(controller_yaml, input_topic_type)
        self.js_msg.name = kinematics_solver.get_active_joint_names()
        self._teleop_controller.register(self._output_callback)

        self.current_input_tf = np.identity(4)
        self.current_output_tf = np.identity(4)

    def enable(self):
        self._wait_for_output_feedback_sub_msg(True)
        # TODO, this is not good oop
        if self._teleop_controller.input_type == InputType.INCREMENT:
            self._teleop_controller.enable(self.current_output_jps)
            if self._jaw_inc_controller:
                _, current_output_jaw, _ = self.kinematics_solver.compute_all_fk(self.current_output_jps)
                self._jaw_inc_controller.enable(np.array([current_output_jaw]))
        elif self._teleop_controller.input_type == InputType.FOLLOW:
            self.wait_for_input_sub_msg(True)
            self._teleop_controller.enable(self.current_input_tf, self.current_output_jps)
            if self._jaw_mimic_controller:
                _, current_output_jaw, _ = self.kinematics_solver.compute_all_fk(self.current_output_jps)
                self._jaw_mimic_controller.enable(self.input_jaw_js, np.array([current_output_jaw]))

    def disable(self):
        self._teleop_controller.disable()
        if self._jaw_controller:
            self._jaw_controller.disable()

    def clutch(self):
        self._teleop_controller.clutch()
        if self._jaw_controller:
            self._jaw_controller.clutch()

    def unclutch(self):
        # TODO, this is not good oop
        if self._teleop_controller.input_type == InputType.INCREMENT:
            self._teleop_controller.unclutch()
            if self._jaw_inc_controller:
                _, current_output_jaw, _ = self.kinematics_solver.compute_all_fk(self.current_output_jps)
                self.__jaw_inc_controller.unclutch(np.array([current_output_jaw]))
        elif self._teleop_controller.input_type == InputType.FOLLOW:
            self.wait_for_input_sub_msg()
            self._wait_for_output_feedback_sub_msg()
            self._teleop_controller.unclutch(self.current_input_tf, self.current_output_jps)
            if self._jaw_mimic_controller:
                _, current_output_jaw, _ = self.kinematics_solver.compute_all_fk(self.current_output_jps)
                self._jaw_mimic_controller.unclutch(self.input_jaw_js, np.array([current_output_jaw]))

    def _debug_output_tf(self):
        if self.is_debug_output_tf:
            output_tf = self._teleop_controller.current_output_tf
            self.br.sendTransform((output_tf[0, 3], output_tf[1, 3], output_tf[2, 3]),
                                tf.transformations.quaternion_from_matrix(output_tf),
                                rospy.Time.now(),
                                "ee",
                                "world")

    def _input_callback_tf(self, data):
        self._is_half_hz_trigger = not self._is_half_hz_trigger
        if self.is_half_hz and not self._is_half_hz_trigger:
            return

        self.current_input_tf = gm_tf_to_numpy_mat(data.transform)
        # print(self.desired_output_jaw_angle)
        self._teleop_controller.update(self.current_input_tf, self.desired_output_jaw_angle)
        if self.is_mtm_hold_home_off: # How to take away notion of MTM in this case
            f = Wrench()
            self.mtm_device.servo_cf(f)
        self._debug_output_tf()

    def _input_jaw_mimic(self, data):
        self._is_half_hz_trigger_jaw = not self._is_half_hz_trigger_jaw
        if self.is_half_hz and not self._is_half_hz_trigger_jaw:
            return

        self.input_jaw_js = data.position
        self._jaw_mimic_controller.update(self.input_jaw_js)

    def _output_jaw_callback(self, joint_positions):
        self.desired_output_jaw_angle = joint_positions[0]

    def _input_callback_twist(self, data):
        twist = data.twist
        self._teleop_controller.update(
            Vector(twist.linear.x, twist.linear.y, twist.linear.z),
            Rotation.RPY(twist.angular.x, twist.angular.y, twist.angular.z),
            self.desired_output_jaw_angle)
        # print(self.desired_output_jaw_angle)
        self._debug_output_tf() # after the update

    def _input_jaw_increment(self, data):
        self.input_jaw_js = data.position
        self._jaw_inc_controller.update(self.input_jaw_js)

    def _output_jaw_callback_inc(self, joint_positions):
        self.desired_output_jaw_angle = joint_positions[0]
        self._teleop_controller.update(
            Vector(0, 0, 0),
            Rotation.RPY(0, 0 ,0),
            self.desired_output_jaw_angle)
        # print(self.desired_output_jaw_angle)
        self._debug_output_tf() # after the update

    def wait_for_input_sub_msg(self, is_print = False):
        super().wait_for_input_sub_msg(is_print)
        if self.jaw_sub is not None:
            print(self._get_str_name(), ": waiting for message from topic [" + self.jaw_input_topic +"]" )
            rospy.wait_for_message(self.jaw_input_topic, JointState)
            print(self._get_str_name(), ": finished waiting for message from topic [" + self.jaw_input_topic +"]" )

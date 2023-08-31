import rospy
import tf
import yaml
import numpy as np

from geometry_msgs.msg import TransformStamped, TwistStamped

from PyKDL import Rotation, Vector

from dvrk_planning.controller.cartesian_teleop_controller import CartesianFollowTeleopController, CartesianIncrementTeleopController, InputType
from dvrk_planning_ros.utils import gm_tf_to_numpy_mat
from dvrk_planning_ros.ros_teleop_controller import RosTeleopController

def quat_yaml_to_pykdl(quaternion_yaml):
    x = quaternion_yaml["x"]
    y = quaternion_yaml["y"]
    z = quaternion_yaml["z"]
    w = quaternion_yaml["w"]

    return Rotation.Quaternion(x, y, z, w)

def output_ref_to_input_rot_from_yaml(output_ref_to_input_rot_yaml):
    if "quaternion" in output_ref_to_input_rot_yaml:
        return quat_yaml_to_pykdl(output_ref_to_input_rot_yaml["quaternion"])
    elif "lookup_tf" in output_ref_to_input_rot_yaml:
        lookup_tf_yaml = output_ref_to_input_rot_yaml["lookup_tf"]
        t = tf.TransformListener()
        rospy.sleep(1) # Sleep is needed so TransformListener has time to get the tf's
        _, quat_rot  = t.lookupTransform(lookup_tf_yaml["camera_tf"],lookup_tf_yaml["base_tf"], rospy.Time(0))
        return Rotation.Quaternion(quat_rot[0], quat_rot[1], quat_rot[2], quat_rot[3])
    else:
        raise KeyError ("output_to_camera in yaml looking for quaternion or lookup_tf")

class RosCartesiansTeleopController(RosTeleopController):
    def __init__(self, controller_yaml, kinematics_solver):
        output_yaml = controller_yaml["output"]
        output_ref_to_input_rot = Rotation.Quaternion(0, 0, 0, 1)
        if("output_ref_to_input_rot" in output_yaml):
            output_ref_to_input_rot = output_ref_to_input_rot_from_yaml(output_yaml["output_ref_to_input_rot"])
        input_yaml = controller_yaml["input"]
        if input_yaml["type"] == "follow":
            position_scale = 1.0
            if("position_scale" in input_yaml):
                position_scale = input_yaml["position_scale"]
            self._teleop_controller = CartesianFollowTeleopController(
                kinematics_solver,
                output_ref_to_input_rot = output_ref_to_input_rot,
                position_scale = position_scale)
            input_topic_type = TransformStamped
            self._input_callback_impl = self._input_callback_tf
        elif input_yaml["type"] == "increment":
            self._teleop_controller = CartesianIncrementTeleopController(kinematics_solver, output_ref_to_input_rot = output_ref_to_input_rot)
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
            self._teleop_controller.enable(self.current_output_tf)
        elif self._teleop_controller.input_type == InputType.FOLLOW:
            self._wait_for_input_sub_msg(True)
            self._teleop_controller.enable(self.current_input_tf, self.current_output_tf)

    def disable(self):
        self._teleop_controller.disable()

    def clutch(self):
        self._teleop_controller.clutch()

    def unclutch(self):
        # TODO, this is not good oop
        if self._teleop_controller.input_type == InputType.INCREMENT:
            self._teleop_controller.unclutch()
        elif self._teleop_controller.input_type == InputType.FOLLOW:
            self._wait_for_input_sub_msg()
            self._wait_for_output_feedback_sub_msg()
            self._teleop_controller.unclutch(self.current_input_tf, self.current_output_tf)

    def _output_feedback_callback(self, js):
        super()._output_feedback_callback(js)
        self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)

    def _input_callback_tf(self, data):
        self.current_input_tf = gm_tf_to_numpy_mat(data.transform)
        self._teleop_controller.update(self.current_input_tf)

    def _input_callback_twist(self, data):
        twist = data.twist
        self._teleop_controller.update(
            Vector(twist.linear.x, twist.linear.y, twist.linear.z),
            Rotation.RPY(twist.angular.x, twist.angular.y, twist.angular.z))
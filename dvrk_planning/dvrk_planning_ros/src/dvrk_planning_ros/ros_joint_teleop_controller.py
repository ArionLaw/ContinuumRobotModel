import numpy as np
import rospy

from sensor_msgs.msg import JointState

from dvrk_planning.controller.joint_teleop_controller import JointFollowTeleopController, JointIncrementTeleopController, InputType
from dvrk_planning_ros.ros_teleop_controller import RosTeleopController

class RosJointTeleopController(RosTeleopController):
    def __init__(self, controller_yaml):

        input_yaml = controller_yaml["input"]
        if input_yaml["type"] == "follow":
            scale = 1.0
            if "scale" in input_yaml:
                scale = input_yaml["scale"]
            teleop_controller = JointFollowTeleopController(scale)
        elif input_yaml["type"] == "increment":
            teleop_controller = JointIncrementTeleopController()
        else:
            raise KeyError ("controller: input type: must be follow or increment")

        self._input_callback_impl = self._input_callback_js
        self.current_input_jps = np.array([])

        super().__init__(controller_yaml, JointState)
        self._teleop_controller = teleop_controller
        self._teleop_controller.register(self._output_callback)

    def enable(self):
        self._wait_for_output_feedback_sub_msg(True)
        # TODO, this is not good oop
        if self._teleop_controller.input_type == InputType.INCREMENT:
            self._teleop_controller.enable(self.current_output_jps)
        elif self._teleop_controller.input_type == InputType.FOLLOW:
            self._wait_for_input_sub_msg(True)
            self._teleop_controller.enable(self.current_input_jps, self.current_output_jps)

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
            self._teleop_controller.unclutch(self.current_input_jps, self.current_output_jps)

    def _input_callback_js(self, data):
        self.current_input_jps = data.position
        self._teleop_controller.update(self.current_input_jps)

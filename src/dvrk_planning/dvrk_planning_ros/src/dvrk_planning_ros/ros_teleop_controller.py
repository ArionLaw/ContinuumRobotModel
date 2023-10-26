import rospy
import numpy as np
from sensor_msgs.msg import JointState

from dvrk_planning.kinematics.utilities import get_harmonized_joint_positions, TWO_PI
from dvrk_planning_ros.average_timer import AverageTimer

class RosTeleopController:
    def __init__(self, controller_yaml, ros_input_type):
        self.name = controller_yaml["name"]
        self.input_topic = controller_yaml["input"]["topic"]
        self.ros_input_type = ros_input_type

        if "is_timed" in controller_yaml and controller_yaml["is_timed"]:
            self._average_timer = AverageTimer()
            self.input_sub = rospy.Subscriber(self.input_topic, self.ros_input_type, self._input_callback_timed)
        else:
            self._average_timer = None
            self.input_sub = rospy.Subscriber(self.input_topic, self.ros_input_type, self._input_callback_impl)

        output_yaml = controller_yaml["output"]
        self.js_msg = JointState()
        self.output_pub = rospy.Publisher(output_yaml["control_topic"], JointState, queue_size = 1)
        self.output_feedback_topic = output_yaml["feedback_topic"]
        self.output_feedback_sub = rospy.Subscriber(self.output_feedback_topic, JointState, self._output_feedback_callback)

        self.current_output_jps = np.array([])

    def _get_str_name(self):
        return "Teleop controller [" + self.name + "]"

    def enable(self):
        raise NotImplementedError

    def disable(self):
        raise NotImplementedError

    def clutch(self):
        raise NotImplementedError

    def unclutch(self):
        raise NotImplementedError

    def _wait_for_output_feedback_sub_msg(self, always_print = False):
        try:
            if(always_print):
                 raise
            rospy.wait_for_message(self.output_feedback_topic, JointState, timeout=0.01) # timeout 0.01s to see if publishing
        except:
            print(self._get_str_name(), ": waiting for message from topic [" + self.output_feedback_topic +"]" )
            rospy.wait_for_message(self.output_feedback_topic, JointState)
            print(self._get_str_name(), ": finished for message from topic [" + self.output_feedback_topic +"]" )

    def _output_callback(self, joint_positions):
        self.js_msg.position = get_harmonized_joint_positions(joint_positions, np.array(self.current_output_jps))

        print("np.array(self.current_output_jps):\n", np.around(np.array(self.current_output_jps),3))
        print("output js:\n", np.around(joint_positions,3))
        print("output js harmonized:\n", np.around(self.js_msg.position,3))

        self.output_pub.publish(self.js_msg)

    def _output_feedback_callback(self, js):
        self.js_msg.name =  js.name
        self.current_output_jps = js.position

    def _wait_for_input_sub_msg(self, always_print=False):
        try:
            if(always_print):
                 raise
            rospy.wait_for_message(self.input_topic, self.ros_input_type, timeout=0.01) # timeout 0.01s to see if publishing
        except:
            print(self._get_str_name(), ": waiting for message from topic [" + self.input_topic +"]" )
            rospy.wait_for_message(self.input_topic, self.ros_input_type)
            print(self._get_str_name(), ": finished waiting for message from topic [" + self.input_topic +"]" )

    def _input_callback_impl(data):
        raise NotImplementedError

    def _input_callback_timed(self, data):
        self._average_timer.start()
        self._input_callback_impl(data)
        self._average_timer.stop()

    def get_str_statistics_report(self):
        if self._average_timer:
            return f"---[{self.name}]---\n" + "Input callback latency:\n" + self._average_timer.str_statistics_report()
        return None

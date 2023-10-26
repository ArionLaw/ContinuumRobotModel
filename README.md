# ContinuumRobotModel

elif kin_yaml["robot"] == "fetal":
                    kin_solver = Peter_Francis_tool_Kinematics_Solver
                    
rosrun dvrk_planning_ros dvrk_teleop_node.py -y config/keyboard_psm2_PeterFrancis.yaml

[ERROR] [1695757557.537887]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.566472]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.568903]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.569828]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.576715]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.591125]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.596541]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.606699]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.618407]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.626710]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.639205]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.650328]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

[ERROR] [1695757557.656880]: bad callback: <bound method RosCartesiansTeleopController._output_feedback_callback of <dvrk_planning_ros.ros_cartesian_teleop_controller.RosCartesiansTeleopController object at 0x7f02d8892f70>>
Traceback (most recent call last):
  File "/opt/ros/noetic/lib/python3/dist-packages/rospy/topics.py", line 750, in _invoke_callback
    cb(msg)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 91, in _output_feedback_callback
    self.current_output_tf = self._teleop_controller.kinematics_solver.compute_fk(js.position)
TypeError: compute_fk() missing 1 required positional argument: 'psm_joints'

Traceback (most recent call last):
  File "/home/dvrk/ContinuumRobotModel/devel/lib/dvrk_planning_ros/dvrk_teleop_node.py", line 15, in <module>
    exec(compile(fh.read(), python_script, 'exec'), context)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/scripts/dvrk_teleop_node.py", line 112, in <module>
    dvrk_teleop_node = DvrkTeleopNode(config_yaml = config_yaml)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/scripts/dvrk_teleop_node.py", line 40, in __init__
    self.ros_teleop_controllers[controller_yaml["name"]] = RosCartesiansTeleopController(controller_yaml, kin_solver)
  File "/home/dvrk/ContinuumRobotModel/src/dvrk_planning/dvrk_planning_ros/src/dvrk_planning_ros/ros_cartesian_teleop_controller.py", line 59, in __init__
    self.js_msg.name = kinematics_solver.get_active_joint_names()
AttributeError: 'Peter_Francis_tool_Kinematics_Solver' object has no attribute 'get_active_joint_names'

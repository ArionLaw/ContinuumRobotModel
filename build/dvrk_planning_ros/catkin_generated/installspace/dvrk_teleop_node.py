#!/usr/bin/env python3
import importlib
import argparse
import os.path
import rospy
import rospkg
import sys
import yaml

from sensor_msgs.msg import  Joy
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, CustomSphericalWristFromYaml
from dvrk_planning_ros.ros_joint_teleop_controller import RosJointTeleopController
from dvrk_planning_ros.ros_cartesian_teleop_controller import RosCartesiansTeleopController

class DvrkTeleopNode:
    def __init__(self, config_yaml):
        self.ros_teleop_controllers = {}

        if "cartesian_controllers" in config_yaml:
            for controller_yaml in config_yaml["cartesian_controllers"]:
                kin_yaml = controller_yaml["kinematics"]
                kin_solver = None
                if kin_yaml["robot"] == "psm":
                    tool_yaml = kin_yaml["tool"]
                    if tool_yaml["type"] == "custom":
                        kin_solver = PsmKinematicsSolver(CustomSphericalWristFromYaml(tool_yaml))
                    else:
                        module = importlib.import_module("dvrk_planning.kinematics.psm")
                        class_ = getattr(module, tool_yaml["type"])
                        scale = 1.0
                        if "scale" in tool_yaml:
                            scale = tool_yaml["scale"] 
                            print("Choosing scale {}".format(scale))
                        kin_solver = PsmKinematicsSolver(class_(scale))
                elif kin_yaml["robot"] == "fetal":
                    tool_yaml = kin_yaml[""]
                    kin_solver = Peter_Francis_tool_Kinematics_Solver
                else:
                    raise KeyError ("Only [psm] available now")
                self.ros_teleop_controllers[controller_yaml["name"]] = RosCartesiansTeleopController(controller_yaml, kin_solver)

        if "joint_controllers" in config_yaml:
            for controller_yaml in config_yaml["joint_controllers"]:
                self.ros_teleop_controllers[controller_yaml["name"]] = RosJointTeleopController(controller_yaml)

        clutch_topic = "/console/clutch"
        if ("clutch_topic" in config_yaml):
            clutch_topic = config_yaml["clutch_topic"]
            self.clutch_sub = rospy.Subscriber(clutch_topic, Joy, self.clutch_callback)

        #switch_topic = "/console/camera"
        # switch_topic = "/toggle/mode"
        if ("switcher" in config_yaml):
            switcher_yaml = config_yaml["switcher"]
            switch_topic = switcher_yaml["switch_topic"]
            self.disable_list = switcher_yaml["disable_list"]
            self.enable_list = switcher_yaml["enable_list"]
            self.switch_sub = rospy.Subscriber(switch_topic, Joy, self.switch_callback)

        if "enable_at_start" in config_yaml:
            enable_at_start = config_yaml["enable_at_start"]
        else:
            enable_at_start = self.ros_teleop_controllers.keys()
        for name in enable_at_start:
            self.ros_teleop_controllers[name].enable()

    def clutch_callback(self, data):
        if data.buttons[0] == 1:
            for _, rtc in self.ros_teleop_controllers.items():
                rtc.clutch()
        elif data.buttons[0] == 0:
            for _, rtc in self.ros_teleop_controllers.items():
                rtc.unclutch()

    def _execute_enable_disable(self, enable_list, disable_list):
        for name in enable_list:
            self.ros_teleop_controllers[name].enable()
        for name in disable_list:
            self.ros_teleop_controllers[name].disable()

    def switch_callback(self, data):
        if data.buttons[0] == 1:
            self._execute_enable_disable(self.enable_list, self.disable_list)
        elif data.buttons[0] == 0:
            self._execute_enable_disable(self.disable_list, self.enable_list)

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-p', '--config_package', type=str, required=False,
                        default = "dvrk_planning_ros",
                        help = 'the package where you store the config yaml, default is "dvrk_planning_ros"')
    parser.add_argument('-y', '--yaml', type=str, required=False,
                        default = "config/teleop.yaml",
                        help = 'the relative path to your yaml file from package directory, default is "config/teleop.yaml"')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name

    # Find yaml full path
    config_package_name = args.config_package
    relative_yaml_path = args.yaml
    package_path = rospkg.RosPack().get_path(config_package_name)
    full_config_path = os.path.join(package_path, relative_yaml_path)

    # Parse yaml
    yaml_file = open(full_config_path)
    config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)

    rospy.init_node('dvrk_teleop')
    try:
        dvrk_teleop_node = DvrkTeleopNode(config_yaml = config_yaml)
        rospy.spin()
        for _, rtc in dvrk_teleop_node.ros_teleop_controllers.items():
            str_report = rtc.get_str_statistics_report()
            if str_report:
                print(str_report + "\n")
    except rospy.ROSInterruptException: 
        pass

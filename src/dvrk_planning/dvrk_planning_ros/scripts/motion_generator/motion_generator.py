#!/usr/bin/env python3
import importlib
import argparse
import os.path
import rospy
import rospkg
import sys
import yaml
import time
import rospy

from trajectory_modules import trajectory_follower
from trajectory_modules import trajectory_generator
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Joy
from threading import Thread
from dvrk_planning_msgs.msg import Waypoints
from dvrk_planning_msgs.msg import TrajectoryStatus
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Transform
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float64MultiArray

class MotionGenerator:
    def __init__(self, config_yaml):
        self.cutting_cartesian_velocity = 0.00125
        self.current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.position_update_flag = 0
        self.jaw_position = [0.0]
        self.jaw_update_flag = 0
        self.instructions = []
        self.follower_interrupt_flag = [0]
        self.last_point = 0
        self.last_jaw = 0

        self.JointStatePublisher = rospy.Publisher(config_yaml['ros_communication'][0]['output']['joint_control_topic'], JointState, queue_size = 10)
        self.JawStatePublisher = rospy.Publisher(config_yaml['ros_communication'][0]['output']['jaw_control_topic'], JointState, queue_size = 10)
        self.StatusPublisher = rospy.Publisher("/motion_generator" + config_yaml['ros_communication'][0]['output']['trajectory_status_topic'], TrajectoryStatus, queue_size = 10)
        self.RvizTrajPublisher = rospy.Publisher("/motion_generator" + config_yaml['ros_communication'][0]['output']['rviz_trajectory_topic'], Float64MultiArray, queue_size = 10)

        self.follower_obj = trajectory_follower.Follower()
        self.generator_obj = trajectory_generator.Trajectories()
        self.set_config(config_yaml)

    def generator_function(self, data, mode, name):
        if not self.position_update_flag:
            time.sleep(0.3)
        rospy.wait_for_message("/PSM2/measured_js", JointState, timeout=2)

        # generator_obj = trajectory_generator.Trajectories()
        if self.position_update_flag == 1:
            if mode == "arm":
                if self.last_point == 0:
                    self.last_point = self.current_position
                trajectory, durations, interpolated_points, waypoints_cartesian = self.generator_obj.generate_traj (self.last_point, data, name, self.cutting_cartesian_velocity)
                if trajectory == []:
                    print("already here, no trajectory to generate")
                else:
                    self.last_point = trajectory [len(trajectory)-1]
                    self.instructions.append(  {'type':'trajectory', 'trajectory': trajectory, 'durations': durations, 'interpolated_points': interpolated_points, 'name':name, 'waypoints_cartesian': waypoints_cartesian } )
            if mode == "jaw":
                if self.jaw_update_flag ==1:
                    if self.last_jaw == 0:
                        self.last_jaw = self.jaw_position
                    jaw_traj, duration, points = self.generator_obj.generate_traj_jaw(self.last_jaw, data)
                    self.instructions.append(  {'type':'jaw', 'trajectory': jaw_traj, 'duration': duration, 'num_points': points } )
                    if len(jaw_traj)>0:
                        self.last_jaw = jaw_traj [len(jaw_traj)-1]

    def follower_function(self):
        # follower_obj = trajectory_follower.Follower()
        display_flag = 0
        while not rospy.is_shutdown():
            if self.follower_interrupt_flag[0]==1:
                if len(self.instructions) == 0:
                    if display_flag == 0:
                        print ("Waiting for further instructions ......")
                        display_flag = 1
                else: 
                    display_flag = 0
                    if self.instructions[0]['type'] == 'trajectory':
                        self.follower_obj.follow_trajectory(self.instructions[0]['trajectory'],self.JointStatePublisher, self.instructions[0]['durations'], self.instructions[0]['interpolated_points'], self.instructions[0]['name'], self.instructions[0]['waypoints_cartesian'], self.StatusPublisher, self.RvizTrajPublisher, self.JawStatePublisher, self.jaw_position)
                        self.instructions.pop(0)
                    elif self.instructions[0]['type'] == 'jaw':
                        self.follower_obj.follow_jaw_trajectory(self.instructions[0]['trajectory'],self.JawStatePublisher, self.instructions[0]['duration'], self.instructions[0]['num_points'])
                        self.instructions.pop(0)

    def position_callback(self, msg):
        self.current_position = msg.position
        self.position_update_flag = 1

    def get_current_position(self):
        print (self.current_position)
        return self.current_position

    def jaw_callback(self, msg):
        self.jaw_position = msg.position[0]
        self.jaw_update_flag = 1

    def enable(self):
        print ("Motion generator enabled.....")
        self.follower_interrupt_flag = [1]
        # follower_obj = trajectory_follower.Follower()
        self.follower_obj.set_follower_status(self.follower_interrupt_flag)
        self.position_update_flag = 0

    def disable(self):
        print ("Motion generator disabled.....")
        self.follower_interrupt_flag = [0]
        # follower_obj = trajectory_follower.Follower()
        self.follower_obj.set_follower_status(self.follower_interrupt_flag)
        self.position_update_flag = 0

    def switching_callback(self, msg):
        if msg.buttons[0] == 0:
            self.disable()
        elif msg.buttons[0] == 1:
            self.enable()
        else:
            print("Something went wrong with the switching")

    def set_config(self, config):
        self.cutting_cartesian_velocity = config['cutting_parameters']['cutting_cartesian_velocity']
        self.follower_obj.set_config(config)
        self.generator_obj.set_config(config)
        if config['default_follower_status'] == "enabled":
            self.enable()
        else:
            self.disable()

    def offset_callback(self, offset):
        self.follower_obj.set_offset(offset.data)

    def msg_rotMatrix(self, waypoints):
        waypoints_output=[]
        #print("all goals", waypoints)
        for i in range(0, len(waypoints)):
            
            r = R.from_quat([waypoints[i].rotation.x, waypoints[i].rotation.y, waypoints[i].rotation.z, waypoints[i].rotation.w])
            temp = r.as_matrix()
            point = temp.tolist()
            
            point[0].append(waypoints[i].translation.x)
            point[1].append(waypoints[i].translation.y)
            point[2].append(waypoints[i].translation.z)
            
            point.append([0,0,0,1])
            

            waypoints_output.append(point)
        #print("wayout", waypoints_output)
        return waypoints_output

    def waypoint_callback(self, msg):
        if msg.instruction_mode == "jaw":
            self.generator_function([msg.jaw_instruction], msg.instruction_mode, msg.trajectory_name)
        else:
            data = self.msg_rotMatrix(msg.waypoints)
            self.generator_function([data], msg.instruction_mode, msg.trajectory_name)


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

    # Parse yamlposition
    yaml_file = open(full_config_path)
    config_yaml = yaml.load(yaml_file, Loader=yaml.FullLoader)
    mg = MotionGenerator(config_yaml)

    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber("/motion_generator" + config_yaml['ros_communication'][0]['input']['waypoint_input'],Waypoints, mg.waypoint_callback)
    rospy.Subscriber("/motion_generator" + config_yaml['ros_communication'][0]['input']['trajectory_offset_topic'],Float32MultiArray, mg.offset_callback)
    rospy.Subscriber(config_yaml['ros_communication'][0]['input']['joint_positions_topic'],JointState, mg.position_callback)
    rospy.Subscriber(config_yaml['ros_communication'][0]['input']['jaw_position_topic'],JointState, mg.jaw_callback)
    rospy.Subscriber("/motion_generator" + config_yaml['toggle_topic'], Joy, mg.switching_callback)
 
    #Thread(target = mg.generator_function).start()
    Thread(target = mg.follower_function).start()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")




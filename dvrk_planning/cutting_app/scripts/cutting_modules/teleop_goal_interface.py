#!/usr/bin/env python3
from numpy import deprecate_with_doc
import rospy
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007
import copy

from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import JointState
import math

import numpy as np
import matplotlib.pyplot as plt
from scipy import interpolate
from dvrk_planning_msgs.msg import Waypoints
from geometry_msgs.msg import Transform

from scipy import interpolate
from mpl_toolkits.mplot3d import Axes3D

class TeleopGoalInterface:
    def __init__(self):
        self.kinematics = RTS470007()

        ###############################################
        #config
        ###############################################

        # the depth the cutting tool should go further than the waypoint
        self.depth = 0.0001

        #orientation of the cutting tool to be maintained
        self.x_incline = 80
        self.y_incline =  0
        self.z_rotation = -90

        #Interpolation parameters
        self.interpolated_points = 75
        self.smoothing_factor = 0
        self.polynomial_degree = 2
        self.spline_mode = 1

        ###############################################

        self.last_data = []
        self.current_position = []

        self.circle_radius = 0.03

    def remove_duplicates(self, x_sample,y_sample,z_sample):
        
        #Todo
        #for i in range():
        return x_sample,y_sample,z_sample

    def get_goals(self):
        index = 0
        mode = 0
        goal_output = []        
        
        while not rospy.is_shutdown():
            #print("g if current location is a goal")
            print("w if current location is a waypoint")
            print("s to send the points to the generator")
            print("g to send the points as goals")
            print("o to home to origin")
            print("c for circle")
            print("d to discard the points")
            print("r to retry")
            #print("b to toggle B spline")
            choice = input()
            #try:
            temp = list(choice)
            #curr_position = motion_generator.get_current_position()
            curr_position = self.current_position
            if temp[0] == 'g' or temp[0] == 'G':
                mode = 2 
            elif temp[0] == 'w' or temp[0] == 'W':# or temp[0] == 's' or temp[0] == 'S' :
                mode = 1
            elif temp[0] == 's' or temp[0] == 'S':
                mode = 2
            elif temp[0] == 'd' or temp[0] == 'D':
                goal_output = []
                mode = 2
            elif temp[0] == 'r' or temp[0] == 'R':
                self.send_goals_and_plot(goal_output)
                mode = 2
            elif temp[0] == 'o' or temp[0] == 'O':
                reorient_goal = [[0, 1, 0, 0],[1, 0, 0, 0],[0, 0, -1, -0.1],[0,0,0,1]]
                self.publish_goals("goal", [ reorient_goal])
                mode = 2
            elif temp[0] == 'c' or temp[0] == 'C':
                self.create_circle()
                mode = 2
            else: 
                print ("invalid input, try again ......")
            
            if len (goal_output) ==0 and mode!=2:
                rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=2)
                goal_output.append([list( curr_position )] )
            else:
                rospy.wait_for_message("/PSM2/measured_js",JointState, timeout=2)
                if mode == 1:
                    goal_output[index].append( list( curr_position ))
                elif mode == 0:
                    goal_output[index].append( list( curr_position ) )
                    goal_output.append ([list( curr_position )])
                    index = index + 1
            if temp[0] == 'g' or temp[0] == 'G':
                if (len(goal_output))<1: 
                    print("nothing to send")
                elif(len(goal_output[0])<2):
                    print("choose at least 2 points")
                else: 
                    self.send_goals(goal_output)

            if temp[0] == 's' or temp[0] == 'S':
                if (len(goal_output))<1: 
                    print("nothing to send")
                elif(len(goal_output[0])<2):
                    print("choose at least 2 points")
                else: 
                    self.send_goals_and_plot(goal_output)
                    #get_goals()
            print(goal_output)
            # except:
            #     print ("something went wrong, try again ......")
            #     return 1 

    def send_goals(self, goal_output):
        for i in range (0, len(goal_output)):
            waypoints = []
            #print("lenght goal", len(goal_output[0]))
            for j in range(0, len(goal_output[i])):
                p = PsmKinematicsSolver(self.kinematics)
                current_postion_cartesian = p.compute_fk(goal_output[i][j])
                current_postion_cartesian = current_postion_cartesian.getA()
                waypoints.append (list(current_postion_cartesian))
            #print("lenght", len(waypoints))

            waypoints.reverse()
            x_sample = []
            y_sample = []
            z_sample = []
            for i in range(0, len(waypoints)):
                x_sample.append(waypoints[i][0][3])
                y_sample.append(waypoints[i][1][3])
                z_sample.append(waypoints[i][2][3])

            waypoints = self.jaw_orientation([x_sample, y_sample, z_sample])

            for i in range(0, len(waypoints)-1):

                self.publish_goals("goal", [ waypoints[i]])
                temp = copy.deepcopy( waypoints[i])
                temp[0][3] =  waypoints[i+1][0][3]
                temp[1][3] =  waypoints[i+1][1][3]
                temp[2][3] =  waypoints[i+1][2][3]
                self.publish_goals("scissor", [ waypoints[i],temp])

    def send_goals_and_plot(self, goal_output):
        for i in range (0, len(goal_output)):
            waypoints = []
            for j in range(0, len(goal_output[i])):
                p = PsmKinematicsSolver(self.kinematics)
                current_postion_cartesian = p.compute_fk(goal_output[i][j])
                current_postion_cartesian = current_postion_cartesian.getA()
                waypoints.append (list(current_postion_cartesian))

            waypoints.reverse()
            waypoints = self.interpolator(waypoints)

            temp =  [[0, 1, 0],[1, 0, 0],[0, 0, -1]]

            waypoint = temp
            waypoint[0].append (waypoints[0] [0][3])
            waypoint[1].append (waypoints[0] [1][3])
            waypoint[2].append (waypoints[0] [2][3] )
            waypoint.append ([0,0,0,1])
            
            self.publish_goals("goal", [ waypoints[0]])
            self.publish_goals("scissor", waypoints)

            temp2 = copy.deepcopy(waypoints[len(waypoints)-1])
            temp2[0][3] = temp2[0][3] + 0.005
            #temp2[1][3] = temp2[1][3] - 0.0025
            self.publish_goals("scissor", [temp2])

            
            #print (goal_output_cart)

    def create_circle(self):
        p = PsmKinematicsSolver(self.kinematics)
        current_postion_cartesian = p.compute_fk(self.current_position)
        current_postion_cartesian = current_postion_cartesian.getA()
        circle_waypoints = []
        point_temp = current_postion_cartesian
        circle_waypoints.append(p.compute_ik(np.matrix(point_temp)))
        point_temp1 = point_temp
        point_temp[0][3] = point_temp[0][3] - self.circle_radius
        point_temp[1][3] = point_temp[1][3] + self.circle_radius
        circle_waypoints.append(p.compute_ik(np.matrix(point_temp.copy())))
        point_temp[0][3] = point_temp[0][3] + self.circle_radius
        point_temp[1][3] = point_temp[1][3] + self.circle_radius
        circle_waypoints.append(p.compute_ik(np.matrix(point_temp)))
        point_temp[0][3] = point_temp[0][3] + self.circle_radius
        point_temp[1][3] = point_temp[1][3] - self.circle_radius
        circle_waypoints.append(p.compute_ik(np.matrix(point_temp)))
        point_temp[0][3] = point_temp[0][3] - self.circle_radius
        point_temp[1][3] = point_temp[1][3] - self.circle_radius
        circle_waypoints.append(p.compute_ik(np.matrix(point_temp)))
        self.send_goals_and_plot([circle_waypoints])

    def rotMatix_msg(self, goals):
        
        waypoints_output=[]
        #print("all goals", goals)
        for i in range(0, len(goals)):
            
            temp = Transform()
            temp.translation.x = goals[i][0][3]
            temp.translation.y = goals[i][1][3]
            temp.translation.z = goals[i][2][3]
            
            goal = goals[i]
            #print(goal)
            r = R.from_matrix([goal[0][:3], goal[1][:3], goal[2][:3]])
            quat = r.as_quat()

            temp.rotation.x = quat[0]
            temp.rotation.y = quat[1]
            temp.rotation.z = quat[2]
            temp.rotation.w = quat[3]

            waypoints_output.append(temp)
        #print("wayout", waypoints_output)
        return waypoints_output

    def publish_goals(self, name, goals):
        dat = Waypoints()
        dat.trajectory_name = name
        dat.instruction_mode = "arm"
        dat.waypoints = self.rotMatix_msg(goals)
        GoalStatePublisher.publish(dat)

    def open_jaw(self):
        dat = Waypoints()
        dat.trajectory_name = "jaw"
        dat.instruction_mode = "jaw"
        dat.jaw_instruction = "o"
        GoalStatePublisher.publish(dat)

    def close_jaw(self):
        dat = Waypoints()
        dat.trajectory_name = "jaw"
        dat.instruction_mode = "jaw"
        dat.jaw_instruction = "c"
        GoalStatePublisher.publish(dat)

    def interpolator(self, waypoints):
        x_sample = []
        y_sample = []
        z_sample = []
        for i in range(0, len(waypoints)):
            x_sample.append(waypoints[i][0][3])
            y_sample.append(waypoints[i][1][3])
            z_sample.append(waypoints[i][2][3])
        
        x_sample,y_sample,z_sample = self.remove_duplicates(x_sample,y_sample,z_sample)

        if self.spline_mode ==1:
            if (len(x_sample))<4:
                self.polynomial_degree = len(x_sample)-1
            tck, u = interpolate.splprep([x_sample,y_sample,z_sample], s= self.smoothing_factor, k = self.polynomial_degree)
            x_knots, y_knots, z_knots = interpolate.splev(tck[0], tck)
            u_fine = np.linspace(0,1, self.interpolated_points )
            x_fine, y_fine, z_fine = interpolate.splev(u_fine, tck)

        fig2 = plt.figure(2)
        ax3d = fig2.add_subplot(111, projection='3d')

        
        ax3d.plot(x_sample, y_sample, z_sample, 'r*')
        if self.spline_mode == 1:
            ax3d.plot(x_knots, y_knots, z_knots, 'go')
        # else:
        #     x_fine, y_fine, z_fine = x_sample, y_sample, z_sample
        ax3d.plot(x_fine, y_fine, z_fine, 'g')
        
        #print(x_fine[0])
        fig2.show()
        plt.show()

        return self.jaw_orientation([x_fine, y_fine, z_fine])

    def jaw_orientation(self, goals):
        #print (goals)

        waypoints = []
        for i in range (0, len(goals[0])-1 ):
            y = goals[1][i+1] - goals[1][i]
            x = goals[0][i+1] - goals[0][i]
            angle = math.atan2(y,x) *180/3.14
            #print (angle)
            r = R.from_euler('xyz', [(-180 + self.x_incline), self.y_incline, (angle)+ self.z_rotation], degrees=True)
            temp = r.as_matrix()
            temp = temp.tolist()

            waypoint = copy.deepcopy(temp)
            waypoint[0].append (goals[0][i])
            waypoint[1].append (goals[1][i])
            waypoint[2].append (goals[2][i])
            waypoint.append ([0,0,0,1])
            waypoints.append(waypoint)
            
            if i == len(goals[0])-2:
                waypoint1 = copy.deepcopy(temp)
                waypoint1[0].append (goals[0][i+1])
                waypoint1[1].append (goals[1][i+1])
                waypoint1[2].append (goals[2][i+1])
                waypoint1.append ([0,0,0,1])
                # print("waypoint1\n", waypoint1)
                waypoints.append(waypoint1)

        return(waypoints)

    def position_callback(self, msg):
        self.current_position = msg.position

    def temp(self):
        #scissor 3cm
        goal1 =   [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
        goal2 =  [0.32312284660346086, 0.3378970772970957, 0.05229669581385103, 0.0, -0.3378970772970953, -0.32312284660346097]
        goal3 =  [0.32312284660346086, -0.3378970772970957, 0.05229669581385103, -0.0, 0.3378970772970953, -0.32312284660346097]
        goal4 =  [-0.32312284660346086, -0.3378970772970957, 0.05229669581385103, 0.0, 0.3378970772970953, 0.32312284660346136]
        goal5 =  [-0.32312284660346086, 0.3378970772970957, 0.05229669581385103, -0.0, -0.3378970772970953, 0.32312284660346097]
        goal6 =  [0.0, -0.0, 0.04229000000000005, 0.0, -0.0, 0.0]
        goal7 =  [0.32312284660346086, -0.0, 0.04717945861402896, 0.0, -0.0, -0.32312284660346136]

        goals = [[goal2, goal3, goal4]]

        self.send_goals_and_plot(goals)
        return 0

    def sequential_goals(self):
        if len(self.last_data)>0:
            temp = self.last_data[0]['waypoints']
            name = self.last_data[0]['name']
            self.last_data.pop(0)
            return temp, name
        else:
            return [], ""


if __name__ == '__main__':
    tgi = TeleopGoalInterface()

    rospy.init_node('Teleop_Goal_interface', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js", JointState, tgi.position_callback)
    GoalStatePublisher = rospy.Publisher("/motion_generator/input/waypoints", Waypoints, queue_size = 10)
    tgi.get_goals()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
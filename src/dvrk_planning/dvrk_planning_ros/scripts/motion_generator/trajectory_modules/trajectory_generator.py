#!/usr/bin/env python3

from dis import distb
import numpy as np
#import roboticstoolbox as rtb
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import math
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007, RTS420007

class Trajectories:

    def __init__(self):
        self.interpolating_distance = 0.01
        self.no_of_points_jp = 10
        #output frequency
        self.frequency = 50 
        self.rate  = 1 / self.frequency
        #constraints
        self.vlims = np.array([0.15,0.15,0.15,0.15,0.15,0.15]) * 10
        self.alims = np.array([0.075, 0.075, 0.075, 0.075, 0.075, 0.075]) * 2
        #jaw stuff
        self.interpolating_angle_jaw = 0.25          #degrees
        self.jaw_angle_open = 30                     #degrees
        self.jaw_angle_close = 0                     #degrees
        self.kinematics = LND400006()                #default kinematics, change from config
        self.experiment_name = "test"

    def set_config(self, config):
        self.interpolating_distance = config['motion_generation_parameters']['general']['cartesian_interpolating_distance']
        self.no_of_points_jp = config['motion_generation_parameters']['general']['no_of_points_jp']
        self.frequency = config['motion_generation_parameters']['general']['output_frequency']
        self.jaw_angle_open = config['motion_generation_parameters']['jaw']['jaw_angle_open']
        self.jaw_angle_close = config['motion_generation_parameters']['jaw']['jaw_angle_close']
        self.vlims = config['motion_generation_parameters']['default_joint_constraints']['vlims']
        self.alims = config['motion_generation_parameters']['default_joint_constraints']['alims']
        self.kinematics = eval(config['motion_generation_parameters']['kinematics']['tool']+ "()")
        self.experiment_name = config['experiment_name']

    def linear_interpolator_function(self, start_point, end_point):
        distance = math.sqrt( math.pow((start_point[0]-end_point[0]),2) +math.pow((start_point[1]-end_point[1]),2) + math.pow((start_point[2]-end_point[2]),2) )
        n = self.interpolating_distance
        waypoints = [start_point]
        while n <= (distance-self.interpolating_distance):
            factor = n/distance
            x = start_point[0] + (factor * (end_point[0]-start_point[0]) )
            y = start_point[1] + (factor * (end_point[1]-start_point[1]) )
            z = start_point[2] + (factor * (end_point[2]-start_point[2]) )
            n = self.interpolating_distance+n
            waypoints.append([x,y,z])
        waypoints.append(end_point)
        return waypoints

    def rotational_interpolator(self, no_of_points, start_orientation, end_rotation):
        np.set_printoptions(precision = 4, suppress = True)
        key_rots = R.from_matrix( [start_orientation, end_rotation] )
        key_times = [0, 1]
        slerp = Slerp(key_times, key_rots)
        times = np.array(list(range(0, no_of_points)))
        times = times / (no_of_points-1) 
        interp_rots = slerp(times)
        return interp_rots.as_matrix()


    def toppra(self, current_position=[], goal_js=[], way_pts = [0.005], name = "", desired_duration = 0):    
        no_of_points = self.no_of_points_jp * len (way_pts)
        if way_pts[0] == 0.01:
            way_pts = [current_position,goal_js]
        N_samples = len(way_pts)
        ss = np.linspace(0, 1, N_samples)
        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(self.vlims)
        pc_acc = constraint.JointAccelerationConstraint(self.alims)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel", gridpt_max_err_threshold=1e-3, gridpt_min_nb_points= no_of_points)
        jnt_traj = instance.compute_trajectory()
        ts_sample = np.linspace(0, jnt_traj.duration, int(jnt_traj.duration/self.rate) )
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1) 
        qdds_sample = jnt_traj(ts_sample, 2)
        trajectory = qs_sample
        duration = jnt_traj.duration

        if name=="scissor":
            instance = algo.TOPPRAsd([pc_vel, pc_acc], path)
            instance.set_desired_duration(desired_duration)
            jnt_traj = instance.compute_trajectory()
            ts_sample = np.linspace(0, jnt_traj.get_duration(), int(jnt_traj.get_duration()/self.rate))
            qs_sample = jnt_traj(ts_sample)
            trajectory = qs_sample
            duration = jnt_traj.get_duration() 
        
        return trajectory, duration, len(ts_sample)


    def cartesian_interpolator(self, current_postion_cartesian, goal):
        curr_xyz = []
        curr_xyz.append (current_postion_cartesian[0][3])
        curr_xyz.append (current_postion_cartesian[1][3])
        curr_xyz.append (current_postion_cartesian[2][3])
        goal_xyz = []
        goal_xyz.append (goal[0][3])
        goal_xyz.append (goal[1][3])
        goal_xyz.append (goal[2][3])
        rot_curr = [current_postion_cartesian[0][:3], current_postion_cartesian[1][:3], current_postion_cartesian[2][:3]]
        rot_goal = [goal[0][:3], goal[1][:3], goal[2][:3]]
        waypoints_xyz = self.linear_interpolator_function(curr_xyz, goal_xyz)
        waypoints_rot = self.rotational_interpolator (len(waypoints_xyz), rot_curr, rot_goal)
        return waypoints_xyz, waypoints_rot

    def generate_waypoints_jp (self, current_jp, goals, name, velocity):
        p = PsmKinematicsSolver(self.kinematics)
        current_postion_cartesian = p.compute_fk(current_jp)
        current_postion_cartesian=current_postion_cartesian.getA()
        t_xyz, t_rot = self.cartesian_interpolator(current_postion_cartesian, goals[0])
        waypoints_xyz = t_xyz
        waypoints_rot = t_rot.tolist()
        if len(goals)>1:
            for i in range (1,len(goals)):
                t_xyz, t_rot = self.cartesian_interpolator(goals[i-1], goals[i])
                t_xyz.pop(0)
                t_rot = t_rot.tolist()
                t_rot.pop(0)
                waypoints_xyz = waypoints_xyz + t_xyz
                waypoints_rot = waypoints_rot + t_rot
        duration = 0
        if name=="scissor":
            distance =0
            
            if len(goals)>1:
                if velocity == "nil":
                    velocity = input ("please enter a velocity for the cutting portion of the path:")
                for i in range (1,len(waypoints_xyz)-1):
                    temp = math.sqrt( math.pow((waypoints_xyz[i][0]-waypoints_xyz[i+1][0]),2) +math.pow((waypoints_xyz[i][1]-waypoints_xyz[i+1][1]),2) + math.pow((waypoints_xyz[i][2]-waypoints_xyz[i+1][2]),2) )
                    distance = distance + temp
                print("duration", duration)
                print("distance", distance)
                duration = distance / velocity
                
        # plot(waypoints_xyz)
        # plot_2d(waypoints_xyz)

        waypoints_jp = []
        for i in range(0,len(waypoints_xyz)):
            temp = waypoints_rot[i]
            for j in range(0,3):
                temp[j].append (waypoints_xyz[i][j])
            temp.append ([0,0,0,1])
            temp_jp = p.compute_ik(np.matrix(temp))
            waypoints_jp.append(temp_jp)
        waypoints = self.toppra (current_jp, waypoints_jp[len(waypoints_jp)-1], waypoints_jp, name, duration)
        return waypoints
    
    def generate_traj(self, current_jp, goals = [], name = "", velocity = "nil"):
        import time
        # get the start time
        st = time.time()

        durations = []
        points = []
        tmp, duration, point = self.generate_waypoints_jp ( current_jp, goals[0], name, velocity)
        waypoints = tmp.tolist()
        durations.append (duration)
        points.append (point)
        if len(goals)>1:
            for i in range (1, len(goals)):
                temp, duration, point = self.generate_waypoints_jp ( waypoints[len(waypoints)-1], goals[i], name, velocity)
                waypoints = waypoints + temp.tolist()
                durations.append (duration)
                points.append (point)

        
        waypoints_cartesian = []
        for i in range(0,len(waypoints)):
            p = PsmKinematicsSolver(self.kinematics)
            point = p.compute_fk(waypoints[i])
            point = point.getA()
            waypoints_cartesian.append(point)


        et = time.time()
        # get the execution time
        elapsed_time = et - st
        print('Execution time:', elapsed_time, 'seconds')
        print("number of points", len(waypoints))
        return waypoints, durations, points, waypoints_cartesian
    
    def generate_traj_jaw(self, current_jaw, goal_data):
        print ("generating jaw trajectory ........")
        print ("data", goal_data)
        goal = 0.0
        int_angle = self.interpolating_angle_jaw * (3.14/18)
        trajectory = []
        if goal_data[0] == 'o':
            goal = self.jaw_angle_open * (3.14/18)
            pub = current_jaw
            while pub <  goal:
                pub = pub + int_angle
                trajectory.append(pub)

        elif goal_data[0] == 'c':
            goal = self.jaw_angle_close * (3.14/18)
            
            pub = current_jaw
            while pub >  goal:
                pub = pub - int_angle
                trajectory.append(pub)
        else:
            print ("invalid choice")
        return trajectory, (self.rate* len(trajectory)), len(trajectory)


    def plot (self, temp):
        import matplotlib.pyplot as plt

        from mpl_toolkits.mplot3d import Axes3D

        print (len (temp))
        print (temp)


        fig = plt.figure(figsize=(4,4))

        ax = fig.add_subplot(111, projection='3d')

        for i in range (0,len(temp)):
            ax.scatter(temp[i][0], temp[i][1], temp[i][2]) # plot the point (2,3,4) on the figure

        plt.show()

    def plot_2d(self, temp):
        import numpy as np
        import matplotlib.pyplot as plt
        
        data = []
        for i in range (0,len(temp)):
            data.append([temp[i][0], temp[i][1]])
        # The data are given as list of lists (2d list)
        data = np.array(data)
        print(data)
        # Taking transpose
        x, y = data.T
        
        
        # plot our list in X,Y coordinates
        plt.scatter(x, y)
        plt.show()
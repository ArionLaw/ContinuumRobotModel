#!/usr/bin/env python3

from sensor_msgs.msg import JointState
import time
import numpy as np
import rospy
import math
from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006, RTS470007, RTS420007
from dvrk_planning_msgs.msg import TrajectoryStatus
from threading import Thread
from std_msgs.msg import Float64MultiArray

class Follower:
    def __init__(self):
        self.mode = ""

        self.max_open = 40
        self.max_close = 0
        self.distance_close_open = 0.0015
        self.point_ratio  = 15
        self.jaw_trajectory = []
        self.kinematics = LND400006()                #default kinematics, change from config
        self.offset_update_flag = 0
        self.offset = []
        self.follower_interrupt_flag = [1]
        self.current_trajectory = []
        self.plot_message = []
        self.plot_message_update_flag = 0
        self.back_flag = 0
        self.experiment_name = "test"
        self.total_offset = [0,0,0]
        self.shared_clicks = 0
        self.shared_invoked_index = []
        self.trajectory_edit_status =0

    def set_config(self, config):
        self.kinematics = eval(config['motion_generation_parameters']['kinematics']['tool']+ "()")
        self.max_open = config['cutting_parameters']['max_open']
        self.max_close = config['cutting_parameters']['max_close']
        self.distance_close_open = config['cutting_parameters']['distance_close_open']
        self.experiment_name = config['experiment_name']

    def set_offset(self, data):
        self.offset = data
        self.offset_update_flag = 1

    def plot(self):
        trajectory = self.current_trajectory
        msg = []
        for i in range(0, len(trajectory)):
            p = PsmKinematicsSolver(self.kinematics)
            point = p.compute_fk(trajectory[i])
            point = point.getA()
            msg.append(point[0][3]) 
            msg.append(point[1][3]) 
            msg.append(point[2][3])
        self.plot_message = msg
        self.plot_message_update_flag = 1 
        #if trajectory_edit_status==1:
        f = open(self.experiment_name, "a")
        f.write("\n")
        f.write(str(msg))
        f.close()

    def publish_plot_message(self, RvizTrajPublisher):
        dat = Float64MultiArray()
        dat.data = self.plot_message
        RvizTrajPublisher.publish(dat)
        self.plot_message_update_flag = 0

    def jaw_cutting(self, traj, jaw_position):
        #print ("trajectory", len(traj))
        self.max_open = self.max_open * (3.14/180)
        self.max_close = self.max_close * (3.14/180)
        inc_points = int((len(traj)-1)/ self.point_ratio)
        #print (inc_points)
        points = inc_points
        points = points +1
        p = PsmKinematicsSolver(self.kinematics)
        mode = 0
        curr = jaw_position
        l=0
        
        while (l+inc_points) < (len(traj)):
            start = traj[l]
            l= l+ inc_points
            end = traj[l]
            start = p.compute_fk(start)
            end = p.compute_fk(end)
            start = start.getA()
            end = end .getA()
            distance = math.sqrt( math.pow((start[0][3]-end[0][3]),2) +math.pow((start[1][3]-end[1][3]),2) + math.pow((start[2][3]-end[2][3]),2) )
            increment_points = points / (distance/ self.distance_close_open)
            increment = (self.max_open - self.max_close )/increment_points
            #print("increment",increment)
            increment = 0.06
            self.max_open = 1
            self.max_close = 0
            for i in range (0,points+5):
                if curr > self.max_open:
                    mode = 1
                elif curr < self.max_close:
                    mode = 0

                if mode == 0:
                    self.jaw_trajectory.append(curr + increment)
                    curr = curr + increment
                elif mode ==1 :
                    self.jaw_trajectory.append(curr - increment)
                    curr = curr - increment

    def offset_update_function(self, i, traj, cart_trajectory):
        import time
        # get the start time
        st = time.time()
        temp = traj.copy()

        self.total_offset[0] = self.total_offset[0] + self.offset[0]
        
        if self.offset[3]==1:
            self.back_flag =1
        else:
            self.total_offset[1] = self.total_offset[1] + self.offset[1]
            self.total_offset[2] = self.total_offset[2] + self.offset[2]
            self.shared_clicks = self.shared_clicks +1
            self.trajectory_edit_status = 1
            self.shared_invoked_index.append(i)
            while i< len(cart_trajectory):
                p = PsmKinematicsSolver(self.kinematics)
                # point = p.compute_fk(trajectory[i])
                # point = point.getA()
                point = cart_trajectory[i]
                #print (point)
                point[0][3] = point[0][3] + self.offset[0]
                point[1][3] = point[1][3] + self.offset[1]
                point[2][3] = point[2][3] + self.offset[2]
                

                temp[i] = p.compute_ik(np.matrix(point))

                i = i+1
            

        et = time.time()
        # get the execution time
        elapsed_time = et - st
        print('Execution time:', elapsed_time, 'seconds')

        self.offset_update_flag = 0
        return temp
    
    def set_follower_status(self, msg):
        self.follower_interrupt_flag = msg

    def follow_trajectory(self, traj, JointStatePublisher, durations, interpolated_points, name, cartesian_traj, StatusPublisher, RvizTrajPublisher, JawStatePublisher=[], jaw_position=[]):
        trajectory = traj.copy()
        self.current_trajectory = trajectory.copy()
        cartesian_trajectory = cartesian_traj.copy()
        Thread(target = self.plot).start()
        self.mode = name
        print ("Name:", self.mode)
        print("Following the trajectory ........")
        rate = durations[0]/interpolated_points[0]
        durations_index = 1
        points_index = 0
        curr = 0

        import time
        # get the start time
        st = time.time()
        
        if self.mode == "scissor":
            self.jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)
        i=0
        while i<len(trajectory):
            if rospy.is_shutdown():
                return
            if self.follower_interrupt_flag[0] == 0:
                print("Interupting follwer")
                return

            if self.offset_update_flag == 1:
                trajectory = self.offset_update_function(i, trajectory, cartesian_trajectory).copy()
                self.current_trajectory = trajectory.copy()
                Thread(target = self.plot).start()

            if self.plot_message_update_flag == 1:
                self.publish_plot_message(RvizTrajPublisher)

            if self.back_flag == 1:
                if i>15:
                    i = i - 15 
                else: 
                    i = 0
                self.back_flag = 0
                #print("recd")
            
            dat = JointState()
            dat.position = trajectory[i]
            JointStatePublisher.publish(dat)

            statusDat = TrajectoryStatus()
            statusDat.trajectory_name = self.mode
            statusDat.percentage_completed = (i/len(trajectory))*100
            statusDat.completed_trajectory_points = i
            statusDat.total_trajectory_points = len(trajectory)
            StatusPublisher.publish(statusDat)
            time.sleep(rate/2)
            if self.mode == "scissor":
                dat = JointState()
                dat.position = [self.jaw_trajectory[i]]
                JawStatePublisher.publish(dat)
            if ((i-curr) >= interpolated_points[points_index]):
                points_index = points_index +1
                rate = durations[durations_index]/interpolated_points[points_index]
                curr = i
                durations_index = durations_index + 1

                if self.mode == "scissor":
                    self.jaw_cutting(trajectory[0:interpolated_points[points_index]-1],jaw_position)

            time.sleep(rate/2)
            i=i+1
            
        if self.mode == "scissor":
            et = time.time()
            # get the execution time
            elapsed_time = et - st
            f = open("time-"+ self.experiment_name, "a")
            f.write("\n")
            f.write("generated time: " + str(durations[0]))
            f.write("\n")
            f.write("actual time: " + str(elapsed_time))
            f.write("\n")
            f.write("number of times shared autonomy is invoked: " + str(self.shared_clicks))
            f.write("\n")
            f.write("total offset: " + str(self.total_offset))
            f.write("\n")
            f.write("trajectory indeces that shared was invoked at: " + str(self.shared_invoked_index))
            f.close()
            
            self.shared_clicks =0
            self.shared_invoked_index = []
            self.total_offset = [0,0,0]
        print("done")

    def follow_jaw_trajectory(self, trajectory, JointStatePublisher, duration, interpolated_points):
        if len(trajectory)>0:
            print("Following the jaw trajectory ........")
            rate = duration/interpolated_points
            
            for i in range(0,len(trajectory)):
                if rospy.is_shutdown():
                    return
                if self.follower_interrupt_flag[0] == 0:
                    return
                dat = JointState()
                dat.position = [trajectory[i]]
                JointStatePublisher.publish(dat)
                time.sleep(rate)
            print("done")
        else:
            print ("Already here....")

   
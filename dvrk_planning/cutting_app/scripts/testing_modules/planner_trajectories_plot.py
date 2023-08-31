#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState
import time
import numpy as np
import roboticstoolbox as rtb

import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import matplotlib.pyplot as plt

from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006

class Constraints:
    velocity_constraints = []
    accln_constraints = []

class CartesianGoal:
    goals = []
    waypoints = []
    rotation_goals = []
    rotation_waypoints = []

class PlannerTrajectoryPlot:
    def __init__(self):
        self.current_position = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.updated = 0
        self.goal_js = [0.0,0.0,0.0,0.0,0.0,0.0]
        self.trajectory = []

    def position_callback(self, msg):
        self.current_position = msg.position
        #print("updated")
        self.updated = 1

    def follow_trajectory(self):
        for i in range(0,len(self.trajectory)):
            dat = JointState()
            dat.position = self.trajectory[i]
            print(dat)
            JointStatePublisher.publish(dat)
            time.sleep(0.1)
        print("done")

    def generate_trajectory_rtb(self):
        traj = rtb.jtraj(self.current_position, self.goal_js, 100)
        self.trajectory = traj.q
        
        
        fig, axs = plt.subplots(3)
        axs[0].plot(list(range(1, 101)), traj.q)
        axs[1].plot(list(range(1, 101)), traj.qd)
        axs[2].plot(list(range(1, 101)), traj.qd)

        axs[2].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (rad)")
        axs[1].set_ylabel("Velocity (rad/s)")
        axs[2].set_ylabel("Acceleration (rad/s2)")
        #plt.show()

        self.generate_trajectory_toppra()

        #follow_trajectory()

    def generate_trajectory_toppra(self):

        # Parameters
        N_samples = 2
        dof = 6
        way_pts = np.array([self.current_position, self.goal_js])
        print(way_pts)
        
        ss = np.linspace(0, 1, N_samples)

        vlims = 10 + np.random.rand(dof) * 20
        alims = 10 + np.random.rand(dof) * 2
        

        path = ta.SplineInterpolator(ss, way_pts)
        pc_vel = constraint.JointVelocityConstraint(vlims)
        pc_acc = constraint.JointAccelerationConstraint(alims)
        instance = algo.TOPPRA([pc_vel, pc_acc], path, parametrizer="ParametrizeConstAccel")
        jnt_traj = instance.compute_trajectory()
        ts_sample = np.linspace(0, jnt_traj.duration, 100)
        qs_sample = jnt_traj(ts_sample)
        qds_sample = jnt_traj(ts_sample, 1) 
        qdds_sample = jnt_traj(ts_sample, 2)

        print(qds_sample)
        fig, axs = plt.subplots(3, 1, sharex=True)
        for i in range(path.dof):
            # plot the i-th joint trajectory
            axs[0].plot(ts_sample, qs_sample[:, i], c="C{:d}".format(i))
            axs[1].plot(ts_sample, qds_sample[:, i], c="C{:d}".format(i))
            axs[2].plot(ts_sample, qdds_sample[:, i], c="C{:d}".format(i))
        axs[2].set_xlabel("Time (s)")
        axs[0].set_ylabel("Position (rad)")
        axs[1].set_ylabel("Velocity (rad/s)")
        axs[2].set_ylabel("Acceleration (rad/s2)")
        plt.show()

        self.trajectory = qs_sample
        #follow_trajectory()

    def accept_goal(self):
        self.goal_js = [0.39965444803237915, -0.5209582448005676, 0.13364960253238678, -0.24901601672172546, 1.0345871448516846, -1.196233332157135]
        while not self.updated:
            time.sleep(0.1)
        if self.updated==1:
            #generate_trajectory_toppra()
            self.generate_trajectory_rtb()

    def test_joint_pos_sequence(self):
        np.set_printoptions(precision = 4, suppress = True)

        #t = Timer(timer_repeat_times = 100)
        p = PsmKinematicsSolver(LND400006())

        joint_change = 0.1

        joint_pos = [0.0, 0.0, joint_change, 0.0, 0.0, 0.0]
        self.test_fk_ik(p, joint_pos)

        joint_pos[1] = -joint_change
        self.test_fk_ik(p, joint_pos)

        joint_pos[0] = joint_change
        self.test_fk_ik(p, joint_pos)

        joint_pos[3] = joint_change
        joint_pos =  [0.27342814207077026, -0.5036982893943787, 0.11685218662023544, 0.011314896866679192, 0.5170695781707764, -0.27725905179977417]

        self.test_fk_ik(p, joint_pos)

        joint_pos[4] = joint_change
        joint_pos = [0.0007754408870823681, 0.07288364320993423, 0.10093997418880463, -1.1464053386589512e-05, -0.05122923105955124, -2.1644532353093382e-05]
        self.test_fk_ik(p, joint_pos)

        joint_pos[5] = joint_change

        joint_pos = [0.0007719629211351275, -0.5215796828269958, 0.11377278715372086, 5.5424960009986535e-05, 0.5348151326179504, -6.279001536313444e-05]
        self.test_fk_ik(p, joint_pos, test_time= True)

    def test_fk_ik(self, p, joint_pos, test_time = False):
        output_fk = p.compute_fk(joint_pos)
        output_jp = p.compute_ik(output_fk)  
        
        # Print results
        print("Input joint pos: \n", joint_pos)
        print("FK: \n", output_fk)
        print("IK: \n", np.array(output_jp))

        print("-------------------------------------")


if __name__ == '__main__':
    ptp = PlannerTrajectoryPlot()

    rospy.init_node('MotionGenerator', anonymous= True)
    rospy.Subscriber("/PSM2/measured_js",JointState, ptp.position_callback)
    JointStatePublisher = rospy.Publisher('/PSM2/servo_jp', JointState, queue_size = 1)
    ptp.accept_goal()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
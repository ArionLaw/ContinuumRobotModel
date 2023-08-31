#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point

from sensor_msgs.msg import JointState

from visualization_msgs.msg import Marker
from threading import Thread
from std_msgs.msg import Float64MultiArray

class RvizInterface:
    def __init__(self):
        self.goals = []
        self.curr_position = []
        self.jaw_position = []

    def plot(self):
        marker = Marker()
        marker.header.frame_id = "base_link";
        marker.type = marker.POINTS
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.005
        marker.scale.y = 0.005

        marker.color.r = 0.5
        marker.color.g = 1.0
        marker.color.b = 0.5
        marker.color.a = 1.0
        marker.lifetime = rospy.Duration()

        temp_points = []
        for i in range(0, len(self.goals)):

            temp = Point()
            temp.x = self.goals[i][0]
            temp.y = self.goals[i][1]
            temp.z = self.goals[i][2]
            temp_points.append(temp)
        
        if len(temp_points) > 1:
            marker.points = temp_points
            MarkerPublisher.publish(marker)


    def point_callback(self, msg):
        temp = [0.0,0.0,0.0]
        temp[0] = msg.point.x
        temp[1] = msg.point.y
        temp[2] = msg.point.z
        self.goals.append(temp)
        self.plot()

    def trajectory_callback(self, msg):
        temp = []
        i=0
        while i< len(msg.data)-3:
            temp.append([msg.data[i], msg.data[i+1], msg.data[i+2]])
            i = i+3

        #print()
        #print("temp",temp)
        self.goals = []
        self.goals = temp.copy()
        self.plot()



    def jaw_callback(self, msg):
        self.jaw_position = msg.position

    def position_callback(self, msg):
        self.curr_position = msg.position

    def translator(self):
        while not rospy.is_shutdown():
            if len(self.jaw_position)>0 and len(self.curr_position)>0:
                dat = JointState()
                dat.name = ["outer_yaw","outer_pitch","pitch_bottom_joint","pitch_end_joint","pitch_top_joint","pitch_front_joint","outer_insertion","outer_roll","outer_wrist_pitch","outer_wrist_yaw","tool_gripper1_joint","jaw"]
                dat.position.append(self.curr_position [0])
                dat.position.append(self.curr_position [1])
                dat.position.append( - self.curr_position [1])
                dat.position.append(self.curr_position [1])
                dat.position.append( - self.curr_position [1])
                dat.position.append(self.curr_position [1])
                dat.position.append(self.curr_position [2])
                dat.position.append(self.curr_position [3])
                dat.position.append(self.curr_position [4])
                dat.position.append(self.curr_position [5])
                dat.position.append( - self.jaw_position [0])
                dat.position.append(self.jaw_position [0])
                JointStatePublisher.publish(dat)
            rospy.sleep(0.1)

    def send_points(self):
        while not rospy.is_shutdown():
            choice = input("Enter Y to send goals, and N to discard points and start over:  ")
            #try:
            temp = list(choice)
            if temp[0] == 'y' or temp[0] == 'Y':
                print (self.goals)
                self.goals = []
            elif temp[0] == 'n' or temp[0] == 'N':
                
                self.goals = []
                print ("select two points for it to put the new plot")
            #except:
                #print ("something went wrong, try again ......")
                #send_points()

if __name__ == '__main__':
    ri = RvizInterface()

    rospy.init_node('rviz_interface', anonymous= True)
    rospy.Subscriber("/clicked_point",PointStamped, ri.point_callback)
    rospy.Subscriber("/plot/trajectory/rviz",Float64MultiArray, ri.trajectory_callback)
    rospy.Subscriber("/PSM2/measured_js",JointState, ri.position_callback)
    rospy.Subscriber("/PSM2/jaw/measured_js",JointState, ri.jaw_callback)
    JointStatePublisher = rospy.Publisher('/rviz/joint_states', JointState, queue_size = 1)
    MarkerPublisher = rospy.Publisher('/visualization_marker', Marker, queue_size = 1)   
    Thread(target = ri.translator).start()
    Thread(target = ri.send_points).start() 

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("shutting down")
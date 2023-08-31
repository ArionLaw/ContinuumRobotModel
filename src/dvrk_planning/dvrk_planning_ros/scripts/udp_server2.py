#!/usr/bin/env python3
import socket
import rospy
import numpy as np
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState

localIP = "10.42.0.1"
localPort = 34567
bufferSize = 1024

#create Datagram socket
UDPServerSocket = socket.socket(family=socket.AF_INET, type= socket.SOCK_DGRAM)
UDPServerSocket.bind((localIP,localPort))
print("UDP server is up and listening ")

class JointSubscriber:
    def __init__(self):
        topic_name = "/PSM2/measured_js"
        self.sub = rospy.Subscriber(topic_name, JointState, self.callback)

        print("waiting for ... ", topic_name)
        rospy.wait_for_message(topic_name, JointState)
        print("Finished waiting for, ", topic_name)

        self.joint_positions = []
    def callback(self,data):
        self.joint_positions = data.position
        # str_msg = "{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(
        #     self.joint_positions[0],
        #     self.joint_positions[1],
        #     self.joint_positions[2],
        #     self.joint_positions[3],
        #     self.joint_positions[4],
        #     self.joint_positions[5])
        # print(str_msg)
    def get_joint_positions(self):
        return self.joint_positions

if __name__ == '__main__':

    try:
        # ROS things
        rospy.init_node('talker', anonymous=True)
        joint_subscriber = JointSubscriber()
        
        pub = rospy.Publisher('servo_cp_follow', TransformStamped, queue_size=10)
        tf_stamped_msg = TransformStamped()
        tf_stamped_msg.header.frame_id = "world"
        tf_stamped_msg.child_frame_id = "input_controller"
        rate = rospy.Rate(100) # 100hz
        bytesAddressPair = UDPServerSocket.recvfrom(bufferSize)
        address = bytesAddressPair[1]
        while not rospy.is_shutdown():
            #sending joints to client
            joint_pos = joint_subscriber.get_joint_positions()
            print(joint_pos)
            str_msg = "{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}".format(
                joint_pos[0],
                joint_pos[1],
                joint_pos[2],
                joint_pos[3],
                joint_pos[4],
                joint_pos[5])
            print(str_msg)
            bytesToSend = str.encode(str_msg)
            UDPServerSocket.sendto(bytesToSend, address)
    except rospy.ROSInterruptException:
        pass

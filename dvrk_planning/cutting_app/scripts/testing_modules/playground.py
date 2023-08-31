#!/usr/bin/env python3
import cv2
from cv_bridge import CvBridge
import rospy
from sensor_msgs.msg import Image
import numpy as np

def callback(data):
    # Used to convert between ROS and OpenCV images
    br = CvBridge()

    # Output debugging information to the terminal
    #rospy.loginfo("receiving video frame")
    # Convert ROS Image message to OpenCV image
    current_frame = br.imgmsg_to_cv2(data)

    cv2.imshow("camera", current_frame)

    cv2.waitKey(1)


def listener():
    rospy.init_node("video_sub_py", anonymous=True)
    rospy.Subscriber("/rgb_publisher/color/image", Image, callback)

    rospy.spin()

    cv2.destroyAllWindows()


if __name__ == "__main__":
    listener()
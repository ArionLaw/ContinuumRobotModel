import argparse
import tf
import rospy
import PyKDL

from dvrk_planning_ros.utils import quaternion_rotation_matrix

parser = argparse.ArgumentParser()
parser.add_argument('parent')
parser.add_argument('child')
args = parser.parse_args()

rospy.init_node('tf_listener')
t = tf.TransformListener()
rospy.sleep(0.5) # Has to wait to work
_, quat_rot  = t.lookupTransform(args.parent, args.child, rospy.Time(0))

print("matrix: ", quaternion_rotation_matrix(quat_rot[3], quat_rot[0], quat_rot[1], quat_rot[2]))
# print(PyKDL.Quaternion())
print("x: ", quat_rot[0])
print("y: ", quat_rot[1])
print("z: ", quat_rot[2])
print("w: ", quat_rot[3])

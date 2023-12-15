import argparse
import tf
import rospy


parser = argparse.ArgumentParser()
parser.add_argument('parent')
parser.add_argument('child')
args = parser.parse_args()
t = tf.TransformListener()
_, quat_rot  = t.lookupTransform(args.child, args.parent, rospy.Time(0))

print("x: ", quat_rot[0])
print("y: ", quat_rot[1])
print("z: ", quat_rot[2])
print("w: ", quat_rot[3])

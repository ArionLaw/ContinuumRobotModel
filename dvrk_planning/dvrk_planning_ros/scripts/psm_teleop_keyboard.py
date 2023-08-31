#!/usr/bin/env python

from __future__ import print_function
from doctest import run_docstring_examples

import threading

# import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
i : +x
, : -x
j : +y
l : -y
t : up (+z)
b : down (-z)
I : +rot_x
< : -rot_x
J : +rot_y
L : -rot_y
T : +rot_z
B : -rot_z
------
[ : jaw close
] : jaw open

q/z : increase/decrease max speeds by 10%
w/x : increase/decrease only linear speed by 10%
e/c : increase/decrease only angular speed by 10%
CTRL-C to quit
"""

moveBindings = {
        'i':( 1, 0, 0, 0, 0, 0),
        ',':(-1, 0, 0, 0, 0, 0),
        'j':( 0, 1, 0, 0, 0, 0),
        'l':( 0,-1, 0, 0, 0, 0),
        't':( 0, 0, 1, 0, 0, 0),
        'b':( 0, 0,-1, 0, 0, 0),
        'I':( 0, 0, 0, 1, 0, 0),
        '<':( 0, 0, 0,-1, 0, 0),
        'J':( 0, 0, 0, 0, 1, 0),
        'L':( 0, 0, 0, 0,-1, 0),
        'T':( 0, 0, 0, 0, 0, 1),
        'B':( 0, 0, 0, 0, 0,-1),
    }

speedBindings={
        'q':(1.1,1.1),
        'z':(.9,.9),
        'w':(1.1,1),
        'x':(.9,1),
        'e':(1,1.1),
        'c':(1,.9),
    }

jawBindings = {
    '[': -1,
    ']':  1,
}

class PublishTransformThread(threading.Thread):
    def __init__(self, rate):
        super(PublishTransformThread, self).__init__()
        self.publisher = rospy.Publisher('/keyboard/twist', TwistStamped, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.speed = 0.0
        self.rot_speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, rot_x, rot_y, rot_z, speed, rot_speed):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.rot_x = rot_x
        self.rot_y = rot_y
        self.rot_z = rot_z

        self.speed = speed
        self.rot_speed = rot_speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.condition.acquire()
        self.condition.notify()
        self.condition.release()

        self.done = True
        self.join()

    def run(self):
        msg = TwistStamped()
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            msg.twist.linear.x = self.x * self.speed
            msg.twist.linear.y = self.y * self.speed
            msg.twist.linear.z = self.z * self.speed
            msg.twist.angular.x = self.rot_x * self.rot_speed
            msg.twist.angular.y = self.rot_y * self.rot_speed
            msg.twist.angular.z = self.rot_z * self.rot_speed
            msg.header.stamp = rospy.Time.now()

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)

class PublishJointStateThread(threading.Thread):
    def __init__(self, rate):
        super(PublishJointStateThread, self).__init__()
        self.publisher = rospy.Publisher('/keyboard/joint_state', JointState, queue_size = 1)
        self.jpos = 0.0
        self.speed = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, jpos, speed):
        self.condition.acquire()
        self.jpos = jpos
        self.speed = speed
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.condition.acquire()
        self.condition.notify()
        self.condition.release()
        
        self.done = True
        self.join()

    def run(self):
        js_msg = JointState()
        js_msg.name = ["jaw"]
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            js_msg.position = [self.jpos * self.speed]

            self.condition.release()

            js_msg.header.stamp = rospy.Time.now()
            # Publish.
            self.publisher.publish(js_msg)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, rot_speed):
    return "currently:\tspeed %s rot_speed %s " % (speed,rot_speed)

def key_update_tf(key, speed, rot_speed, thread):
    if key in moveBindings.keys():
        x = moveBindings[key][0]
        y = moveBindings[key][1]
        z = moveBindings[key][2]
        rot_x = moveBindings[key][3]
        rot_y = moveBindings[key][4]
        rot_z = moveBindings[key][5]
        thread.update(x, y, z, rot_x, rot_y, rot_z, speed, rot_speed)
        return True
    else:
        return False

def key_update_js(key, speed, thread):
    if key in jawBindings.keys():
        thread.update(jawBindings[key], speed)
        return True
    else:
        return False

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_twist_keyboard')

    speed = rospy.get_param("~speed", 0.001)
    rot_speed = rospy.get_param("~rot_speed", 0.05)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.0)
    if key_timeout == 0.0:
        key_timeout = None

    pub_tf_thread = PublishTransformThread(repeat)
    pub_js_thread = PublishJointStateThread(repeat)

    x = 0
    y = 0
    z = 0
    rot_x = 0
    rot_y = 0
    rot_z = 0

    status = 0

    try:
        print(msg)
        print(vels(speed,rot_speed))
        while(1):
            key = getKey(key_timeout)
            if key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                rot_speed = rot_speed * speedBindings[key][1]

                print(vels(speed,rot_speed))
                if (status == 14):
                    print(msg)
                status = (status + 1) % 15
                continue
            elif key_update_tf(key, speed,rot_speed, pub_tf_thread):
                continue
            elif key_update_js(key, rot_speed, pub_js_thread):
                continue
            elif (key == '\x03'):
                break

    except Exception as e:
        print(e)

    finally:
        pub_tf_thread.stop()
        pub_js_thread.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

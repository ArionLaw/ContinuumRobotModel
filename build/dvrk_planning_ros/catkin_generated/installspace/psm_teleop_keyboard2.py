#!/usr/bin/env python3

import threading
import rospy

from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import JointState

import pygame

# sudo apt install python3-pygame


class PublishTransformThread(threading.Thread):
    def __init__(self, rate):
        super(PublishTransformThread, self).__init__()
        self.publisher = rospy.Publisher('/keyboard/twist', TwistStamped, queue_size = 1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.th = 0.0
        self.increment = 0.0
        self.rot_increment = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, x, y, z, rot_x, rot_y, rot_z, increment, rot_increment):
        self.condition.acquire()
        self.x = x
        self.y = y
        self.z = z
        self.rot_x = rot_x
        self.rot_y = rot_y
        self.rot_z = rot_z

        self.increment = increment
        self.rot_increment = rot_increment
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
            msg.twist.linear.x = self.x * self.increment
            msg.twist.linear.y = self.y * self.increment
            msg.twist.linear.z = self.z * self.increment
            msg.twist.angular.x = self.rot_x * self.rot_increment
            msg.twist.angular.y = self.rot_y * self.rot_increment
            msg.twist.angular.z = self.rot_z * self.rot_increment
            msg.header.stamp = rospy.Time.now()

            self.condition.release()

            # Publish.
            self.publisher.publish(msg)

class PublishJointStateThread(threading.Thread):
    def __init__(self, rate):
        super(PublishJointStateThread, self).__init__()
        self.publisher = rospy.Publisher('/keyboard/joint_state', JointState, queue_size = 1)
        self.jpos = 0.0
        self.increment = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def update(self, jpos, increment):
        self.condition.acquire()
        self.jpos = jpos
        self.increment = increment
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

            js_msg.position = [self.jpos * self.increment]

            self.condition.release()

            js_msg.header.stamp = rospy.Time.now()
            # Publish.
            self.publisher.publish(js_msg)


msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
d : +x
a : -x
w : +y
s : -y
e : up (+z)
q : down (-z)
D : +rot_x
A : -rot_x
W : +rot_y
S : -rot_y
E : +rot_z
Q : -rot_z
------
o : jaw close
p : jaw open

CTRL-C to quit
"""

if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard')
    print(msg)

    increment = rospy.get_param("~increment", 0.0005) #m
    rot_increment = rospy.get_param("~rot_increment", 0.05)
    rate = rospy.get_param("~rate", 20) # Hz
    repeat = rospy.get_param("~repeat_rate", 0.0)

    print("increment: ", increment, "m")
    print("rate: ", rate, "Hz")
    print("speed if you hold down keys: ", increment * rate, " m/s")

    pub_tf_thread = PublishTransformThread(repeat)
    pub_js_thread = PublishJointStateThread(repeat)

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(1)

    done = False
    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        clock.tick(rate)
        x = 0
        y = 0
        z = 0
        rot_x = 0
        rot_y = 0
        rot_z = 0
        jaw_inc = 0
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                run = False
            # if event.type == pygame.KEYDOWN:
            #     print(pygame.key.name(event.key))

        keys = pygame.key.get_pressed()
        is_key_pressed = False
        # Linear
        if keys[pygame.K_w]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_y = 1
            else:
                y = 1
            is_key_pressed = True
        if keys[pygame.K_s]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_y = -1
            else:
                y = -1
            is_key_pressed = True
        if keys[pygame.K_d]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_x = 1
            else:
                x = 1
            is_key_pressed = True
        if keys[pygame.K_a]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_x = -1
            else:
                x = -1
            is_key_pressed = True
        if keys[pygame.K_e]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_z = 1
            else:
                z = 1
            is_key_pressed = True
        if keys[pygame.K_q]:
            if keys[pygame.K_LSHIFT] or keys[pygame.K_RSHIFT]:
                rot_z = -1
            else:
                z = -1
            is_key_pressed = True
        if keys[pygame.K_p]:
            jaw_inc = 1
            is_key_pressed = True
        if keys[pygame.K_o]:
            jaw_inc = -1
            is_key_pressed = True

        if(is_key_pressed):
            # print("x: {}, y: {}, z: {}".format(x, y, z))
            # print("rot_x: {}, rot_y: {}, rot_z: {}".format(rot_x, rot_y, rot_z))
            # print("jaw_inc: {}".format(jaw_inc))
            pub_tf_thread.update(x, y, z, rot_x, rot_y, rot_z, increment, rot_increment)
            pub_js_thread.update(jaw_inc, rot_increment)

    pub_tf_thread.stop()
    pub_js_thread.stop()

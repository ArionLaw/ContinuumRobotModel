#!/usr/bin/env python3


import rospy
import pygame
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
d : +x
a : -x
w : +y
s : -y
e : up (+z)
q : down (-z)
n : reduce offset_distance by 10%
m : increase offset_distance by 10%
t : switch to teleop
g : switch to motion generator

CTRL-C to quit
"""

if __name__=="__main__":
    rospy.init_node('teleop_twist_keyboard_shared_autonomy')

    print(msg)

    increment = rospy.get_param("~increment", 0.00005) #m
    rot_increment = rospy.get_param("~rot_increment", 0.05)
    rate = rospy.get_param("~rate", 20) # Hz
    repeat = rospy.get_param("~repeat_rate", 0.0)
    mode = -1
    
    ToggleStatePublisher = rospy.Publisher('/motion_generator/toggle/mode', Joy, queue_size = 1)
    Publisher = rospy.Publisher("/motion_generator/trajectory/offset", Float32MultiArray, queue_size = 1)

    #print("increment: ", increment, "m")
    #print("rate: ", rate, "Hz")
    offset_distance = 0.0005
    print("offset_distance if you hold down keys: ", offset_distance, " m")

    # pub_tf_thread = PublishTransformThread(repeat)
    # pub_js_thread = PublishJointStateThread(repeat)

    pygame.init()
    screen = pygame.display.set_mode((640, 200))
    pygame.display.set_caption('Click Here and Press Keys to Teleop')
    pygame.mouse.set_visible(1)

    done = False
    clock = pygame.time.Clock()
    while not rospy.is_shutdown():
        clock.tick(rate)
        x = 0
        back =0
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
        if keys[pygame.K_b]:
            back = 1
            is_key_pressed = True
        if keys[pygame.K_p]:
            jaw_inc = 1
            is_key_pressed = True
        if keys[pygame.K_o]:
            jaw_inc = -1
            is_key_pressed = True

        if keys[pygame.K_n]:
            offset_distance = 0.9 * offset_distance
            #is_key_pressed = True
            print("offset_distance if you hold down keys: ", offset_distance, " m")

        if keys[pygame.K_m]:
            offset_distance = 1.1 * offset_distance
            #is_key_pressed = True    .
            print("offset_distance if you hold down keys: ", offset_distance, " m")

        if keys[pygame.K_t] and mode != 0:
            dat = Joy()
            dat.buttons = [0]
            ToggleStatePublisher.publish(dat)
            is_key_pressed = True
            mode = 0
            print("disabling motion generator and switching to teleop.......")
        
        if keys[pygame.K_g] and mode != 1:
            dat = Joy()
            dat.buttons = [1]
            ToggleStatePublisher.publish(dat)
            is_key_pressed = True
            mode = 1
            print("disabling teleop and switching to motion generator.......")

        if(is_key_pressed):
            # print("x: {}, y: {}, z: {}".format(x, y, z))
            # print("rot_x: {}, rot_y: {}, rot_z: {}".format(rot_x, rot_y, rot_z))
            # print("jaw_inc: {}".format(jaw_inc))
            # print(x)
            dat = Float32MultiArray()
            dat. data = [ x* offset_distance, y*offset_distance, z*offset_distance, back]
            #print(back)
            Publisher.publish(dat)


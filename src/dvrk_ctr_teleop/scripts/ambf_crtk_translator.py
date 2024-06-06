# Import the Client from ambf_client package
from ambf_client import Client
import argparse
import threading
import rospy
import sys
import time
import numpy as np
import random
import pdb

from  sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

class JointErrorsModel:
    def __init__(self, num_joints, errors_distribution_deg):

        self.errors_list_deg = errors_distribution_deg
        self.joint_erros_rad = []
        self.num_jnts = num_joints
        for i in range(self.num_jnts):
            self.joint_erros_rad.append(np.deg2rad(random.choice(self.errors_list_deg)))
        print(np.rad2deg(self.joint_erros_rad))

    def _size_check(self, q, joint_mask):
        qs_size = len(q)
        jnt_mask_size = len(joint_mask)
        if qs_size > self.num_jnts:
            print("ERROR! size of joint positions: ", qs_size, " > num of joints: ", self.num_jnts)
            print("IGNORING! ")
            return False

        if jnt_mask_size > qs_size:
            print("ERROR! JOINT MASK: ", joint_mask, " > size of joint positions: ", qs_size)
            print("IGNORING! ")
            return False
        return True

    def add_to_joints(self, q, joint_mask):
        if self._size_check(q, joint_mask):
            for i in range(len(joint_mask)):
                if joint_mask[i]:
                    q[i] = q[i] + self.joint_erros_rad[i]
        return q

    def remove_from_joints(self, q, joint_mask):
        if self._size_check(q, joint_mask):
            for i in range(len(joint_mask)):
                if joint_mask[i]:
                    q[i] = q[i] - self.joint_erros_rad[i]
        return q

class PublishThread(threading.Thread):
    def __init__(self, cb1):
        super(PublishThread, self).__init__()
        self.cb1 = cb1
        #self.cb2 = cb2
        self.done = False

        self.start()
    def stop(self):
        self.done = True
        self.join()

    def run(self):
        while not self.done:
            self.cb1()
            #self.cb2()
            time.sleep(0.01)

index_to_correct_name = [
    'baselink-yawlink', 
    'yawlink-pitchbacklink',
    'pitchendlink-maininsertionlink',
    'maininsertionlink-toolrolllink',
    'notch_1-notch_2',
    'notch_2-notch_3',
    'notch_3-notch_4',
    'notch_4-notch_5',
    'notch_5-notch_6',
    'notch_6-notch_7',
    'notch_7-notch_8',
    'notch_8-notch_9',
    'notch_9-notch_10',
    'notch_10-gripper_holder',
    ]

class PSMTranslator:
    def __init__(self, crtk_namespace = '/PSM2', initial_jps = []):
        self._client = Client()
        self._client.connect()
        time.sleep(0.2)

        if crtk_namespace == '/PSM1':
            self.namespace = 'ambf/env/psm1'
            self.base_handle = self._client.get_obj_handle('psm1/baselink')
            self.ee_body = self._client.get_obj_handle('/ambf/env/psm1/gripper_1')
        elif crtk_namespace =='/PSM2':
            self.namespace = 'ambf/env/psm2'
            self.base_handle = self._client.get_obj_handle('psm2/baselink')
            self.ee_body = self._client.get_obj_handle('/ambf/env/psm2/EEPose')
        else:
            raise(ValueError('Invalid crtk_namespace'))
        
        self.sensor = self._client.get_obj_handle(self.namespace + '/Sensor0')
        self.actuators = []
        self.actuators.append(self._client.get_obj_handle(self.namespace + '/Actuator0'))
        time.sleep(0.5)
        self.grasped = [False, False, False]
        self.graspable_objs_prefix = ["Needle", "Thread", "Puzzle"]

        self.initalize_pos_rpy(crtk_namespace)
        time.sleep(0.2)
        self.measured_js_pub = rospy.Publisher(crtk_namespace + "/measured_js", JointState, queue_size = 10)
        self.measured_ee_pose_pub = rospy.Publisher(crtk_namespace + "/measured_ee_pose", Pose, queue_size = 1)

        self._num_joints = 15
        self._joint_error_model = JointErrorsModel(self._num_joints, errors_distribution_deg= [0])
        self.joint_names = self.base_handle.get_joint_names() # Might be useful later

        self.jaw_pos = 0.0
        print(self.joint_names)

        gm_js = JointState()

        initial_pos = []
        for i in range(0,15):
            if i == 2:
                initial_pos.append(1.0)
            else:
                initial_pos.append(0.0)

        gm_js.position = initial_pos
        self.servo_jp(gm_js)

        self.pub_thread = PublishThread(self.measured_js)

        self.servo_jp_sub = rospy.Subscriber(crtk_namespace + "/servo_jp", JointState, self.servo_jp)

    def initalize_pos_rpy(self,crtk_namespace):
        
        for i in range(0,16):
            self.base_handle.set_joint_pos(i,0.0)
            time.sleep(0.05)
        self.base_handle.set_joint_pos('notch_9-notch_10', 0)

        if crtk_namespace == '/PSM1':
            self.base_handle.set_pos(0.5,1,2)
            # self.base_handle.set_rot([-0.9227369876445654,
            #     -0.17858953119523796,
            #     -0.08860286458914572,
            #     0.3298662810392971])
        elif crtk_namespace =='/PSM2':
            self.base_handle.set_pos(-0.5,1,2)
            #self.base_handle.set_rpy(0.698099, 0.523600, -3.13158)
            # self.base_handle.set_rot([-0.8491677293967367,
            #   -0.4200572904600633,
            #   -0.1001969776665232,
            #   0.3040174431657418])
        
        self.base_handle.set_rpy(0,0,3.14)
        #self.base_handle.set_rpy(0,0,0)

        for i in range(0,16):
            self.base_handle.set_joint_pos(i,0.0)
            time.sleep(0.05)
        self.base_handle.set_joint_pos('notch_9-notch_10', 0)
        time.sleep(0.5)
    
    def run_grasp_logic(self, jaw_angle):
        for i in range(len(self.actuators)):
            if jaw_angle <= 0.2:
                if self.sensor is not None:
                    if self.sensor.is_triggered(i):
                        sensed_obj = self.sensor.get_sensed_object(i)
                        for s in self.graspable_objs_prefix:
                            if s in sensed_obj:
                                print(sensed_obj)
                                if not self.grasped[i]:
                                    qualified_name = sensed_obj
                                    self.actuators[i].actuate(qualified_name)
                                    self.grasped[i] = True
                                    print('Grasping Sensed Object Names', sensed_obj)
            else:
                if self.actuators[i] is not None:
                    self.actuators[i].deactuate()
                    if self.grasped[i] is True:
                        print('Releasing Grasped Object')
                    self.grasped[i] = False
                    # print('Releasing Actuator ', i)
        
    def update_jaw_links(self, jaw_pos):
        self.base_handle.set_joint_pos('gripper_holder-gripper_1', -jaw_pos)
        self.base_handle.set_joint_pos('gripper_holder-gripper_2', jaw_pos)

    def update_jaw(self, jaw_pos):
        self.jaw_pos = jaw_pos
        self.update_jaw_links(self.jaw_pos)
        self.run_grasp_logic(jaw_pos)

    def servo_jp(self, gm_joint_state):

        joint_positions = gm_joint_state.position

        if len(joint_positions) != self._num_joints:
            rospy.logerr(f"joint positions size must be {self._num_joints} but are {len(joint_positions)}")
            return

        #pdb.set_trace()
        for i in range (0,14):
            self.base_handle.set_joint_pos(index_to_correct_name[i], joint_positions[i])
        
        self.update_jaw(joint_positions[14])
        self.base_handle.set_joint_pos('notch_10-gripper_holder', joint_positions[13])
        self.base_handle.set_joint_pos('notch_9-notch_10', joint_positions[12])

    def get_jaw_pos(self):
        j1 = self.base_handle.get_joint_pos('gripper_holder-gripper_1')
        j2 = self.base_handle.get_joint_pos('gripper_holder-gripper_2')
        return j1

    def measured_js(self):

        q = []
        for i in range (0,14):
             q.append(self.base_handle.get_joint_pos(index_to_correct_name[i]))
        q.append(self.get_jaw_pos())

        q = self._joint_error_model.remove_from_joints(q, [1, 1, 0, 0])
        gm_js = JointState()
        gm_js.position = q
        self.measured_js_pub.publish(gm_js)

        ee_pose = Pose()
        ee_pose.position = self.ee_body.get_pos()
        self.measured_ee_pose_pub.publish(ee_pose)
        

    def jaw_measured_js(self):
        gm_js = JointState()
        jaw_pos = self.get_jaw_pos()
        gm_js.position = [jaw_pos]
        self.jaw_measured_js_pub.publish(gm_js)

    def stop(self):
        self.pub_thread.stop()

    def __del__(self):
        self._client.clean_up()

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)

    # Parse arguments
    '''parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--namespace', type=str, required=False,
                        default = "PSM2",
                        help = 'the package where you store the config yaml, default is "dvrk_planning_ros"')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name'''


    rospy.init_node('ambf_crtk_translator')
    try:
        crtk_translator_1 = PSMTranslator(crtk_namespace = '/PSM1')
        crtk_translator_2 = PSMTranslator(crtk_namespace = '/PSM2')                                     
        rospy.spin()
        crtk_translator_1.stop()
        crtk_translator_2.stop()
    except rospy.ROSInterruptException:
        pass



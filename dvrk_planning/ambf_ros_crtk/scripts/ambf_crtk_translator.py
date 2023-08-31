# Import the Client from ambf_client package
from ambf_client import Client
import argparse
import threading
import rospy
import sys
import time
import numpy as np
import random

from  sensor_msgs.msg import JointState

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
    def __init__(self, cb1, cb2):
        super(PublishThread, self).__init__()
        self.cb1 = cb1
        self.cb2 = cb2
        self.done = False

        self.start()
    def stop(self):
        self.done = True
        self.join()

    def run(self):
        while not self.done:
            self.cb1()
            self.cb2()
            time.sleep(0.01)

index_to_correct_name = [
    'baselink-yawlink', 
    'yawlink-pitchbacklink',
    'pitchendlink-maininsertionlink',
    'maininsertionlink-toolrolllink',
    'toolrolllink-toolpitchlink'
    ]

class PSMTranslator:
    def __init__(self, crtk_namespace = '/PSM2', initial_jps = []):
        self._client = Client()
        self._client.connect()
        self.base_handle = self._client.get_obj_handle('psm/baselink')
        time.sleep(0.2)
        self.measured_js_pub = rospy.Publisher(crtk_namespace + "/measured_js", JointState, queue_size = 10)
        self.jaw_measured_js_pub = rospy.Publisher(crtk_namespace + "/jaw/measured_js", JointState, queue_size = 10)

        self._num_joints = 6
        # self._joint_error_model = JointErrorsModel(self._num_joints, errors_distribution_deg= [-5., -2. , 2., 5.])
        self._joint_error_model = JointErrorsModel(self._num_joints, errors_distribution_deg= [0])
        self.joint_names = self.base_handle.get_joint_names() # Might be useful later

        self.yaw_pos = 0.0
        self.jaw_pos = 0.0

        print(self.joint_names)
        gm_js = JointState()
        gm_js.position = [0.0, 0.0, 0.1, 0.0, 0.0, self.yaw_pos]
        self.servo_jp(gm_js)
        gm_js.position = [self.jaw_pos]
        self.jaw_servo_jp(gm_js)

        self.pub_thread = PublishThread(self.measured_js, self.jaw_measured_js)

        self.servo_jp_sub = rospy.Subscriber(crtk_namespace + "/servo_jp", JointState, self.servo_jp)
        self.jaw_servo_jp_sub = rospy.Subscriber(crtk_namespace + "/jaw/servo_jp", JointState, self.jaw_servo_jp)

    def update_jaw_links(self, yaw_pos, jaw_pos):
        self.base_handle.set_joint_pos('toolpitchlink-toolgripper1link', yaw_pos - jaw_pos)
        self.base_handle.set_joint_pos('toolpitchlink-toolgripper2link', -yaw_pos - jaw_pos)

    def update_tool_yaw(self, yaw_pos):
        self.yaw_pos = yaw_pos
        self.update_jaw_links(self.yaw_pos, self.jaw_pos)

    def update_jaw(self, jaw_pos):
        self.jaw_pos = jaw_pos
        self.update_jaw_links(self.yaw_pos, self.jaw_pos)

    def servo_jp(self, gm_joint_state):

        joint_positions = gm_joint_state.position
        if len(joint_positions) != self._num_joints:
            rospy.logerr("joint positions size must be 6")
            return

        self.base_handle.set_joint_pos(index_to_correct_name[0], joint_positions[0])
        self.base_handle.set_joint_pos(index_to_correct_name[1], joint_positions[1])
        self.base_handle.set_joint_pos(index_to_correct_name[2], joint_positions[2])
        self.base_handle.set_joint_pos(index_to_correct_name[3], joint_positions[3])
        self.base_handle.set_joint_pos(index_to_correct_name[4], joint_positions[4])
        self.update_tool_yaw(joint_positions[5])

    def jaw_servo_jp(self, gm_joint_state):
        joint_positions = gm_joint_state.position
        if len(joint_positions) != 1:
            rospy.logerr("joint positions size must be 1")
            return
        self.update_jaw(joint_positions[0])

    def get_jaw_pos(self):
        j1 = self.base_handle.get_joint_pos('toolpitchlink-toolgripper1link')
        j2 = self.base_handle.get_joint_pos('toolpitchlink-toolgripper2link')
        return (-j1 - j2)/2, j1
    
    def get_yaw_pos(self):
        current_jaw_pose, j1 = self.get_jaw_pos()
        j1 + current_jaw_pose
        return j1 + current_jaw_pose

    def measured_js(self):
        j0 = self.base_handle.get_joint_pos(index_to_correct_name[0])
        j1 = self.base_handle.get_joint_pos(index_to_correct_name[1])
        j2 = self.base_handle.get_joint_pos(index_to_correct_name[2])
        j3 = self.base_handle.get_joint_pos(index_to_correct_name[3])
        j4 = self.base_handle.get_joint_pos(index_to_correct_name[4])
        j5 = self.get_yaw_pos()

        q = [j0, j1, j2, j3, j4, j5]
        q = self._joint_error_model.remove_from_joints(q, [1, 1, 0, 0, 0, 0])
        gm_js = JointState()
        gm_js.position = q
        self.measured_js_pub.publish(gm_js)

    def jaw_measured_js(self):
        gm_js = JointState()
        jaw_pos, _ = self.get_jaw_pos()
        gm_js.position = [jaw_pos]
        self.jaw_measured_js_pub.publish(gm_js)

    def stop(self):
        self.pub_thread.stop()

    def __del__(self):
        self._client.clean_up()

if __name__ == '__main__':
    argv = rospy.myargv(argv=sys.argv)

    # Parse arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-n', '--namespace', type=str, required=False,
                        default = "PSM2",
                        help = 'the package where you store the config yaml, default is "dvrk_planning_ros"')
    args = parser.parse_args(argv[1:]) # skip argv[0], script name


    rospy.init_node('ambf_crtk_translator')
    try:
        crtk_translator = PSMTranslator(crtk_namespace = args.namespace)
        rospy.spin()
        crtk_translator.stop()
    except rospy.ROSInterruptException:
        pass

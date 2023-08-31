#!/usr/bin/env python3

import rospy
from dvrk_planning_ros.utils import *

from dvrk_planning.kinematics.psm import PsmKinematicsSolver, LND400006
from dvrk_planning_msgs.srv import ComputeIK, ComputeIKResponse

class DvrkPlanningNode:
    def __init__(self):
        self.kin_solver = PsmKinematicsSolver(LND400006())
        self.compute_ik_srv = rospy.Service("psm/compute_ik", ComputeIK, self.psm_compute_ik)

    def psm_compute_ik(self, req):
        joint_states = self.kin_solver.compute_ik(gm_tf_to_numpy_mat(req.tf_stamped.transform))
        res = ComputeIKResponse()
        res.joint_state.name = self.kin_solver.kinematics_data.joint_names
        res.joint_state.position = joint_states
        return res

if __name__ == '__main__':
    rospy.init_node('dvrk_planning')
    try:
        dvrk_planning_node = DvrkPlanningNode()
        rospy.spin()
    except rospy.ROSInterruptException: pass

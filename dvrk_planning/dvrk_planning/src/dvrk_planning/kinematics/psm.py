import math
import numpy as np

from dvrk_planning.utilities import *
from dvrk_planning.kinematics.dh import *
from dvrk_planning.kinematics.kinematics_solver import KinematicsSolver
from PyKDL import Vector, Rotation, Frame

# For the frame definitions, launch https://github.com/WPI-AIM/dvrk_env/tree/feature/standardize_model_kinematics
# `roslaunch dvrk_description psm_rviz.launch`
# This model in dvrk_env uses the DH axis defined in dvrk manual. NOTE, dvrk manual
# dh figures are wrong, so dvrk_env frames correspond to https://arxiv.org/pdf/1902.10875.pdf figure 4a.
# The kinematics are wrt to the base_link frame (Remote center of motion)

# THIS IS THE FK FOR THE PSM MOUNTED WITH THE LARGE NEEDLE DRIVER TOOL. THIS IS THE
# SAME KINEMATIC CONFIGURATION FOUND IN THE DVRK MANUAL. NOTE, JUST LIKE A FAULT IN THE
# MTM's DH PARAMETERS IN THE MANUAL, THERE IS A FAULT IN THE PSM's DH AS WELL. BASED ON
# THE FRAME ATTACHMENT IN THE DVRK MANUAL THE CORRECT DH CAN FOUND IN THIS FILE

# ALSO, NOTICE THAT AT HOME CONFIGURATION THE TIP OF THE PSM HAS THE FOLLOWING
# ROTATION OFFSET W.R.T THE BASE. THIS IS IMPORTANT FOR IK PURPOSES.
# R_7_0 = [ 0,  1,  0 ]
#       = [ 1,  0,  0 ]
#       = [ 0,  0, -1 ]
# Basically, x_7 is along y_0, y_7 is along x_0 and z_7 is along -z_0.

# You need to provide a list of joint positions. If the list is less that the number of joint
# i.e. the robot has 6 joints, but only provide 3 joints. The FK till the 3+1 link will be provided

class SphericalWristToolParams():
    def __init__(self, L_rcc, L_tool, L_pitch2yaw, L_yaw2ctrlpnt, scale = 1.0, negate_joint_list = [1, 1, 1, 1, 1, 1]):
        self.L_rcc = L_rcc * scale
        self.L_tool = L_tool * scale
        self.L_pitch2yaw = L_pitch2yaw * scale
        self.L_yaw2ctrlpnt = L_yaw2ctrlpnt * scale
        # Delta between tool tip and the Remote Center of Motion
        # self.L_tool2rcm_offset = 0.0229
        self.L_tool2rcm_offset = self.L_rcc - self.L_tool
        self.scale = scale
        self.negate_joint_list = negate_joint_list

class LND400006(SphericalWristToolParams):
    def __init__(self, scale = 1.0):
        super().__init__(
            L_rcc = 0.4318,  # From dVRK documentation
            L_tool = 0.4162,  # From dVRK documentation
            L_pitch2yaw = 0.0091,  # Fixed length from the palm joint to the pinch joint
            L_yaw2ctrlpnt = 0.0102,  # Fixed length from the pinch joint to the pinch tip
            scale = scale
        )

class CF470049(SphericalWristToolParams):
    def __init__(self, scale = 1.0):
        super().__init__(
            L_rcc = 0.4318,  # From dVRK documentation
            L_tool = 0.4162,  # From dVRK documentation
            L_pitch2yaw = 0.0091,  # Fixed length from the palm joint to the pinch joint
            L_yaw2ctrlpnt = 0.01977,  # From CAD measurement
            scale = scale
        )

class RTS470007(SphericalWristToolParams):
    def __init__(self, scale = 1.0):
        super().__init__(
            L_rcc = 0.4318,  # From dVRK documentation
            L_tool = 0.4162,  # From dVRK documentation
            L_pitch2yaw = 0.0091,  # Fixed length from the palm joint to the pinch joint
            L_yaw2ctrlpnt = 0.01041,  # From CAD measurement
            scale = scale
        )

class RTS420007(SphericalWristToolParams):
    def __init__(self, scale = 1.0):
        super().__init__(
            L_rcc = 0.4318,  # From dVRK documentation
            # L_tool = 0.4677,  # From dVRK documentation
            L_tool = 0.47,  # From dVRK documentation
            L_pitch2yaw = 0.0091,  # Fixed length from the palm joint to the pinch joint
            L_yaw2ctrlpnt = 0.01041,  # From CAD measurement
            scale = scale,
            negate_joint_list = [1, 1, 1, -1, 1, 1]
        )

class CustomSphericalWristFromYaml(SphericalWristToolParams):
    def __init__(self, yaml):
        scale = 1.0
        if "scale" in yaml:
            scale = yaml["scale"]
        super().__init__(
            L_rcc = yaml["L_rcc"],  # From dVRK documentation
            L_tool = yaml["L_tool"],  # From dVRK documentation
            L_pitch2yaw = yaml["L_pitch2yaw"],  # Fixed length from the palm joint to the pinch joint
            L_yaw2ctrlpnt = yaml["L_yaw2ctrlpnt"],  # Fixed length from the pinch joint to the pinch tip
            scale = scale
        )

class PsmKinematicsData:
    def __init__(self, s_wrist_tool_params: SphericalWristToolParams):
        self.swt_params = s_wrist_tool_params
        # From the urdf, and the jhu dvrk_robot
        self.joint_names = ["outer_yaw", "outer_pitch", "outer_insertion", \
            "outer_roll", "outer_wrist_pitch", "outer_wrist_yaw"] #Should have jaw, but doesnt work with current controller

        # PSM DH Params, All link axes and names now correspond to dvrk_env psm.urdf.xacro and tool lnd400006.
        # [previous_link, link_name, [alpha | a | d | theta | type | convention]]
        self._kinematics = [["", "yaw_link", DH(PI_2, 0, 0, PI_2, 
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["yaw_link", "pitch_link", DH(-PI_2, 0, 0, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["pitch_link", "main_insertion_link", DH(PI_2, 0, -self.swt_params.L_rcc, -PI_2,
                              JointType.PRISMATIC, Convention.MODIFIED)],
                           ["main_insertion_link", "tool_roll_link", DH(0, 0, self.swt_params.L_tool, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["tool_roll_link", "tool_pitch_link", DH(PI_2, 0, 0, PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["tool_pitch_link", "tool_yaw_link", DH(-PI_2, self.swt_params.L_pitch2yaw, 0, PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["tool_yaw_link", "tool_tip", DH(PI_2, 0, self.swt_params.L_yaw2ctrlpnt, -PI_2,
                              JointType.REVOLUTE, Convention.MODIFIED)],
                           ["tool_yaw_link", "tool_gripper1_link", DH(0, 0, 0, 0, JointType.REVOLUTE, Convention.MODIFIED, axis_multiplier=-1)],
                           ["tool_yaw_link", "tool_gripper2_link", DH(0, 0, 0, 0, JointType.REVOLUTE, Convention.MODIFIED)]]

        # Parse _kinematics
        self.link_name_to_dh = {}
        self.link_to_previous_link = {}
        for kin in self._kinematics:
            if (kin[1] in self.link_name_to_dh):
                raise Exception ("link defined twice in kinematics")
            self.link_to_previous_link[kin[1]] = kin[0]
            self.link_name_to_dh[kin[1]] = kin[2]
        self.num_links = len(self.link_name_to_dh)

        # Standard kinematic chain
        default_target_link = "tool_tip"
        # default_base_link = "yaw_link"
        prev_link = self.link_to_previous_link[default_target_link]
        self.default_chain = [default_target_link]
        while prev_link != "":
            self.default_chain.append(prev_link)
            prev_link = self.link_to_previous_link[prev_link]
            
        self.default_chain.reverse()
        self.default_chain_link_num = len(self.default_chain)

    def get_dh(self, link_name):
            return self.link_name_to_dh[link_name]

class Chain():
    def __init__(self, link_names):
        self.link_names = link_names
    def get_num_of_active_joints(self):
        return len(self.link_names) - 1

class PsmKinematicsSolver(KinematicsSolver):
    def __init__(self, spherical_wrist_tool_params: SphericalWristToolParams):
        self.kinematics_data = PsmKinematicsData(spherical_wrist_tool_params)
        self.negate_joint_list = spherical_wrist_tool_params.negate_joint_list

    # 7 comes from default_chain_link_num
    def compute_fk(self, joint_positions, up_to_link_num = 7):
        if up_to_link_num > self.kinematics_data.default_chain_link_num:
            raise "ERROR! COMPUTE FK UP_TO_LINK GREATER THAN DEFAULT CHAIN LINK NUM"
        j = np.zeros(self.kinematics_data.default_chain_link_num)
        for i in range(len(joint_positions)):
            j[i] = joint_positions[i]

        T_N_0 = np.identity(4)
        negate_joint_list = self.negate_joint_list + [1.0]
        for i in range(up_to_link_num):
            link_dh = self.kinematics_data.get_dh(self.kinematics_data.default_chain[i])
            T_N_0 = T_N_0 * link_dh.get_trans(negate_joint_list[i] * j[i])
        return T_N_0

    def get_chain(self, reference_link, target_link):
        # Construct the chain
        prev_link = self.kinematics_data.link_to_previous_link[target_link]
        link_names = [target_link]
        while prev_link != reference_link:
            link_names.append(prev_link)
            prev_link = self.kinematics_data.link_to_previous_link[prev_link]
        link_names.append(prev_link)
        link_names.reverse()
        return Chain(link_names)

    def compute_fk_relative(self, joint_positions, chain:Chain):
        # Error Checking
        if len(joint_positions) != chain.get_num_of_active_joints():
            raise Exception("length of joint_positions [{}] \
                must be the same as length of chain [{}]".format(len(joint_positions), len(chain)))

        # Calculate the TF
        T_R_T = np.identity(4)
        for i in range(len(joint_positions)):
            link_dh = self.kinematics_data.get_dh(chain.link_names[i+1])
            T_R_T = T_R_T * link_dh.get_trans(joint_positions[i])
        return T_R_T

    def enforce_limits(self, j_raw):
        # Min to Max Limits
        j1_lims = [np.deg2rad(-91.96), np.deg2rad(91.96)]
        j2_lims = [np.deg2rad(-60), np.deg2rad(60)]
        j3_lims = [0.0, 2.40]
        j4_lims = [np.deg2rad(-175), np.deg2rad(175)]
        j5_lims = [np.deg2rad(-90), np.deg2rad(90)]
        j6_lims = [np.deg2rad(-85), np.deg2rad(85)]

        j_limited = [0.0]*6
        j_lims = [j1_lims, j2_lims, j3_lims, j4_lims, j5_lims, j6_lims]

        for idx in range(6):
            min_lim = j_lims[idx][0]
            max_lim = j_lims[idx][1]
            j_limited[idx] = max(min_lim, min(j_raw[idx], max_lim))

        return [j_limited[0], j_limited[1], j_limited[2], j_limited[3], j_limited[4], j_limited[5]]

    def compute_ik(self, T_7_0_mat):
        T_7_0 = convert_mat_to_frame(T_7_0_mat)

        swt_params = self.kinematics_data.swt_params
        # Pinch Joint
        T_PinchJoint_7 = Frame(Rotation.RPY(
            0, 0, 0), swt_params.L_yaw2ctrlpnt * Vector(0.0, 0.0, -1.0))
        # Pinch Joint in Origin
        T_PinchJoint_0 = T_7_0 * T_PinchJoint_7

        R_0_PinchJoint = T_PinchJoint_0.M.Inverse()
        P_PinchJoint_local = R_0_PinchJoint * T_PinchJoint_0.p
        # print("P_PinchJoint_local: ", round_vec(P_PinchJoint_local))
        # Now we can trim the value along the x axis to get a projection along the YZ plane as mentioned above
        N_PalmJoint_PinchJoint = -P_PinchJoint_local
        N_PalmJoint_PinchJoint[0] = 0
        N_PalmJoint_PinchJoint.Normalize()

        T_PalmJoint_PinchJoint = Frame(Rotation.RPY(
            0, 0, 0), N_PalmJoint_PinchJoint * swt_params.L_pitch2yaw)
        # Get the shaft tip or the Palm's Joint position
        T_PalmJoint_0 = T_7_0 * T_PinchJoint_7 * T_PalmJoint_PinchJoint

        # Calculate insertion_depth to check if the tool is past the RCM
        insertion_depth = T_PalmJoint_0.p.Norm()

        # Now having the end point of the shaft or the PalmJoint, we can calculate some
        # angles as follows
        xz_diagonal = math.sqrt(T_PalmJoint_0.p[0] ** 2 + T_PalmJoint_0.p[2] ** 2)

        j1 = math.atan2(T_PalmJoint_0.p[0], -T_PalmJoint_0.p[2])
        j2 = -math.atan2(T_PalmJoint_0.p[1], xz_diagonal)
        j3 = insertion_depth + swt_params.L_tool2rcm_offset

        # Calculate j4
        # This is an important case and has to be dealt carefully. Based on some inspection, we can find that
        # we need to construct a plane based on the vectors Rx_7_0 and D_PinchJoint_PalmJoint_0 since these are
        # the only two vectors that are orthogonal at all configurations of the EE.
        cross_palmlink_x7_0 = T_7_0.M.UnitX() * (T_PinchJoint_0.p - T_PalmJoint_0.p)

        # To get j4, compare the above vector with Y axes of T_3_0
        T_3_0 = convert_mat_to_frame(self.compute_fk([j1, j2, j3], 3))
        j4 = get_angle(cross_palmlink_x7_0, -T_3_0.M.UnitX(),
                    up_vector=-T_3_0.M.UnitZ())

        # Calculate j5
        # This should be simple, just compute the angle between Rz_4_0 and D_PinchJoint_PalmJoint_0
        link4_dh = self.kinematics_data.get_dh(self.kinematics_data.default_chain[3])
        T_4_3 = convert_mat_to_frame(link4_dh.get_trans(j4))
        T_4_0 = T_3_0 * T_4_3

        j5 = get_angle(T_PinchJoint_0.p - T_PalmJoint_0.p,
                    T_4_0.M.UnitZ(), up_vector=T_4_0.M.UnitY())

        # Calculate j6
        # This too should be simple, compute the angle between the Rz_7_0 and Rx_5_0.
        link5_dh = self.kinematics_data.get_dh(self.kinematics_data.default_chain[4])
        T_5_4 = convert_mat_to_frame(link5_dh.get_trans(j5))
        T_5_0 = T_4_0 * T_5_4

        j6 = get_angle(T_7_0.M.UnitZ(), T_5_0.M.UnitX(),
                    up_vector=-T_5_0.M.UnitY())
        
        joint_list = [j1, j2, j3, j4, j5, j6]
        for i in range(len(joint_list)):
            joint_list[i] *= self.negate_joint_list[i]

        return joint_list

    def get_active_joint_names(self):
        return self.kinematics_data.joint_names

    def get_link_names(self):
        return list(self.kinematics_data.link_name_to_dh.keys())

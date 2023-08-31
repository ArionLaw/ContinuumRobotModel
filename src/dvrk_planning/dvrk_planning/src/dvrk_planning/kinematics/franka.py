import math
import numpy as np

from dvrk_planning.kinematics.kinematics_solver import KinematicsSolver
import roboticstoolbox as rtb
from spatialmath import SE3


class FrankaKinematicsSolver(KinematicsSolver):
    def __init__(self, spherical_wrist_tool_params: SphericalWristToolParams):
        self.panda = rtb.models.DH.Panda()


    def compute_fk(self, joint_positions):
        end_effector_config = self.panda.fkine(joint_angles)
        return tool_config_from_end_effector(end_effector_config)
    
    
    def compute_ik(self, config_matrix, q0 = None):
        desired_end_effector_config = end_effector_config_from_tool(config_matrix)
        return self.panda.ikine_LM(desired_end_effector_config, q0=q0)
    
    
    def tool_config_from_end_effector(self, end_effector_config) -> SE3:
        return end_effector_config * SE3.Tz(0.4) # Tz goes in direction of end effector


    def end_effector_config_from_tool(self, tool_config) -> SE3:
        return tool_config * SE3.Tz(-0.4)
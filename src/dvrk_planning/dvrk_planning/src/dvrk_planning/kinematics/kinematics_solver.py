from abc import ABC, abstractmethod

class KinematicsSolver(ABC):
    @abstractmethod
    def compute_fk(self, joint_positions):
        pass
    @abstractmethod
    def compute_ik(self, T_tip_0_mat):
        pass

    # These two below belong in the robot description class later
    @abstractmethod
    def get_active_joint_names(self):
        pass
    @abstractmethod
    def get_link_names(self):
        pass

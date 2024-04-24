import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

class SUJKinematics:
    def __init__(self, suj_psm1, suj_psm2, suj_ecm):
        self.rcc = 0.4318  # radius of curvature for the cannula?

        # DH tables for each SUJ arm
        self.suj_psm_1_dh = self.create_dh_table(suj_psm1)
        self.suj_psm_2_dh = self.create_dh_table(suj_psm2)
        self.suj_ecm_dh = self.create_ecm_dh_table(suj_ecm)

        # Base transform from the world to frame 3 (using additional parameters for each PSM)
        self.psm_1_base_tf = self.compute_psm_base_transform(suj_psm1[6], suj_psm1[7])
        self.psm_2_base_tf = self.compute_psm_base_transform(suj_psm2[6], suj_psm2[7])

        self.psm_1_w_B_tf = self.compute_transform(self.suj_psm_1_dh)@self.psm_1_base_tf
        self.psm_2_w_B_tf = self.compute_transform(self.suj_psm_2_dh)@self.psm_2_base_tf

        self.ecm_w_B_tf = self.compute_transform(self.suj_ecm_dh)

    def create_dh_table(self, joints):
        return np.array([
            [0.0896, 0, joints[0], 0],
            [0, 0, 0.4166, joints[1]],
            [0.4318, 0, 0.1429, joints[2]],
            [0.4318, 0, -0.1302, joints[3] + np.pi / 2],
            [0, np.pi / 2, 0.4089, joints[4]],
            [0, -np.pi / 2, -0.1029, joints[5]],
            [0, np.pi / 2, 0, joints[6] + np.pi / 2],
            [0, np.pi / 2, 0, np.pi / 2],
            [-0.0296, 0, 0, joints[7] - 0.2908 - np.pi / 2],
            [0.0664, 0, 0, joints[7] - 0.2908 - np.pi / 2],
            [-0.0296, 0, 0, joints[7] - 0.3675 - np.pi / 2],
            [0.150, 0, 0, -joints[7] + 0.2908 + np.pi / 2],
            [0.1842, 0, 0, -joints[7] + 0.3675 + np.pi / 2],
            [0.516, 0, 0, joints[7]],
            [0.043, -np.pi / 2, 0 - 0.2881, np.pi / 2]
        ])

    def create_ecm_dh_table(self, suj_ecm):
        return np.array([[0.0896, 0, suj_ecm[0],0],
                                     [0,0,0.4166,suj_ecm[1]],
                                     [0.4318,0,0.1429,suj_ecm[2]],
                                     [0.4318, 0, -0.3459, suj_ecm[3]+ np.pi/2],
                                     [0, -0.7853, 0, np.pi/2],
                                     [-0.0667, 0, 0,0],
                                     [0,0,0.1029, np.pi/2],
                                     [0, np.pi/2, 0.2722, suj_ecm[4]+ np.pi/2], #q1 in table 5
                                     [-0.0098, -np.pi/2, 0, np.pi/2],
                                     [0, -np.pi/2, 0, suj_ecm[5] - 0.3448], #q2 table 5
                                     [0.03657, 0, 0, suj_ecm[5] - 0.3448 - np.pi/2],
                                     [0, -np.pi/2, 0, suj_ecm[5] - 0.3229],
                                     [0.3047, 0, 0, -suj_ecm[5] + 0.3448 + np.pi/2],
                                     [0.3419, 0, 0, -suj_ecm[5] + 0.3229 + np.pi/2],
                                     [0.3404, 0, 0, suj_ecm[5]],
                                     [0.103, -np.pi/2, suj_ecm[6] - 0.0953, np.pi],
                                     [0, 0, 0.3829, suj_ecm[7]]]) 

    
    def compute_transform(self,dh_params):
        transform = np.eye(4)
        for params in dh_params:
            transform = np.dot(transform, self.compute_dh_matrix(*params))
        return transform

    
    def compute_psm_base_transform(self, q1, q2):
        dh_params = np.array([
            [0, np.pi / 2, 0, np.pi / 2 + q1],
            [0, -np.pi / 2, 0, q2 - np.pi / 2],
            [0, np.pi / 2, 0, 0]
        ])
        transform = self.compute_transform(dh_params)

        # Compute the inverse of the transform manually
        R = transform[0:3, 0:3]  
        P = transform[0:3, 3]    
        R_transpose = np.transpose(R)
        inverse_transform = np.eye(4)
        inverse_transform[0:3, 0:3] = R_transpose
        inverse_transform[0:3, 3] = -np.dot(R_transpose, P)       
        return inverse_transform

    @staticmethod
    def compute_dh_matrix(a, alpha, d, theta):
        return np.array([
            [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
            [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
            [0, np.sin(alpha), np.cos(alpha), d],
            [0, 0, 0, 1]
        ])

    def export_transforms_to_yaml(self, filename='src/dvrk_ctr_teleop/config/transforms.yaml'):
        # Convert transforms to quaternions and positions, then write to YAML
        data = {
            'psm_1': self.extract_quaternion_and_position(self.psm_1_w_B_tf),
            'psm_2': self.extract_quaternion_and_position(self.psm_2_w_B_tf),
            'ecm': self.extract_quaternion_and_position(self.ecm_w_B_tf)
        }
        with open(filename, 'w') as file:
            yaml.dump(data, file)

    def extract_quaternion_and_position(self, transform):
        rotation = R.from_matrix(transform[:3, :3])
        quaternion = rotation.as_quat()
        position = transform[:3, 3]
        return {'quaternion': quaternion.tolist(), 'position': position.tolist()}


if __name__ == "__main__":
    suj_psm1 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    suj_psm2 = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    suj_ecm = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8]
    kinematics = SUJKinematics(suj_psm1, suj_psm2, suj_ecm)
    kinematics.export_transforms_to_yaml()


import rospy
import PyKDL
import numpy as np
import trimesh
from pysdf import SDF
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import WrenchStamped
from ambf_msgs.msg import RigidBodyState
from std_msgs.msg import Header
from std_msgs.msg import Float32
from collections import deque
from cvxopt import matrix, solvers
from jacobian import compute_jacobian

def vfi_optimization_step(J_t, q_current, gradient, distance, speed, eta_d, d_safe, alpha, k, q_min, q_max):
    normal = -(gradient / np.linalg.norm(gradient))
    J_d = np.dot(normal,J_t)
    d_error = distance - d_safe
    d_dot_safe = k*np.exp(alpha*()





def compute_sdf_gradient(sdf, point, epsilon=1e-5):
    """
    Computes the gradient of the SDF at a given point using finite differences.

    Args:
        sdf (SDF): The SDF object.
        point (np.ndarray): The 3D point at which to compute the gradient.
        epsilon (float): A small value for finite differences.

    Returns:
        np.ndarray: The gradient vector at the point.
    """
    gradient = np.zeros(3)
    for i in range(3):
        offset = np.zeros(3)
        offset[i] = epsilon
        sdf_plus = sdf(point + offset)
        sdf_minus = sdf(point - offset)
        gradient[i] = (sdf_plus[0] - sdf_minus[0]) / (2 * epsilon)
    return gradient

def transform_mesh(position, quaternion):
    rotation = R.from_quat(quaternion).as_matrix()
    transformation_matrix = np.eye(4)
    transformation_matrix[:3, :3] = rotation
    transformation_matrix[:3, 3] = position
    return transformation_matrix

def generate_haptic_force(gradient, distance, velocity, alpha=3, kappa = 25, F_max=4.0, d_min = 0.10):

    if (np.linalg.norm(velocity) <= 1e-5) or (distance > d_min):
        return np.zeros(3), 0.0
    
    norm_velocity = velocity / np.linalg.norm(velocity)
    normal = -(gradient / np.linalg.norm(gradient))
    dot_product = np.dot(norm_velocity, normal)
    beta = alpha * np.sqrt((1 - dot_product)*(np.tanh(kappa*(d_min-distance))) /2)
    force_magnitude = min(beta * np.linalg.norm(velocity), F_max)

    if dot_product < 0:
        force_direction = normal
    else:
        theta = (1 - dot_product) * np.pi / 2
        n = np.cross(norm_velocity, normal)
        norm_n = np.linalg.norm(n)
        n = n / norm_n
        # Rotate the velocity vector around the axis n by angle theta
        force_direction = rotate_vector(norm_velocity, theta, n)
        #print(f"distance:{distance}, force_dir: {force_direction}, beta: {beta}")
    
    if force_magnitude>F_max:
        force_magnitude = F_max 

    
    return force_direction, force_magnitude

def rotate_vector(v, theta, n):
    """
    Rotate vector v around axis n by angle theta (in radians) using Rodrigues' rotation formula.
    """
    v_rot = (v * np.cos(theta) + 
             np.cross(n, v) * np.sin(theta) + 
             n * np.dot(n, v) * (1 - np.cos(theta)))
    return v_rot

class DoubleExponentialMovingAverage:
    def __init__(self, alpha):
        self.alpha = alpha
        self.ema1 = None
        self.ema2 = None
        self.dema = None

    def update(self, value):
        if self.ema1 is None:
            self.ema1 = value
            self.ema2 = value
            self.dema = value
        else:
            self.ema1 = self.alpha * value + (1 - self.alpha) * self.ema1
            self.ema2 = self.alpha * self.ema1 + (1 - self.alpha) * self.ema2
            self.dema = 2 * self.ema1 - self.ema2

        return self.dema

class PSMForceHandler:
    def __init__(self, gripper_1_mesh, gripper_2_mesh, psm_id, sdf, alpha = 0.05):
        self.psm_id = psm_id
        self.sdf = sdf
        self.pose_gripper_1 = None
        self.pose_gripper_2 = None
        self.velocity = np.zeros(3)
        self.prev_position = None

        self.force_pub = rospy.Publisher(f'/ambf/env/{psm_id}/servo_cp', WrenchStamped, queue_size=10)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_1/State', RigidBodyState, self.pose_callback_gripper_1)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_2/State', RigidBodyState, self.pose_callback_gripper_2)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_holder/State', RigidBodyState, self.velocity_callback)
        self.force_magnitude_pub = rospy.Publisher(f'/ambf/env/{psm_id}/force_magnitude', Float32, queue_size=10)
        self.distance_pub = rospy.Publisher(f'/ambf/env/{psm_id}/distance', Float32, queue_size=10)

        self.last_time = rospy.Time.now()
        self.gripper_1 = gripper_1_mesh
        self.gripper_2 = gripper_2_mesh
        self.dema_calculator = DoubleExponentialMovingAverage(alpha)
        self.dema_calculator_speed= DoubleExponentialMovingAverage(0.3)

        self.J = np.zeros([3,6], dtype=float)
        self.l_wrist = 0.0015
        self.l_tool = 0.400
        self.l_rcc = 0.4318


    def pose_callback_gripper_1(self, msg):
        self.pose_gripper_1 = msg.pose
    
    def pose_callback_gripper_2(self, msg):
        self.pose_gripper_2 = msg.pose
    
    def velocity_callback(self,msg):
        current_time = msg.header.stamp

        position = np.array([msg.pose.position.x, 
                             msg.pose.position.y, 
                             msg.pose.position.z])

        if self.prev_position is not None:
            dt = (current_time - self.last_time).to_sec()
            if dt > 0:
                self.velocity = (position - self.prev_position) / dt
        
        self.prev_position = position
        self.last_time = current_time

    def compute_and_publish_force(self):
        if self.pose_gripper_1 is None or self.pose_gripper_2 is None:
            return

        position_gripper_1 = np.array([self.pose_gripper_1.position.x, 
                                       self.pose_gripper_1.position.y, 
                                       self.pose_gripper_1.position.z])
        
        quaternion_gripper_1 = np.array([self.pose_gripper_1.orientation.x, 
                                         self.pose_gripper_1.orientation.y, 
                                         self.pose_gripper_1.orientation.z, 
                                         self.pose_gripper_1.orientation.w])

        gripper_1_transformation_matrix = transform_mesh(position_gripper_1, quaternion_gripper_1)
        gripper_1_mesh = self.gripper_1.copy()
        gripper_1_mesh.apply_transform(gripper_1_transformation_matrix)

        position_gripper_2 = np.array([self.pose_gripper_2.position.x, 
                                       self.pose_gripper_2.position.y, 
                                       self.pose_gripper_2.position.z])
        
        quaternion_gripper_2 = np.array([self.pose_gripper_2.orientation.x, 
                                         self.pose_gripper_2.orientation.y, 
                                         self.pose_gripper_2.orientation.z, 
                                         self.pose_gripper_2.orientation.w])

        gripper_2_mesh = self.gripper_2.copy()
        gripper_2_transformation_matrix = transform_mesh(position_gripper_2, quaternion_gripper_2)
        gripper_2_mesh.apply_transform(gripper_2_transformation_matrix)

        gripper_vertices = np.vstack((gripper_1_mesh.vertices, gripper_2_mesh.vertices))
        distances = abs(self.sdf(gripper_vertices))
        min_distance = np.min(distances)
        min_index = np.argmin(distances)
        min_distance_vertex = gripper_vertices[min_index]

        gradient = compute_sdf_gradient(self.sdf, min_distance_vertex)
        force_direction, force_magnitude = generate_haptic_force(gradient, min_distance, self.velocity)
        smoothed_speed = self.dema_calculator_speed(np.linalg.norm(self.velocity))



        smoothed_force_magnitude = self.dema_calculator.update(force_magnitude)
        #smoothed_force_magnitude = force_magnitude
        force = smoothed_force_magnitude * force_direction

        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = rospy.Time.now()
        wrench_msg.wrench.force.x = force[0]
        wrench_msg.wrench.force.y = force[1]
        wrench_msg.wrench.force.z = force[2]

        self.force_pub.publish(wrench_msg)

        # Publish the smoothed force magnitude
        force_magnitude_msg = Float32()
        force_magnitude_msg.data = smoothed_force_magnitude
        self.force_magnitude_pub.publish(force_magnitude_msg)

        distance_msg = Float32()
        distance_msg.data = min_distance
        self.distance_pub.publish(distance_msg)

def main():

    rospy.init_node('haptic_force_generator', anonymous=True)

    fetal_mesh = trimesh.load('src/dvrk_ctr_teleop/data/30Phantom.STL')
    gripper_1 = trimesh.load('src/dvrk_ctr_teleop/data/gripper_1.STL')
    gripper_2 = trimesh.load('src/dvrk_ctr_teleop/data/gripper_2.STL')

    position = np.array([-0.10757, 0.36021, 1.10982])
    quaternion = np.array([-0.7700439296218831,
                     0.0, 
                     0.0,
                     0.6379908670604059])

    transformation_matrix = transform_mesh(position, quaternion)
    fetal_mesh.apply_transform(transformation_matrix)
    sdf = SDF(fetal_mesh.vertices, fetal_mesh.faces)

    # Create PSM handlers
    psm1_handler = PSMForceHandler(gripper_1, gripper_2, 'psm1', sdf)
    psm2_handler = PSMForceHandler(gripper_1, gripper_2, 'psm2', sdf)

    rate = rospy.Rate(100) 
    while not rospy.is_shutdown():
        psm1_handler.compute_and_publish_force()
        psm2_handler.compute_and_publish_force()
        rate.sleep()

if __name__ == '__main__':
    main()
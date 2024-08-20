import rospy
import PyKDL
import numpy as np
import trimesh
from pysdf import SDF
from scipy.spatial.transform import Rotation as R
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Vector3
from ambf_msgs.msg import RigidBodyState
from std_msgs.msg import Header
from std_msgs.msg import Float32
from collections import deque
from cvxopt import matrix, solvers

# class VFI_Solver:
#     def __init__(self, eta_d, alpha, k, q_min, q_max):
#         self.alpha = alpha
#         self.eta_d = eta_d
#         self.d_safe_previous = 0.0
#         self.d_previous = 0.0
#         self.previous_time = 0.0
#         self.q_min = q_min
#         self.q_max = q_max
#         self.dma_d_dot = DoubleExponentialMovingAverage(0.5)
#         self.dma_d_dot_safe = DoubleExponentialMovingAverage(0.5)


#     def vfi_optimization_step(self, J_t, q_dot, gradient, distance, speed):

#         current_time = rospy.get_time()
#         dt = current_time - self.previous_time
#         normal = -(gradient / np.linalg.norm(gradient))
#         J_d = np.dot(normal,J_t)
#         d_safe = np.exp(self.alpha*speed)
#         d_dot_safe = (d_safe-self.d_safe_previous)/dt
#         d_error = distance - d_safe
#         d_dot = (distance - self.d_previous)/dt

#         smoothed_d_dot = self.dma_d_dot.update(d_dot)
#         smoothed_d_dot_safe = self.dma_d_dot_safe.update(d_dot_safe)

#         zeta = d_dot - J_d@q_dot
#         zeta_safe = zeta - d_dot_safe
#         self.previous_time = current_time
#         self.d_previous = distance

class VFI_Solver:
    def __init__(self, q_min, q_max, d_safe):
        self.q_min = [-1.599,-np.pi/3,0,-6.28, 0,-3.14]
        self.q_max = [1.599,np.pi/3,2.4,6.28, 1,57,3.14]
        self.d_safe = d_safe
        self.eta_t = 0.5 #position gain
        self.eta_r = 0.5 #rotation gain
        self.damping = 0.3 #qdot damping factor
        self.eta_d
        self.dt = 1/25 
    
    def update_timestep(self, dt):
        self.dt = dt
    
    def vfi_optimization_step(self, J_t, J_r, gradient, distance, q_current,
                               p_desired, p_current, r_desired, r_current):

        # Formulate Optimization Problem 
        v_desired = p_desired - p_current
        r_diff  = R.from_matrix(r_desired)*R.from_matrix(r_current)
    
        # Calculate the error vector
        omega_desired = r_diff.as_rotvec()
        Lambda = np.diag([0.75, 0.75, 0.75, 0.25, 0.25, 0.25])

        Q = 2 * (self.eta_t * J_t.T @ J_t + self.eta_r * J_r.T @ J_r + Lambda.T @ Lambda)

        # Calculate p
        p = -2 * (self.eta_t * J_t.T @ v_desired + self.eta_r * J_r.T @ omega_desired)

        # Convert to CVXOPT format
        Q_cvx = matrix(Q)
        p_cvx = matrix(p)

        # Joint angle constraints
        delta_q_min = (self.q_min - q_current) / self.dt
        delta_q_max = (self.q_max - q_current) / self.dt
        G_angle = np.vstack([-np.eye(6), np.eye(6)])
        h_angle = np.hstack([delta_q_min, delta_q_max])

        #distance constraints
        normal = -(gradient / np.linalg.norm(gradient))
        J_d = np.dot(normal,J_t)
        d_error = distance - self.d_safe
        G_distance = -J_d  # Jacobian for distance constraint
        h_distance = self.eta_d * d_error

        # Combine all constraints
        G = matrix(np.vstack([G_angle, G_distance]))
        h = matrix(np.hstack([h_angle, h_distance]))

        # Solve the QP problem
        sol = solvers.qp(Q_cvx, p_cvx, G, h)
        q_optimal = np.array(sol['x']).flatten()

        return q_optimal


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
        self.dt = 0.0

        self.force_pub = rospy.Publisher(f'/ambf/env/{psm_id}/servo_cp', WrenchStamped, queue_size=10)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_1/State', RigidBodyState, self.pose_callback_gripper_1)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_2/State', RigidBodyState, self.pose_callback_gripper_2)
        rospy.Subscriber(f'/ambf/env/{psm_id}/gripper_holder/State', RigidBodyState, self.velocity_callback)
        self.force_magnitude_pub = rospy.Publisher(f'/ambf/env/{psm_id}/force_magnitude', Float32, queue_size=10)
        self.distance_pub = rospy.Publisher(f'/ambf/env/{psm_id}/distance', Float32, queue_size=10)
        self.gradient_pub = rospy.Publisher(f'/ambf/env/{psm_id}/gradient', Vector3, queue_size=10)

        self.last_time = rospy.Time.now()
        self.gripper_1 = gripper_1_mesh
        self.gripper_2 = gripper_2_mesh
        self.dema_calculator = DoubleExponentialMovingAverage(alpha)
        self.dema_calculator_speed= DoubleExponentialMovingAverage(0.3)


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
            self.dt = (current_time - self.last_time).to_sec()
            if self.dt > 0:
                self.velocity = (position - self.prev_position) / self.dt
        
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
        distances = -self.sdf(gripper_vertices)
        min_distance = np.min(distances)
        min_index = np.argmin(distances)
        min_distance_vertex = gripper_vertices[min_index]

        gradient = compute_sdf_gradient(self.sdf, min_distance_vertex)
        force_direction, force_magnitude = generate_haptic_force(gradient, min_distance, self.velocity)
        #smoothed_speed = self.dema_calculator_speed(np.linalg.norm(self.velocity))



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

        gradient_vec = Vector3()
        gradient_vec.x = gradient[0]
        gradient_vec.y = gradient[1]
        gradient_vec.z = gradient[2]
        self.gradient_pub.publish(gradient_vec)

def main():

    rospy.init_node('haptic_force_generator', anonymous=True)

    #fetal_mesh = trimesh.load('src/dvrk_ctr_teleop/data/30Phantom.STL')
    fetal_mesh = trimesh.load('src/dvrk_ctr_teleop/data/deformable_fixed_final_001.OBJ',force ='mesh')
    gripper_1 = trimesh.load('src/dvrk_ctr_teleop/data/gripper_1.STL')
    gripper_2 = trimesh.load('src/dvrk_ctr_teleop/data/gripper_2.STL')

    # position = np.array([-0.10757, 0.36021, 1.10982])
    # quaternion = np.array([-0.7700439296218831,
    #                  0.0, 
    #                  0.0,
    #                  0.6379908670604059])

    # transformation_matrix = transform_mesh(position, quaternion)
    # fetal_mesh.apply_transform(transformation_matrix)
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
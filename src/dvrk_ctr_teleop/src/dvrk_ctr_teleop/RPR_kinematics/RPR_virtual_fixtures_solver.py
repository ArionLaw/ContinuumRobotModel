import numpy as np
import cvxpy as cp
from scipy.spatial.transform import Rotation as R
import rospy
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float32
from dvrk_ctr_teleop.RPR_kinematics.RPR_jacobian import compute_translational_jacobian, compute_rotational_jacobian
import pdb

###DEPRECIATED
# class VFI_Solver:
#     def __init__(self, l_rcc, l_tool, l_wrist, psm_id, q_min, q_max, d_safe):
        
#         self.l_rcc = l_rcc
#         self.l_tool = l_tool
#         self.l_wrist = l_wrist 
#         self.q_min = np.array(q_min)
#         self.q_max = np.array(q_max)
#         self.d_safe = d_safe
#         self.eta_t = 0.5 # position gain
#         self.eta_r = 0.5 # rotation gain
#         self.eta_d = 0.6
#         self.dt = 1/25
#         self.gradient = np.array([0,0,0],dtype=float)
#         self.J_t = np.zeros([3,6])
#         self.J_r = np.zeros([3,6])

#         rospy.Subscriber(f'/ambf/env/{psm_id}/gradient', Vector3, self.gradient_callback)
#         rospy.Subscriber(f'/ambf/env/{psm_id}/distance', Float32, self.distance_callback)
    
#     def update_timestep(self, dt):
#         self.dt = dt
    
#     def gradient_callback(self,msg):
#         self.gradient[0] = msg.x
#         self.gradient[1] = msg.y
#         self.gradient[2] = msg.z
    
#     def distance_callback(self,msg):
#         self.distance = msg.data

#     def vfi_optimization_step(self, q_current, p_desired, p_current, r_desired, r_current):

#         # Formulate the translational and rotational errors
#         translation_error = p_desired - p_current
#         r_diff = R.from_matrix(r_desired) * R.from_matrix(r_current).inv()
#         rotation_error = r_diff.as_rotvec()

#         #Calculate Jacobians
#         self.J_t = compute_translational_jacobian(self.J_t, q_current, self.l_rcc, self.l_tool, self.l_wrist)
#         self.J_r = compute_rotational_jacobian(self.J_r, q_current)

#         # Define the optimization variable
#         q_dot = cp.Variable(6)
#         self.Lambda = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])

#         # Objective function: Minimize the translational and rotational error
#         objective = cp.Minimize(
#             self.eta_t*cp.norm(self.J_t @ q_dot + self.eta_t * translation_error) +
#             self.eta_r*cp.norm(self.J_r @ q_dot + self.eta_r * rotation_error) +
#             cp.norm(self.Lambda @ q_dot)
#         )

#         # Joint angle constraints
#         delta_q_min = (self.q_min - q_current) / self.dt
#         delta_q_max = (self.q_max - q_current) / self.dt
#         joint_constraints = [q_dot >= delta_q_min, q_dot <= delta_q_max]

#         # Distance constraint
#         normal = (self.gradient / np.linalg.norm(self.gradient))
#         J_d = normal.T@self.J_t
#         d_error = self.distance - self.d_safe
#         distance_constraint = [-J_d @ q_dot <= self.eta_d * d_error]

#         # Solve the problem
#         constraints = joint_constraints + distance_constraint
#         problem = cp.Problem(objective, constraints)
#         problem.solve()

#         q_optimal = q_dot.value.flatten()

#         return q_optimal, problem.status
    

class VFI_Solver:
    def __init__(self, l_rcc, l_tool, l_wrist, psm_id, q_min, q_max, d_safe):
        
        self.l_rcc = l_rcc
        self.l_tool = l_tool
        self.l_wrist = l_wrist 
        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)
        self.d_safe = d_safe
        self.eta_t = 0.5 # position gain
        self.eta_r = 0.5 # rotation gain
        self.eta_d = 0.6 #Distance Gain 
        self.dt = 1/25
        self.gradient = np.array([0,0,0],dtype=float)
        self.J_t = np.zeros([3,6])
        self.J_r = np.zeros([3,6])

        rospy.Subscriber(f'/ambf/env/{psm_id}/gradient', Vector3, self.gradient_callback)
        rospy.Subscriber(f'/ambf/env/{psm_id}/distance', Float32, self.distance_callback)
    
    def update_timestep(self, dt):
        self.dt = dt
    
    def gradient_callback(self,msg):
        self.gradient[0] = msg.x
        self.gradient[1] = msg.y
        self.gradient[2] = msg.z
    
    def distance_callback(self,msg):
        self.distance = msg.data

    def vfi_optimization_step(self, q_current, q_desired):
         
        n_joints = len(q_current)

        # Define the optimization variable
        q_adjusted = cp.Variable(n_joints)

        # Objective: Minimize the difference between adjusted and output joint angles
        objective = cp.Minimize(cp.norm(q_adjusted - q_desired)**2)
        normal = (self.gradient / np.linalg.norm(self.gradient))
        self.J_t = compute_translational_jacobian(self.J_t, q_current, self.l_rcc, self.l_tool, self.l_wrist)
        J_d = -normal.T@self.J_t
        #distance = np.clip(self.distance, 0.0, 100)

        d_error = self.distance - self.d_safe
        # Constraints
        constraints = [
            -J_d @ ((q_adjusted - q_current) / self.dt) <= self.eta_d * d_error,  # Velocity constraint
            q_adjusted >= self.q_min,  # Joint lower limits
            q_adjusted <= self.q_max   # Joint upper limits
        ]

        # Define the optimization problem
        problem = cp.Problem(objective, constraints)

        # Solve the problem
        problem.solve()
        print(self.distance)
        #pdb.set_trace()

        # Check if the problem was solved successfully
        if problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
            # Get the adjusted joint angles
            q_adjusted_value = q_adjusted.value.flatten()
        else:
            # If the problem is not solved, return current joint angles
            q_adjusted_value = q_desired
            print("Warning: Optimization problem not solved. Returning current joint angles.")

        return q_adjusted_value
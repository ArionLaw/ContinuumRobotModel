import numpy as np
from geometry_msgs.msg import TransformStamped
from dvrk_planning.utilities import convert_mat_to_frame

# q0 is qw. qx, qy, qz
def quaternion_rotation_matrix(q0, q1, q2, q3):
    """
    Covert a quaternion into a full three-dimensional rotation matrix.

    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 

    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """

    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)

    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)

    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1

    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])

    return rot_matrix

def gm_tf_to_numpy_mat(gm_transform):
    np_transform = np.identity(4)
    np_transform[0:3, 0:3] = quaternion_rotation_matrix(gm_transform.rotation.w, \
        gm_transform.rotation.x, gm_transform.rotation.y, gm_transform.rotation.z)
    np_transform[0:3, 3] = [gm_transform.translation.x, gm_transform.translation.y, gm_transform.translation.z]
    return np_transform

def numpy_mat_to_gm_tf(mat):
    tf_stamped = TransformStamped()
    tf_stamped.transform.translation.x = mat[0, 3]
    tf_stamped.transform.translation.y = mat[1, 3]
    tf_stamped.transform.translation.z = mat[2, 3]
    q = convert_mat_to_frame(mat).M.GetQuaternion()
    tf_stamped.transform.rotation.x = q[0]
    tf_stamped.transform.rotation.y = q[1]
    tf_stamped.transform.rotation.z = q[2]
    tf_stamped.transform.rotation.w = q[3]
    return tf_stamped

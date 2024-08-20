import numpy as np

def compute_translational_jacobian(J, q, l_rcc, l_tool, l_wrist):
    q1, q2, q3, q4, q5, q6 = q

    # First row of the Jacobian
    J[0, 0] = (l_wrist * (np.cos(q6) * np.cos(q1 + np.pi/2) - np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2)) - np.sin(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)))))/2 + np.cos(q1 + np.pi/2) * (l_tool + l_wrist/2) - np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) * (l_rcc - q3)
    J[0, 1] = np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * (l_rcc - q3) - (l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4)) - np.sin(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2))))/2
    J[0, 2] = -np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2)
    J[0, 3] = -(l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4)) - np.sin(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2))))/2
    J[0, 4] = -(l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4)) - np.sin(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2))))/2
    J[0, 5] = -(l_wrist * (np.sin(q6) * np.sin(q1 + np.pi/2) + np.cos(q6) * (np.cos(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2)) + np.sin(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4)))))/2

    # Second row of the Jacobian
    J[1, 0] = np.sin(q1 + np.pi/2) * (l_tool + l_wrist/2) - (l_wrist * (np.sin(q6) * (np.cos(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2)) + np.sin(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4))) - np.cos(q6) * np.sin(q1 + np.pi/2)))/2 + np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) * (l_rcc - q3)
    J[1, 1] = np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) * (l_rcc - q3) - (l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)) + np.sin(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2))))/2
    J[1, 2] = -np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2)
    J[1, 3] = -(l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)) + np.sin(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2))))/2
    J[1, 4] = -(l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)) + np.sin(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2))))/2
    J[1, 5] = (l_wrist * (np.cos(q1 + np.pi/2) * np.sin(q6) + np.cos(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2)) - np.sin(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)))))/2

    # Third row of the Jacobian
    J[2, 0] = 0
    J[2, 1] = np.sin(q2 - np.pi/2) * (l_rcc - q3) + (l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) - np.sin(q4) * np.sin(q2 - np.pi/2)) - np.sin(q5) * (np.cos(q4) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4))))/2
    J[2, 2] = np.cos(q2 - np.pi/2)
    J[2, 3] = (l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) - np.sin(q4) * np.sin(q2 - np.pi/2)) - np.sin(q5) * (np.cos(q4) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4))))/2
    J[2, 4] = (l_wrist * np.sin(q6) * (np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) - np.sin(q4) * np.sin(q2 - np.pi/2)) - np.sin(q5) * (np.cos(q4) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4))))/2
    J[2, 5] = (l_wrist * np.cos(q6) * (np.cos(q5) * (np.cos(q4) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4)) + np.sin(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) - np.sin(q4) * np.sin(q2 - np.pi/2))))/2

    return J

def compute_rotational_jacobian(J, q):
    q1, q2, q3, q4, q5, q6 = q

    J = np.array([
        [
            0,
            np.sin(q1 + np.pi/2),
            0,
            np.sin(q1 + np.pi/2),
            np.sin(q1 + np.pi/2),
            np.sin(q5) * (np.cos(q1 + np.pi/2) * np.sin(q4) * np.sin(q2 - np.pi/2) - np.cos(q4) * np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2)) 
            - np.cos(q5) * (np.cos(q4) * np.cos(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q1 + np.pi/2) * np.cos(q2 - np.pi/2) * np.sin(q4))
        ],
        [
            0,
            -np.cos(q1 + np.pi/2),
            0,
            -np.cos(q1 + np.pi/2),
            -np.cos(q1 + np.pi/2),
            -np.cos(q5) * (np.cos(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4) * np.sin(q1 + np.pi/2)) 
            - np.sin(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) * np.sin(q1 + np.pi/2) - np.sin(q4) * np.sin(q1 + np.pi/2) * np.sin(q2 - np.pi/2))
        ],
        [
            1,
            0,
            0,
            0,
            0,
            np.cos(q5) * (np.cos(q4) * np.cos(q2 - np.pi/2) - np.sin(q4) * np.sin(q2 - np.pi/2)) 
            - np.sin(q5) * (np.cos(q4) * np.sin(q2 - np.pi/2) + np.cos(q2 - np.pi/2) * np.sin(q4))
        ]
    ])

    return J

import math
import numpy as np

'''
Holds functions to help compute things.

'''

def ConvertToTransformMatrix(R,p):
    T = np.identity(4)
    for i in range(3):
        for j in range(3):
            T[i,j] = R[(i,j)]                              
    for i in range(3):
        T[i,3] = p[i]
    return T

def getEulerAngles(R):
    if abs(R[2,0]) != 1:
        y_angle_theta = [-np.arcsin(R[2,0]) , np.pi - (-np.arcsin(R[2,0]))]
        x_angle_psi = [math.atan2(R[2,1]/np.cos(y_angle_theta[0]),R[2,2]/np.cos(y_angle_theta[0])) , math.atan2(R[2,1]/np.cos(y_angle_theta[1]),R[2,2]/np.cos(y_angle_theta[1]))]
        z_angle_phi = [math.atan2(R[1,0]/np.cos(y_angle_theta[0]),R[0,0]/np.cos(y_angle_theta[0])) , math.atan2(R[1,0]/np.cos(y_angle_theta[1]),R[0,0]/np.cos(y_angle_theta[1]))]
        angle_set1 = [x_angle_psi[0],y_angle_theta[0],z_angle_phi[0]]
        angle_set2 = [x_angle_psi[1],y_angle_theta[1],z_angle_phi[1]]
        return angle_set1
    else:
        z_angle_phi = 0
        if R[3,1] == -1:
            y_angle_theta = np.pi/2
            x_angle_psi = z_angle_phi + math.atan2(R[1,2],R[1,3])
        else:
            y_angle_theta = -np.pi/2
            x_angle_psi = -z_angle_phi + math.atan2(-R[1,2],-R[1,3])
        return [x_angle_psi,y_angle_theta,z_angle_phi]

def RotMtx(a,theta):
    ct = np.cos(theta)
    st = np.sin(theta)
    if a == 'z':
        R = np.array([[ct , -st , 0],
                      [st ,  ct , 0],
                      [0  ,  0  , 1]])
    elif a == 'y':
        R = np.array([[ct  , 0 , st],
                      [0   , 1 , 0],
                      [-st , 0 , ct]])
    elif a == 'x':
        R = np.array([[1 , 0  ,  0],
                      [0 , ct , -st],
                      [0 , st ,  ct]])
    else:
        R = np.array([[1 , 0 , 0],
                      [0 , 1 , 0],
                      [0 , 0 , 1]])
    return R

def getDistance(p1, p2):
    '''
    Calculates the straight distance given two points in 2D.
    '''
    res = math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    return res

def getDistance3D(p1, p2):
    '''
    Calculates the straight distance given two points in 3D.
    '''
    res = np.linalg.norm(p1-p2)
    return res

def getSlope(p1, p2):
    '''
    Calculates the slope between two points in 2D.
    '''
    res = (p2[1] - p1[1]) / (p2[0] - p1[0])
    return res

def getPerpendicularSlope(m):
    '''
    Calculates the perpendicular slope given a slope in 2D.
    '''
    res = - 1 / m
    return res

def getHypotenuse(a, b):
    '''
    Calculates the hypotenuse of a right triangle given the two minor sides.
    '''
    res = math.sqrt(a**2 + b**2)
    return res

def getTriangleLeg(a, h):
    '''
    Calculates the length of a right triangle given the hypotenuse (h) and a minor side (or leg) (a).
    '''
    res = math.sqrt(h**2 - a**2)
    return res

def get_arc_length(p1, p2, r):
    '''
    Calculates the arclength of a circle with radius r given two points on the circle p1 and p2.
    '''
    dist = getDistance(p1, p2)
    res = 2 * r * math.asin(dist / (2 * r))
    return res
   
def proj(u, v):
    '''
    Function to get the vector projection of u onto v.
    Both u and v should be numpy arrays.
    '''
    res = (np.dot(u, v) / np.dot(v, v)) * v
    return res

def projOntoPlane(a, n):
    '''
    Returns the projection of vector, a, onto a plane defined by normal vector, n.
    '''
    res = a - proj(a, n)
    return res

def getEndPoseAngleToBase(frame, point):
    '''
    Returns the angle of a point with respect to the xy-plane of the given frame (i.e., in nullspace of frame).
    '''
    # Project the point onto the xy-plane
    projVec = projOntoPlane(point-frame[0:3, 3], frame[0:3, 2])
    # Get the angle based on the x- and y-axes
    angle = np.arctan2(np.dot(projVec, frame[0:3, 1]), np.dot(projVec, frame[0:3, 0]))
    # change angle from [-pi, pi] to [0, 2*pi] for convenience
    if angle < 0:
        angle = angle + 2 * np.pi

    return angle

def getAngleTwoVectors(v1, v2):
    '''
    Returns the angle between two vectors in 3D
    '''
    angle = np.arccos(np.dot(v1, v2) / (np.linalg.norm(v1, ord=2) * np.linalg.norm(v2, ord=2)))
    return angle

def getRotationAxis(v1, v2, angle):
    """
    Returns the axis of rotation between 2 vectors in 3D
    Angle in Radians
    """
    axis = np.cross(v1,v2)/np.linalg.norm(v1, ord=2)/np.linalg.norm(v2, ord=2)/np.sin(angle)
    return axis

def fitCircleToPoints2D(P):
    '''
    Returns circle parameters ((x, y) circle centroid and radius r) given a list of 2D points, P (nx2).
    '''
    n = np.shape(P)[0]
    # Compute fitting parameters for circle: (x, y) center and radius, r
    avgP = np.mean(P, axis=0)
    u = P[:, 0] - avgP[0]*np.ones(n)
    v = P[:, 1] - avgP[1]*np.ones(n)
    suu = np.sum(np.power(u, 2))
    suv = np.sum(u * v)
    svv = np.sum(np.power(v, 2))
    suuu = np.sum(np.power(u, 3))
    svvv = np.sum(np.power(v, 3))
    svuu = np.sum(v * u * u)
    suvv = np.sum(u * v * v)

    A = np.array([[suu, suv], [suv, svv]])
    b = np.array([[0.5*(suuu + suvv)], [0.5*(svvv + svuu)]])
    x = np.linalg.solve(A, b)
    res = np.array([x[0]+avgP[0], x[1]+avgP[1]])

    alpha = x[0]**2 + x[1]**2 + (suu + svv) / n
    r = np.sqrt(alpha)

    return res, r

def getCircleInFrame(frame, r, numPts):
    '''
    Returns a list of points representing a circle in the xy-plane of frame, centered at point
    defined by frame, and with radius r.

    Inputs:
        - frame: 4x4 homogeneous transformation matrix
    '''
    theta = np.tile(np.linspace(0, 2*np.pi, numPts), (3, 1))
    theta = np.transpose(theta)
    points = r * np.tile(frame[0:3, 0], (numPts, 1)) * np.cos(theta) + r * np.tile(frame[0:3, 1], (numPts, 1)) * np.sin(theta)
    points = points + np.tile(frame[0:3, 3], (numPts, 1))
    return points
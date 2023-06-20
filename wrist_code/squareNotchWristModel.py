from utils import *
from plotting import *

'''
Python file for coding wrist model. Assuming a 2DOF (notches at 120 degrees).

Key parameters:
- n: number of notches for one bending plane; total number of notches is 3*n
- c: distance between notches
- h: length of notch
- prevStraightLen: distance from starting point to first notch
- postStraightLen: distance from last notch to end of tube
- OD: outer diameter of tube
- ID: inner diameter of tube

Secondary parameters (e.g. for testing, visualization):
- d: number of points around circle
- m: number of points per notch/division

Notes:
- Model does not currently consider the introduced slack in Francis' thesis (when two cables are actuated)
'''

def cableToEndPoints_square_tube(l, startPt, initMatrix, y, ID, n, m, d, c, h, r_o, straightLen1, straightLen2):
    '''
    To create a single function that takes in cable displacements (and geometric parameters) and returns the tube points.
    '''
    k1, s1 = getBendingPlaneParams(h, l[0], y, ID)
    k2, s2 = getBendingPlaneParams(h, l[1], y, ID)
    k3, s3 = getBendingPlaneParams(h, l[2], y, ID)
    tubePts,  backbonePts = squareWristFK_tube(startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, r_o, straightLen1, straightLen2)
    return tubePts,  backbonePts

def cableToEndPoints_square_backbone(l, startPt, initMatrix, y, ID, n, m, c, h, straightLen1, straightLen2):
    '''
    To create a single function that takes in cable displacements (and geometric parameters) and returns the center-line points (for end-effector position).
    '''
    k1, s1 = getBendingPlaneParams(h, l[0], y, ID)
    k2, s2 = getBendingPlaneParams(h, l[1], y, ID)
    k3, s3 = getBendingPlaneParams(h, l[2], y, ID)
    pts = squareWristFK_backbone(startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, c, h, straightLen1, straightLen2)
    return pts

def getBendingPlaneParams(h, l, y, ID):
    '''
    Returns the curvature, and arclength based on provided values (geometric and user inputs).
    From York, Swaney (2015).
    '''
    k = l / (h*(ID/2 + y) - l*y)
    s = h / (1 + y * k)
    return (k, s)

def squareWristFK_tube(startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, r_o, straightLen1, straightLen2):
    '''
    Computes FK for the robot based on geometric, computed, and user inputs.
    Will need initial point and direction from CTR code.

    Returns tube points of a straight tube (with no notches; e.g. as a patch).

    Assuming an initial straight section, should have (3*n)*(m+2)+3 (*d) points.
        -> m + 2 as each notch as m+2 levels (including first and last levels)
            - also as Tc is separated into c translation and rotation by 120 degrees (to make face indices creation easier (and straight))
        -> +3 as first, second, and last levels of total wrist
    Point layers go: base/CTR end point, first straight length, 5 layers for notches, 1 layer for c, 1 layer for rotation of reference frame,
        repeat these 7 layers 3*n times, last straight length (end of wrist)
    '''
    tubeNumPoints = (int(n)*3*(m + 2) + 3) * d
    tubePoints = np.zeros((tubeNumPoints, 3))
    backboneNumPoints = (int(n)*3*(m + 2) + 3)
    backbonePoints = np.zeros((backboneNumPoints, 3))
    # Create transformational matrices
    # Initialize first matrix
    T1 = np.eye(4)
    T1[0:3, 0:3] = initMatrix
    T1[0, 3] = startPt[0]
    T1[1, 3] = startPt[1]
    T1[2, 3] = startPt[2]
    # Matrix for first straight length
    T2 = np.eye(4)
    T2[2, 3] = straightLen1
    # Matrices for rotations for the curved portions
    Tj1 = getTj(k1, s1, m, h)
    Tj2 = getTj(k2, s2, m, h)
    Tj3 = getTj(k3, s3, m, h)
    Tc1 = np.eye(4)
    Tc1[2, 3] = c
    Tc2 = np.array([[np.cos(2 * np.pi / 3), -np.sin(2 * np.pi / 3), 0, 0],
                    [np.sin(2 * np.pi / 3), np.cos(2 * np.pi / 3), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ])

    # Perform forward kinematics to get points
    T = T1
    pt = np.reshape(T[0:3, 3], (1, 3))
    tubePoints[0:d, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    backbonePoints[0, :] = pt
    T = np.matmul(T, T2)
    pt = np.reshape(T[0:3, 3], (1, 3))
    tubePoints[d:2*d, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    backbonePoints[1, :] = pt
    for i in range(int(n)*3):
        if (i % 3) == 0:     # 0, 3, 6
            Tj = Tj1
        elif (i % 3) == 1:   # 1, 4, 7
            Tj = Tj2
        elif (i % 3) == 2:  # 2, 5, 8
            Tj = Tj3

        for j in range(m):
            T = np.matmul(T, Tj)
            pt = np.reshape(T[0:3, 3], (1, 3)) 
            ind = (i*(m + 2) + j + 2)
            backbonePoints[ind, :] = pt
            ind *= d
            tubePoints[ind:ind+d, :] = getCirclePoints(pt,
                                                        T[0:3, 0:3], r_o, d)

        T = np.matmul(T, Tc1)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = ((i+1)*(m + 2))
        backbonePoints[ind, :] = pt
        ind *= d
        tubePoints[ind:ind+d, :] = getCirclePoints(pt,
                                                    T[0:3, 0:3], r_o, d)

        T = np.matmul(T, Tc2)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = ((i+1)*(m + 2) + 1)
        backbonePoints[ind, :] = pt
        ind *= d
        tubePoints[ind:ind+d, :] = getCirclePoints(pt,
                                                    T[0:3, 0:3], r_o, d)

    # Extend by last straight length
    Tf = np.eye(4)
    # subtract c from last value as Tc is conducted above (3*n)
    Tf[2, 3] = straightLen2 - c
    T = np.matmul(T, Tf)
    pt = np.reshape(T[0:3, 3], (1, 3))
    backbonePoints[backboneNumPoints-1, :] = pt
    tubePoints[tubeNumPoints-d:tubeNumPoints, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    return tubePoints, backbonePoints

def squareWristFK_backbone(startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, c, h, straightLen1, straightLen2):
    '''
    Computes FK for the robot based on geometric, computed, and user inputs.
    Will need initial point and direction from CTR code.

    Returns backbone points of a single line (tube with 0 thickness; with no notches; e.g. as a patch).
    '''
    backboneNumPoints = (int(n)*3*(m + 2) + 3)
    backbonePoints = np.zeros((backboneNumPoints, 3))
    # Create transformational matrices
    # Initialize first matrix
    T1 = np.eye(4)
    T1[0:3, 0:3] = initMatrix
    T1[0, 3] = startPt[0]
    T1[1, 3] = startPt[1]
    T1[2, 3] = startPt[2]
    # Matrix for first straight length
    T2 = np.eye(4)
    T2[2, 3] = straightLen1
    # Matrices for rotations for the curved portions
    Tj1 = getTj(k1, s1, m, h)
    Tj2 = getTj(k2, s2, m, h)
    Tj3 = getTj(k3, s3, m, h)
    Tc1 = np.eye(4)
    Tc1[2, 3] = c
    Tc2 = np.array([[np.cos(2 * np.pi / 3), -np.sin(2 * np.pi / 3), 0, 0],
                    [np.sin(2 * np.pi / 3), np.cos(2 * np.pi / 3), 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                    ])

    # Perform forward kinematics to get points
    T = T1
    pt = np.reshape(T[0:3, 3], (1, 3))
    backbonePoints[0, :] = pt
    T = np.matmul(T, T2)
    pt = np.reshape(T[0:3, 3], (1, 3))
    backbonePoints[1, :] = pt
    for i in range(int(n)*3):
        if (i % 3) == 0:     # 0, 3, 6
            Tj = Tj1
        elif (i % 3) == 1:   # 1, 4, 7
            Tj = Tj2
        elif (i % 3) == 2:  # 2, 5, 8
            Tj = Tj3

        for j in range(m):
            T = np.matmul(T, Tj)
            pt = np.reshape(T[0:3, 3], (1, 3)) 
            ind = (i*(m + 2) + j + 2)
            backbonePoints[ind, :] = pt

        T = np.matmul(T, Tc1)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = ((i+1)*(m + 2))
        backbonePoints[ind, :] = pt

        T = np.matmul(T, Tc2)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = ((i+1)*(m + 2) + 1)
        backbonePoints[ind, :] = pt

    # Extend by last straight length
    Tf = np.eye(4)
    # subtract c from last value as Tc is conducted above (3*n)
    Tf[2, 3] = straightLen2 - c
    T = np.matmul(T, Tf)
    pt = np.reshape(T[0:3, 3], (1, 3))
    backbonePoints[backboneNumPoints-1, :] = pt
    return backbonePoints

def getTj(k, s, m, h):
    '''
    Helper function to get transformation matrix for bending portion.
    Result depends on if k is 0 (straight) or not.
    '''
    if k == 0:
        res = np.array([[1, 0, 0, 0],
                        [0, 1, 0, 0],
                        [0, 0, 1, h / m],
                        [0, 0, 0, 1]
                        ])
    else:
        theta = k*s / m
        res = np.array([[1, 0, 0, 0],
                        [0, np.cos(theta), -np.sin(theta),
                            (np.cos(theta)-1)/k],
                        [0, np.sin(theta), np.cos(theta), np.sin(theta)/k],
                        [0, 0, 0, 1]
                        ])
    return res

def getCirclePoints(startPt, orientation, r_o, d):
    '''
    Helper function to get the tube points at a single cross-section.
    Based on startPt and points that lie on the xy-plane (nullspace of z-axis).
    r_o - outer radius of tube
    d - number of points around the cross-section 
    '''
    points = np.zeros((d, 3))
    theta = 2 * np.pi / d
    x = np.reshape(orientation[0:3, 0], (1, 3))
    y = np.reshape(orientation[0:3, 1], (1, 3))
    for i in range(d):
        points[i, :] = startPt + r_o * (np.cos(i * theta) * x +
                                        np.sin(i * theta) * y)
    return points

def getSquareWristFaces(n, m, d):
    '''
    Gets the face indices for the wrist tube object (for visualization at a patch-like object).
    Take four pts/vertices per face. Should have 3*n*(m+2)+2 (*d) at start; same as number of points but minus 1 (*d).
    Removed faces for c translation to 120deg rotation; should have 3*n*(m+1)+2 (*d) at end.
    
    Output: list of lists with the four indices of the vertices.
    For compatibility with 3DSlicer
    '''
    inds = [0] * ((3*int(n)*(m+2)+2) * d)
    for i in range(3*int(n)*(m+2)+2):
        for j in range(d):
            v1 = (i)*d + j
            v2 = (i)*d + j + 1
            v3 = (i+1)*d + j + 1
            v4 = (i+1)*d + j
            if j == (d-1):
                v2 = (i)*d + j + 1 - d
                v3 = (i+1)*d + j + 1 - d
            inds[i*d + j] = [v1, v2, v3, v4]

    # Remove the faces from translation c to rotation 120 degrees
    for i in range(3*int(n), 0, -1):
        index = (i*(m + 2)) * d
        del inds[index:index+d]
    
    return inds

def convertFaceIndicestoVertices(verts, facesInd):
    '''
    Takes in list of vertices and list of face indices, and returns list of face vertices.
    i.e., replaces face indices with vertices.
    For compatibility with matplotlib.
    '''
    newFaces = []
    for ind in facesInd:
        face = []
        for v in ind:
            face.append(verts[v, :])
        newFaces.append(face)
    return newFaces

def updatePolyForNotch(vertices, faces, startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, r_o, straightLen1, g):
    '''
    Update the vertices and faces (representing a full tube) to include a notch design.
        -> m+1 (instead of m+2) as some faces already deleted earlier
    do not need to delete points, just faces (referencing those points)
    add points of arc
    add faces at end (referencing new points)
    '''
    newFaces = faces
    newVertices = vertices
    # Delete current notch faces
    for i in range(3*int(n)-1, -1, -1):
        ind = ((i)*(m + 1) + 1) * d
        del newFaces[ind:ind+m*d]

    # Redo FK to get new points for the notched areas
    numPoints = np.shape(vertices)
    numPoints = numPoints[0]
    notchPoints = getNotchPoints(
        startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, r_o, straightLen1, g)
    newVertices = np.append(newVertices, notchPoints, axis=0)
    # Get face indices
    notchFaces = getNotchFaces(numPoints, n, m, d)
    newFaces += notchFaces

    return newVertices, newFaces

def getNotchPoints(startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, r_o, straightLen1, g):
    '''
    Computes FK for Wrist to get points for the notched part of the tube.
    '''
    numPoints = (3*int(n)*(m + 1)) * d
    points = np.zeros((numPoints, 3))
    # Create transformational matrices
    T1 = np.eye(4)
    T1[0:3, 0:3] = initMatrix
    T1[0, 3] = startPt[0]
    T1[1, 3] = startPt[1]
    T1[2, 3] = startPt[2]
    # Extend by first straight length
    T2 = np.eye(4)
    T2[2, 3] = straightLen1
    # Rotations for the curved portion
    Tj1 = getTj(k1, s1, m, h)
    Tj2 = getTj(k2, s2, m, h)
    Tj3 = getTj(k3, s3, m, h)
    Tc = np.array([[np.cos(2 * np.pi / 3), -np.sin(2 * np.pi / 3), 0, 0],
                    [np.sin(2 * np.pi / 3), np.cos(2 * np.pi / 3), 0, 0],
                    [0, 0, 1, c],
                    [0, 0, 0, 1]
                    ])

    # Perform forward kinematics to get points
    T = T1
    T = np.matmul(T, T2)
    pt = np.reshape(T[0:3, 3], (1, 3))
    points[0:d, :] = getNotchCirclePoints(pt, T[0:3, 0:3], g, r_o, d)
    for i in range(int(n)*3):
        if (i % 3) == 0:     # 0, 3, 6
            Tj = Tj1
        elif (i % 3) == 1:   # 1, 4, 7
            Tj = Tj2
        elif (i % 3) == 2:  # 2, 5, 8
            Tj = Tj3

        for j in range(m):
            T = np.matmul(T, Tj)
            pt = np.reshape(T[0:3, 3], (1, 3))
            ind = (i*(m + 1) + j + 1) * d
            points[ind:ind+d, :] = getNotchCirclePoints(pt,
                                                        T[0:3, 0:3], g, r_o, d)

        if i == 8:
            # break as do not need last layer for c/120 rotation
            break
        T = np.matmul(T, Tc)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = ((i+1)*(m + 1)) * d
        points[ind:ind+d, :] = getNotchCirclePoints(pt,
                                                    T[0:3, 0:3], g, r_o, d)

    return points

def getNotchCirclePoints(startPt, orientation, g, r_o, d):
    '''
    Helper function to get points for notched part of tube at single cross-section.
    Assuming g > r_o
    '''
    y = g - r_o
    x = np.sqrt(r_o**2 - y**2)
    startTh = np.arctan2(y, x)
    th = 2 * startTh + np.pi
    tubeTh = (2 * np.pi - th) / (d-1)

    x_axis = np.reshape(orientation[0:3, 0], (1, 3))
    y_axis = np.reshape(orientation[0:3, 1], (1, 3))
    points = np.zeros((d, 3))
    for i in range(d):
        points[i, :] = startPt + r_o * (np.cos(startTh + i * tubeTh) * x_axis +
                                        np.sin(startTh + i * tubeTh) * y_axis)
    return points

def getNotchFaces(startInd, n, m, d):
    '''
    Gets face indices for notched part of tube. 
    '''
    inds = [0] * (3 * int(n) * m * (d-1))
    count = 0
    for i in range(3 * int(n) * (m + 1)):
        if ((i+1) % (m+1)) == 0 and i != 0:
            continue
        for j in range(d-1):
            v1 = (i)*d + j + startInd
            v2 = (i)*d + j + 1 + startInd
            v3 = (i+1)*d + j + 1 + startInd
            v4 = (i+1)*d + j + startInd
            # inds[(i - ((i+1) // (m+1))) *d + j] = (v1, v2, v3, v4)
            inds[count] = (v1, v2, v3, v4)
            count += 1
    return inds

#######################################################
#
#       Inverse Kinematics 2DOF Square Notch
#
#######################################################
# Not completed


#######################################################
#
#               1 DOF Square Notch
#
#######################################################

def cableToEndPoints_square1DOF_tube(l, startPt, initMatrix, y, ID, n, m, d, c, h, r_o, straightLen1, straightLen2):
    '''
    Takes in cable displacements (and geometric parameters) and returns the tube points, backbone points, and end orientation.
    '''
    k, s = getBendingPlaneParams(h, l, y, ID)
    tubePts,  backbonePts, T = square1DOFWristFK_tube(startPt, initMatrix, k, s, n, m, d, c, h, r_o, straightLen1, straightLen2)
    return tubePts,  backbonePts, T

def square1DOFWristFK_tube(startPt, initMatrix, k, s, n, m, d, c, h, r_o, straightLen1, straightLen2):
    '''
    Computes FK for the robot based on geometric, computed, and user inputs.
    Will need initial point and direction from CTR code.

    Returns tube points of a straight tube (with no notches; e.g. as a patch).

    Assuming an initial straight section, should have (n)*(m+1)+3 (*d) points.
        -> m + 2 as each notch as m+2 levels (including first and last levels)
            - also as Tc is separated into c translation and rotation by 120 degrees (to make face indices creation easier (and straight))
        -> +3 as first, second, and last levels of total wrist
    Point layers go: base/CTR end point, first straight length, 5 layers for notches, 1 layer for c, 1 layer for rotation of reference frame,
        repeat these 7 layers 3*n times, last straight length (end of wrist)
    '''
    tubeNumPoints = (int(n)*(m + 1) + 3) * d
    tubePoints = np.zeros((tubeNumPoints, 3))
    backboneNumPoints = (int(n)*(m + 1) + 3)
    backbonePoints = np.zeros((backboneNumPoints, 3))
    # Create transformational matrices
    # Initialize first matrix
    T1 = np.eye(4)
    T1[0:3, 0:3] = initMatrix
    T1[0, 3] = startPt[0]
    T1[1, 3] = startPt[1]
    T1[2, 3] = startPt[2]
    # Matrix for first straight length
    T2 = np.eye(4)
    T2[2, 3] = straightLen1
    # Matrices for rotations for the curved portions
    Tj = getTj(k, s, m, h)
    Tc = np.eye(4)
    Tc[2, 3] = c

    # Perform forward kinematics to get points
    T = T1
    pt = np.reshape(T[0:3, 3], (1, 3))
    tubePoints[0:d, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    backbonePoints[0, :] = pt
    T = np.matmul(T, T2)
    pt = np.reshape(T[0:3, 3], (1, 3))
    tubePoints[d:2*d, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    backbonePoints[1, :] = pt
    for i in range(int(n)):

        for j in range(m):
            T = np.matmul(T, Tj)
            pt = np.reshape(T[0:3, 3], (1, 3)) 
            ind = (i*(m + 1) + j + 2)
            backbonePoints[ind, :] = pt
            ind *= d
            tubePoints[ind:ind+d, :] = getCirclePoints(pt,
                                                        T[0:3, 0:3], r_o, d)

        T = np.matmul(T, Tc)
        pt = np.reshape(T[0:3, 3], (1, 3))
        ind = (i*(m + 1) + m + 2)
        backbonePoints[ind, :] = pt
        ind *= d
        tubePoints[ind:ind+d, :] = getCirclePoints(pt,
                                                    T[0:3, 0:3], r_o, d)


    # Extend by last straight length
    Tf = np.eye(4)
    # subtract c from last value as Tc is conducted above (n)
    Tf[2, 3] = straightLen2 - c
    T = np.matmul(T, Tf)
    pt = np.reshape(T[0:3, 3], (1, 3))
    backbonePoints[backboneNumPoints-1, :] = pt
    tubePoints[tubeNumPoints-d:tubeNumPoints, :] = getCirclePoints(pt, T[0:3, 0:3], r_o, d)
    return tubePoints, backbonePoints, T

def getSquare1DOFWristFaces(n, m, d):
    '''
    Gets the face indices for the wrist tube object (for visualization at a patch-like object).
    Take four pts/vertices per face. Should have 3*n*(m+2)+2 (*d) at start; same as number of points but minus 1 (*d).
    Removed faces for c translation to 120deg rotation; should have 3*n*(m+1)+2 (*d) at end.
    
    Output: list of lists with the four indices of the vertices.
    For compatibility with 3DSlicer
    '''
    inds = [0] * ((int(n)*(m+2)+2) * d)
    for i in range(int(n)*(m+2)+2):
        for j in range(d):
            v1 = (i)*d + j
            v2 = (i)*d + j + 1
            v3 = (i+1)*d + j + 1
            v4 = (i+1)*d + j
            if j == (d-1):
                v2 = (i)*d + j + 1 - d
                v3 = (i+1)*d + j + 1 - d
            inds[i*d + j] = [v1, v2, v3, v4]

    # Remove the faces from translation c to rotation 120 degrees
    for i in range(3*int(n), 0, -1):
        index = (i*(m + 2)) * d
        del inds[index:index+d]
    
    return inds

def main():
    # Set up wrist
    n = 3
    prevStraightLen = 5
    postStraightLen = 1
    h = 0.6
    c = 0.6
    y = 0.56
    g = 1.16
    OD = 1.25
    ID = 0.8
    m = 15
    d = 15

    # Start point and orientation of wrist; should be based on CTR
    startPt = np.array([0, 0, 0])
    initMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    l = np.array([0.2, 0, 0.2])
    tubePts, backbonePts = cableToEndPoints_square_tube(l, startPt, initMatrix, y, ID, n, m, d, c, h, OD/2, prevStraightLen, postStraightLen)
    facesInd = getSquareWristFaces(n, m, d)
    #points, faces = updatePolyForNotch(tubePts, facesInd, startPt, initMatrix, k1, s1, k2, s2, k3, s3, n, m, d, c, h, OD/2, prevStraightLen, g)
    newFaces = convertFaceIndicestoVertices(tubePts, facesInd)
  
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    
    # Plot 2DOF
    plotWrist(backbonePts, newFaces, ax)

    # Plot 1DOF
    l = 0.6
    tubePts, backbonePts, T = cableToEndPoints_square1DOF_tube(l, startPt, initMatrix, y, ID, n, m, d, c, h, OD/2, prevStraightLen, postStraightLen)
    endPt = backbonePts[-1, :]
    #facesInd = getSquare1DOFWristFaces(n, m, d)
    #newFaces = convertFaceIndicestoVertices(tubePts, facesInd)
    #plotWrist(backbonePts, newFaces, ax)

    ax.scatter3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey', s=4)
    ax.plot3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey')
    set_axes_equal(ax)
    
    # 2D plot of 1DOF wrist
    plt.figure()
    plt.plot(backbonePts[:, 1], backbonePts[:, 2], c='black')
    plt.axis('equal')

    plt.scatter(endPt[1], endPt[2], c='black')

    plt.show()

if __name__ == "__main__":
   main()
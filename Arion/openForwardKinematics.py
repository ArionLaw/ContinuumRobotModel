from squareNotchWristModel import *
import matplotlib.pyplot as plt

'''
Script to perform forward kinematics for wrist.
'''

if __name__ == "__main__":
    ########## For wrist
    # Start point and orientation of wrist; should be based on CTR
    startPt = np.array([0, 0, 0])
    initMatrix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])

    # Set up variables for wrist (kinematic parameters follow York et al. (2015)
    n = 3                   # number of sets of 3 cuts
    prevStraightLen = 5     # length from base to first notch
    postStraightLen = 1     # length from last notch to end 
    h = 0.66
    c = 0.66
    y = 0.56
    g = 1.16
    OD = 1.25
    ID = 0.8
    m = 15                  # visualization parameter
    d = 15                  # visualization parameter

    # Choose cable displacements
    l = [0.5, 0, 0.5]

    tubePts, backbonePts = cableToEndPoints_square_tube(l, startPt, initMatrix, y, ID, n, m, d, c, h, OD/2, prevStraightLen, postStraightLen)
    facesInd = getSquareWristFaces(n, 15, 15)
    newFaces = convertFaceIndicestoVertices(tubePts, facesInd)

    plotTubeFlag = True
    if plotTubeFlag:
        fig = plt.figure()
        ax = plt.axes(projection='3d')
        ax.scatter3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2])
        ax.plot3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2])
        #ax.scatter3D(tubePts[:, 0], tubePts[:, 1], tubePts[:, 2])
        ax.add_collection3d(Poly3DCollection(newFaces, facecolors='cyan', linewidths=0.5, alpha=.50))
        set_axes_equal(ax)
        plt.show()
import matplotlib.pyplot as plt
import math
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import time

'''
Functions for plotting.
'''

def drawCircle(t, scale, p, r):
    '''
    Draws circle at point p with radius r using turtle.
    '''
    t.penup()
    t.setx(p[0] * scale)
    t.sety((p[1] - r) * scale)
    t.pendown()
    t.circle(r * scale)

def drawDot(t, scale, p, r):
    '''
    Draws dot at point p with radius r using turtle.
    '''
    t.penup()
    t.setx(p[0] * scale)
    t.sety(p[1] * scale)
    t.pendown()
    t.dot(r)

def drawLine(t, scale, p1, p2):
    '''
    Draws a line from point p1 to p2 using turtle.
    '''
    t.width(3)
    t.penup()
    t.goto(p1 * scale)
    t.pendown()
    t.goto(p2 * scale)
    t.width(1)

def drawArc(t, scale, th1, th2, c, r):
    '''
    Draws an arc from point p1 an angle th with circle radius r and center c using turtle.
    Rotates in CW. th1 and th2 in radians and in absolute coordinates.
    '''
    t.width(3)
    t.penup()
    p1 = c + r * np.array([math.cos(th1), math.sin(th1)])
    t.goto(p1 * scale)
    t.pendown()

    # Case in which goes from negative to positive angle in CW direction (th1 to th2)
    # Possible if using atan2 and crosses -pi / pi barrier
    if th1 < 0 and th2 > 0:
        th1 = th1 + 2 * math.pi
    angles = np.linspace(th1, th2)
    for ind in range(len(angles)-1):
        p2 = c + r * np.array([math.cos(angles[ind]), math.sin(angles[ind])])
        t.goto(p2 * scale)

    t.width(1)

def plotWrist(backbonePts, faces, ax):
    '''
    Function to plot points in 3D for the wrist.
    '''
    ax.scatter3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey', s=4)
    ax.plot3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey')
    ax.add_collection3d(Poly3DCollection(faces, facecolors='grey', linewidths=0.5, alpha=.40))
    set_axes_equal(ax)

def returnPlotWrist(backbonePts, faces, ax, timeToPause):
    '''
    Function to animate plotted points in 3D for the wrist.
    '''
    pts = ax.scatter3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey', s=4)
    lines = ax.plot3D(backbonePts[:, 0], backbonePts[:, 1], backbonePts[:, 2], c='grey')
    tube = ax.add_collection3d(Poly3DCollection(faces, facecolors='grey', linewidths=0.5, alpha=.40))
    set_axes_equal(ax)
    #plt.pause(timeToPause)

    return pts, lines, tube
    pts.remove()
    lines[0].remove()
    tube.remove()
    plt.pause(0.0001)
    

def visualizeRP(motorAxis, gearPitch, angle, holderAxis1, holderAxis2, holder1CablePoint, holder2CablePoint, pointOfContact, holderDiameter, scale, t):
    '''
    Visualizes the rack and pinion actuation using turtle (components and cables).
    '''
    drawCircle(t, scale, motorAxis, gearPitch)
    drawCircle(t, scale, holderAxis1, holderDiameter/2)
    drawCircle(t, scale, holderAxis2, holderDiameter/2)

    drawDot(t, scale, motorAxis, 5)
    drawDot(t, scale, holderAxis1, 5)
    drawDot(t, scale, holderAxis2, 5)

    drawLine(t, scale, motorAxis, motorAxis + (gearPitch+1)*np.array([np.cos(np.deg2rad(angle)), np.sin(np.deg2rad(angle))]))

    drawLine(t, scale, holder1CablePoint, pointOfContact)
    drawLine(t, scale, holder2CablePoint, pointOfContact)


def visualizeWiper(motorAxis, wiperAxis, holderAxis1, holderAxis2, wiperP1, wiperP2, holderP1, holderP2, holder1CablePoint, holder2CablePoint, wiperDiameter, holderDiameter, motorDirection, scale, t):
    '''
    Visualizes the wiper actuation using turtle (components and cables).
    '''
    drawCircle(t, scale, motorAxis, 2)
    drawCircle(t, scale, wiperAxis, wiperDiameter/2)
    drawCircle(t, scale, holderAxis1, holderDiameter/2)
    drawCircle(t, scale, holderAxis2, holderDiameter/2)

    drawDot(t, scale, motorAxis, 5)
    drawDot(t, scale, wiperAxis, 5)
    drawDot(t, scale, holderAxis1, 5)
    drawDot(t, scale, holderAxis2, 5)

    # Draw cable path
    if motorDirection == 'CCW':
        drawLine(t, scale, holderAxis1+np.array([holderDiameter/2, 10]), holderAxis1+np.array([holderDiameter/2, 0]))
        drawLine(t, scale, holderAxis2+np.array([holderDiameter/2, -10]), holderAxis2+np.array([holderDiameter/2, 0]))
    elif motorDirection == 'CW':
        drawLine(t, scale, holderAxis1+np.array([-holderDiameter/2, 10]), holderAxis1+np.array([-holderDiameter/2, 0]))
        drawLine(t, scale, holderAxis2+np.array([-holderDiameter/2, -10]), holderAxis2+np.array([-holderDiameter/2, 0]))
    th1 = math.atan2(holder1CablePoint[1]-holderAxis1[1], holder1CablePoint[0]-holderAxis1[0])
    th2 = math.atan2(holderP1[1]-holderAxis1[1], holderP1[0]-holderAxis1[0])
    if motorDirection == 'CCW':
        drawArc(t, scale, th1, th2, holderAxis1, holderDiameter/2)
    elif motorDirection == 'CW':
        drawArc(t, scale, th2, th1, holderAxis1, holderDiameter/2)
    drawLine(t, scale, wiperP1, holderP1)
    th1 = math.atan2(wiperP2[1]-wiperAxis[1], wiperP2[0]-wiperAxis[0])
    th2 = math.atan2(wiperP1[1]-wiperAxis[1], wiperP1[0]-wiperAxis[0])
    if motorDirection == 'CCW':
        drawArc(t, scale, th1, th2, wiperAxis, wiperDiameter/2)
    elif motorDirection == 'CW':
        drawArc(t, scale, th2, th1, wiperAxis, wiperDiameter/2)
    drawLine(t, scale, wiperP2, holderP2)
    th1 = math.atan2(holderP2[1]-holderAxis2[1], holderP2[0]-holderAxis2[0])
    th2 = math.atan2(holder2CablePoint[1]-holderAxis2[1], holder2CablePoint[0]-holderAxis2[0])
    if motorDirection == 'CCW':
        drawArc(t, scale, th1, th2, holderAxis2, holderDiameter/2)
    elif motorDirection == 'CW':
        drawArc(t, scale, th2, th1, holderAxis2, holderDiameter/2)
    drawLine(t, scale, motorAxis, wiperAxis)

def set_axes_equal(ax):
    '''Make axes of 3D plot have equal scale so that spheres appear as spheres,
    cubes as cubes, etc..  This is one possible solution to Matplotlib's
    ax.set_aspect('equal') and ax.axis('equal') not working for 3D.

    Input
      ax: a matplotlib axis, e.g., as output from plt.gca().

    from https://stackoverflow.com/questions/13685386/matplotlib-equal-unit-length-with-equal-aspect-ratio-z-axis-is-not-equal-to
    '''

    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    # The plot bounding box is a sphere in the sense of the infinity
    # norm, hence I call half the max range the plot radius.
    plot_radius = 0.5*max([x_range, y_range, z_range])

    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])
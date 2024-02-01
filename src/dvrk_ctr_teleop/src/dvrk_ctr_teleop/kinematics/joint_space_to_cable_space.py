from dvrk_ctr_teleop.kinematics.utils import *
from dvrk_ctr_teleop.kinematics.plotting import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_NotchAngle_from_TotalCableDeltas(h,TotalCableDeltas):
    """
    interprets total cable displacement and decomposes into wrist joint angles based off of linear approximation
    """
    notch_height = h
    dInner_dTheta = (-0.02 - notch_height)/(30*np.pi/180)#(0.008 - notch_height)/(30*np.pi/180) # linear approximation
    dOuter_dTheta = (0.515 - notch_height)/(30*np.pi/180)#(0.54- notch_height)/(30*np.pi/180) # linear approximation
    M_dcable_dTheta = np.array([[-dInner_dTheta , -dOuter_dTheta , -dOuter_dTheta],
                                [-dOuter_dTheta , -dInner_dTheta , -dOuter_dTheta],
                                [-dOuter_dTheta , -dOuter_dTheta , -dInner_dTheta]])
    InvM_dcable_dTheta = np.linalg.inv(M_dcable_dTheta)
    #print(M_dcable_dTheta)
    #print(InvM_dcable_dTheta)
    divided_cable_deltas = 1/3*TotalCableDeltas
    #print("Divided Cable Displacements per Notch: ",divided_cable_deltas)
    NotchAngles = InvM_dcable_dTheta@divided_cable_deltas
    #print(NotchAngles)
    NotchAngles[NotchAngles<0] = 0
    #print(NotchAngles)
    return NotchAngles

def get_NotchAngle_from_CableDelta(h, y_, r ,deltaL_inner):
    """
    DEPRECATED
    gets notch angle from displacement of innermost cable of notch
    """
    curvature = deltaL_inner/(h*(r+y_) - deltaL_inner*y_)
    tube_midline = h/(1+y_*curvature)
    
    notch_angle = tube_midline*curvature
    return notch_angle

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_deltaCable_at_Notch(h, y_, r, w, notch_angle, phase):
    """
    gets cable displacements of all cables given the angle at a notch
    """
    if (notch_angle > 0):
        R = abs(h/notch_angle)
        K = 1/R
    
        L0 = np.sqrt(2*(R-y_-r)**2*(1-np.cos(notch_angle)))
        L1 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(notch_angle)))
        L2 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(notch_angle)))
    else:
        L0 = h
        L1 = h
        L2 = h
        #L0 = np.sqrt(2*(R-y_+r)**2*(1-np.cos(notch_angle)))
        #L1 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(notch_angle)))
        #L2 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(notch_angle)))
    
    if phase == "0":
        deltaCable_L0 = L0 - h
        deltaCable_L1 = L1 - h
        deltaCable_L2 = L2 - h
    elif phase == "120":
        deltaCable_L0 = L2 - h
        deltaCable_L1 = L0 - h
        deltaCable_L2 = L1 - h
    elif phase == "240":
        deltaCable_L0 = L1 - h
        deltaCable_L1 = L2 - h
        deltaCable_L2 = L0 - h
    
    #deltaCable = np.array([deltaCable_L0, deltaCable_L1, deltaCable_L2])

    return(np.array([deltaCable_L0, deltaCable_L1, deltaCable_L2]))


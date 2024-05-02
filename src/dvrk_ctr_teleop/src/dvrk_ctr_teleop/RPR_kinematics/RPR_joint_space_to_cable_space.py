#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_notch_pitch_angle_from_cable_delta_per_notch(h, y_, r, n , deltaL_inner):
    """
    gets notch angle from displacement of innermost cable of notch
    """
    curvature = deltaL_inner/(h*(r+y_) - deltaL_inner*y_)
    tube_midline = h/(1+y_*curvature)
    
    notch_angle = tube_midline*curvature
    return notch_angle

def get_pitch_angle_from_pitch_cable_delta(h, y_, r , n , deltaL):
    """
    gets wrist angle from displacement of wrist cable
    """
    deltaL_notch = deltaL/n
    notch_angle = get_notch_pitch_angle_from_cable_delta_per_notch(h,y_,r,n,deltaL_notch)
    wrist_angle = notch_angle*n
    return wrist_angle
    

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_notch_cable_delta_from_notch_pitch_angle(h, y_, r, w, n, notch_angle):
    """
    gets cable displacements of all cables given the angle at a notch
    """
    if (notch_angle > 0):
        R = abs(h/notch_angle)
        K = 1/R
        delta_pitch_cable = h - np.sqrt(2*(R-y_-r)**2*(1-np.cos(notch_angle)))

    else:
        delta_pitch_cable = 0

    return(delta_pitch_cable)

def get_pitch_cable_delta_from_wrist_pitch_angle(h, y_, r, w, n, pitch_angle):
    """
    gets wrist cable displacement given the wrist angle
    """
    notch_angle = pitch_angle/n
    deltaL_notch = get_notch_cable_delta_from_notch_pitch_angle(h,y_,r,w,n,notch_angle)
    total_cable_delta = deltaL_notch*n
    return total_cable_delta
    


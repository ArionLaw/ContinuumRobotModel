#from utils import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_PitchAngle_from_PitchCableDelta(h, y_, r ,deltaL_inner):
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

def get_deltaCable_at_Notch(h, y_, r, w, pitch_angle, phase):
    """
    gets cable displacements of all cables given the angle at a notch
    """
    if (pitch_angle > 0):
        R = abs(h/pitch_angle)
        K = 1/R
        delta_pitch_cable = np.sqrt(2*(R-y_-r)**2*(1-np.cos(pitch_angle)))

    else:
        delta_pitch_cable = h

    return(delta_pitch_cable)


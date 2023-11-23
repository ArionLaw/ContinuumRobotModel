#test_cases = True
test_cases = False
if test_cases == True:
    from utils import *
    from plotting import *

else:
    from dvrk_planning.kinematics.utils import *
    from dvrk_planning.kinematics.plotting import *

import numpy as np

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def allocate_deltaCables(deltaCables):
    """
    interprets total cable displacement and translates into the fraction of cable displacement for a single wrist segment 
    (3 notches 120deg out of phase)
    """
    return 1/3*deltaCables
    
def get_NotchAngle_from_CableDelta(h, y_, r ,deltaL_inner):
    """
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
    
    return(np.array([deltaCable_L0, deltaCable_L1, deltaCable_L2]))


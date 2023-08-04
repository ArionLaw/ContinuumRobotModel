from utils import *
from plotting import *
import numpy as np

def get_deltaCable_at_Notch(h, y_, r, w, notch_angle, phase):
    """
    Cable Displacement Calculation per individual notch
    """
    if (notch_angle > 0):
        R = abs(h/notch_angle)
        K = 1/R
    
        L1 = np.sqrt(2*(R-y_-r)**2*(1-np.cos(notch_angle)))
        L2 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(notch_angle)))
        L3 = np.sqrt(2*(R-y_+w)**2*(1-np.cos(notch_angle)))
    else:
        L1 = h
        L2 = h
        L3 = h
        #L1 = np.sqrt(2*(R-y_+r)**2*(1-np.cos(notch_angle)))
        #L2 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(notch_angle)))
        #L3 = np.sqrt(2*(R-y_-w)**2*(1-np.cos(notch_angle)))
    
    if phase == "0":
        deltaCable_L1 = L1 - h
        deltaCable_L2 = L2 - h
        deltaCable_L3 = L3 - h
    elif phase == "120":
        deltaCable_L1 = L3 - h
        deltaCable_L2 = L1 - h
        deltaCable_L3 = L2 - h
    elif phase == "240":
        deltaCable_L1 = L2 - h
        deltaCable_L2 = L3 - h
        deltaCable_L3 = L1 - h
    
    return(np.array([deltaCable_L1, deltaCable_L2, deltaCable_L3]))
from utils import *
from plotting import *
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
from JointSpace_to_CableSpace import *

#----------------------------------------------------------------------------------------------------------------------------------------------#
# wrist parameters
#----------------------------------------------------------------------------------------------------------------------------------------------#

n = 3 # sets of 3 cuts
h = 0.66 #mm notch height
c = 0.66 #mm notch spacing
prevStraightLength = 5 #mm
postStraightLength = 1 #mm
y_ = 0.56 #mm neutral bending plane
g = 1.16 #mm notch depth
OD = 1.37 #mm
ID = 0.94 #mm
r = OD/2
w = r*np.sin(np.radians(30))

shaft_length = 200 #mm

#----------------------------------------------------------------------------------------------------------------------------------------------#
### Mapping Initialization ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def getCabletoDiskMapping():
    """
    obtain mapping of Wiper Disk Angle vs Cable Displacement in the form of a lookup table
    """
    restingCableLength = 29 #mm
    diskLocation = 10 #mm
    wiperLength = 15 #mm
    wiperWidth = 6 #mm

    global mapping
    mapping = pd.DataFrame(columns = ['DiskAngle','DeltaCable'])
    lengthAC = restingCableLength
    theta = 0
    while theta < np.pi/3:
        positionB = [wiperLength*np.sin(theta) + wiperWidth/2*np.cos(theta) , wiperLength*np.cos(theta) + wiperWidth/2*np.sin(theta)]
        lengthAB = np.sqrt((positionB[0] - wiperWidth/2)**2 + (lengthAC-diskLocation-positionB[1])**2)
        lengthBC = np.sqrt((positionB[0] - wiperWidth/2)**2 + (diskLocation+positionB[1])**2)
        lengthDelta = lengthAB + lengthBC - lengthAC
        
        entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
        mapping = pd.concat([mapping,entry],ignore_index=True)
        
        theta = theta + np.pi/360
    return mapping

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def DiskToCablefromLookUpTable(find):
    """
    interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
    """
    mask_lower = mapping['DiskAngle'].lt(find)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DiskAngle'].gt(find)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['DeltaCable'].max()
    y2 = mask_upper['DeltaCable'].min()
    x1 = mask_lower['DiskAngle'].max()
    x2 = mask_upper['DiskAngle'].min()
    x = find
    y = y1 + (x-x1)*(y2-y1)/(x2-x1)
    return y

def DiskPosition_To_JointSpace(DiskPositions):
    roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix
    EE_grip = DiskPositions[1]/-1 #need to tune according to motion ratio

    if DiskPositions[3] < 0:
        gamma = 0
        beta = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
        alpha = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
    elif DiskPositions[2] > 0:
        gamma = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
        beta = 0
        alpha = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
    else:
        gamma = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
        beta = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
        alpha = 0

    wrist_cable_deltas = np.array([gamma,beta,alpha])
    #print("Total Wrist Cable Deltas: \n" , wrist_cable_deltas)
    segment_deltas = allocate_deltaCables(wrist_cable_deltas)
    #print("3 Notch Segment Cable Deltas: \n" , segment_deltas)
    gamma = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[0])
    beta = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[1])
    alpha = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[2])
    joint_values = [roll,EE_grip,gamma,beta,alpha]
    return joint_values


#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def CableToDiskfromLookUpTable(find):
    """
    interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
    """
    mask_lower = mapping['DeltaCable'].lt(find)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DeltaCable'].gt(find)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['DiskAngle'].max()
    y2 = mask_upper['DiskAngle'].min()
    x1 = mask_lower['DeltaCable'].max()
    x2 = mask_upper['DeltaCable'].min()
    x = find
    y = y1 + (x-x1)*(y2-y1)/(x2-x1)
    return y

def getDiskAngles(roll,EE_pull,deltaL1,deltaL2,deltaL3):
    """
    calculate Disk Angles from jointspace and cablespace inputs
    [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
    """
    deltaL = np.array([deltaL1,deltaL2,deltaL3])
    deltaL[deltaL<0] = 0 # not possible to extend length of cable, set to 0 displacement
    diff = (min(deltaL)) # set smallest cable displacement as reference length
    
    deltaL[deltaL<=diff] = 0
    #deltaL = deltaL - diff
    print("cables: \n", deltaL)

    if (deltaL[0] > 0):
        Disk4 = -CableToDiskfromLookUpTable(deltaL[0])
        if (deltaL[2] > 0):
            Disk3 = -CableToDiskfromLookUpTable(deltaL[2])
        else:
            Disk3 = CableToDiskfromLookUpTable(deltaL[1])
    elif (deltaL[1] > 0):
        Disk3 = CableToDiskfromLookUpTable(deltaL[1])
        if (deltaL[2] > 0):
            Disk4 = CableToDiskfromLookUpTable(deltaL[2])
        else:
            Disk4 = -CableToDiskfromLookUpTable(deltaL[0])

    Disk1 = -1.56323325*roll #from dVRK 8mm needle driver coupling matrix
    Disk2 = -1*EE_pull #need to tune according to motion ratio
    return [Disk1,Disk2,Disk3,Disk4]

#getCabletoDiskMapping()
#mapping.to_csv("cableToDiskMapping.csv",index=True)
#print(mapping)
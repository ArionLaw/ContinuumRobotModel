from utils import *
from plotting import *
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


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
    deltaL1 = deltaL[0] - diff
    deltaL2 = deltaL[1] - diff
    deltaL3 = deltaL[2] - diff
    #print("cables: \n", [deltaL1,deltaL2,deltaL3])

    if (deltaL1 > 0):
        Disk4 = -CableToDiskfromLookUpTable(deltaL1)
        if (deltaL3 > 0):
            Disk3 = -CableToDiskfromLookUpTable(deltaL3)
        else:
            Disk3 = CableToDiskfromLookUpTable(deltaL2)
    elif (deltaL2 > 0):
        Disk3 = CableToDiskfromLookUpTable(deltaL2)
        if (deltaL3 > 0):
            Disk4 = CableToDiskfromLookUpTable(deltaL3)
        else:
            Disk4 = -CableToDiskfromLookUpTable(deltaL1)

    Disk1 = -1.56323325*roll #from dVRK 8mm needle driver coupling matrix
    Disk2 = -1*EE_pull #need to tune according to motion ratio
    return [Disk1,Disk2,Disk3,Disk4]

#getCabletoDiskMapping()
#mapping.to_csv("cableToDiskMapping.csv",index=True)
#print(mapping)
from utils import *
from plotting import *
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd


def getCabletoDiskMapping():
    restingCableLength = 29 #mm
    diskLocation = 10 #mm
    wiperLength = 15 #mm
    wiperWidth = 6 #mm

    global mapping
    #mapping = np.array([[1,1]])
    mapping = pd.DataFrame(columns = ['DiskAngle','DeltaCable'])
    lengthAC = restingCableLength
    theta = 0
    while theta < np.pi/3:
        positionB = [wiperLength*np.sin(theta) + wiperWidth/2*np.cos(theta) , wiperLength*np.cos(theta) + wiperWidth/2*np.sin(theta)]
        lengthAB = np.sqrt((positionB[0] - wiperWidth/2)**2 + (lengthAC-diskLocation-positionB[1])**2)
        lengthBC = np.sqrt((positionB[0] - wiperWidth/2)**2 + (diskLocation+positionB[1])**2)
        lengthDelta = lengthAB + lengthBC - lengthAC
        
        #mapping = np.append(mapping, [[theta,lengthDelta]], axis=0)
        entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
        mapping = pd.concat([mapping,entry],ignore_index=True)
        
        theta = theta + np.pi/360
    #mapping = np.delete(mapping,0,0)
    return mapping

def CableToDiskfromLookUpTable(find):
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

def getDiskAngles(deltaL1,deltaL2,deltaL3):
    deltaL = np.array([deltaL1,deltaL2,deltaL3])
    deltaL[deltaL<0] = 0
    diff = (min(deltaL))
    deltaL1 = deltaL[0] - diff
    deltaL2 = deltaL[1] - diff
    deltaL3 = deltaL[2] - diff

    print("cables: \n", [deltaL1,deltaL2,deltaL3])

    if (deltaL1 > 0):
        Disk3 = CableToDiskfromLookUpTable(deltaL1)
        if (deltaL3 > 0):
            Disk4 = CableToDiskfromLookUpTable(deltaL3)
        else:
            Disk4 = CableToDiskfromLookUpTable(deltaL2)
    elif (deltaL2 > 0):
        Disk4 = CableToDiskfromLookUpTable(deltaL2)
        if (deltaL3 > 0):
            Disk3 = CableToDiskfromLookUpTable(deltaL3)
        else:
            Disk3 = CableToDiskfromLookUpTable(deltaL1)
    # check polarities
    # conditional checking
    return [Disk3, Disk4]

getCabletoDiskMapping()
#np.savetxt("cableToDiskMapping.csv", cableToDiskMapping ,delimiter=",")
mapping.to_csv("cableToDiskMapping.csv",index=True)
#print(mapping)

#diskAngle = CableToDiskfromLookUpTable(4.425786)
#print(diskAngle)

Disks3_4 = getDiskAngles(1,-2,3)
print("disks: \n", Disks3_4)
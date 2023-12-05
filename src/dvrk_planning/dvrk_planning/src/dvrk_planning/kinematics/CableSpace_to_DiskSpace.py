test_cases = True
#test_cases = False
if test_cases == True:
    from utils import *
    from plotting import *
    from JointSpace_to_CableSpace import *

else:
    from dvrk_planning.kinematics.utils import *
    from dvrk_planning.kinematics.plotting import *
    from dvrk_planning.kinematics.JointSpace_to_CableSpace import *

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

#----------------------------------------------------------------------------------------------------------------------------------------------#
### Mapping Initialization ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def getCabletoDiskMapping():
    """
    obtain mapping of Wiper Disk Angle vs Cable Displacement in the form of a lookup table
    """
    global mapping
    mapping = pd.DataFrame(columns = ['DiskAngle','DeltaCable'])

    restingCableLength = 29 #mm
    diskLocation = 10 #mm
    wiperLength = 15 #mm
    wiperWidth = 6 #mm

    lengthAC = restingCableLength
    theta = 0
    
    #calculating up to 90deg to prevent NaN crash error when interpolating for disk angles close to 60deg 
    #actual disks are tracking <30deg when experiencing crash, despite kinematics calculating disks at 60deg

    while theta < np.pi/2:
        positionB = [wiperLength*np.sin(theta) + wiperWidth/2*np.cos(theta) , wiperLength*np.cos(theta) + wiperWidth/2*np.sin(theta)]
        lengthAB = np.sqrt((positionB[0] - wiperWidth/2)**2 + (lengthAC-diskLocation-positionB[1])**2)
        lengthBC = np.sqrt((positionB[0] - wiperWidth/2)**2 + (diskLocation+positionB[1])**2)
        lengthDelta = lengthAB + lengthBC - lengthAC
        
        entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
        mapping = pd.concat([mapping,entry],ignore_index=True)
        
        theta = theta + np.pi/360
    return mapping

def calcEELinkageAnchorPos(thetaA):
    """
    perform positional calculation for EE linkage
    dimensions in mm
    A = dial spin axis
    B = gripper arm pivot
    C = gripper swing arm pivot
    D = mounting spin axis
    """
    L2 = 2 #gripper dial offset
    L3 = 23 #gripper arm length [dial pivot] to [swing arm pivot]
    L4 = 18 #gripper swing arm length [swing arm pivot] to [mounting pivot]
    L1 = 29.77 #distance from [gripper dial spin axis] to [mounting pivot spin axis]
    thetaAD = 50.77*np.pi/180 #in rads (50.77deg)

    xBtoAnchorX = 15
    yBtoAnchorY = 32
    GripperArmAngle = np.arctan(xBtoAnchorX,yBtoAnchorY)
    LGripperArm = np.sqrt(xBtoAnchorX**2 + yBtoAnchorY**2)

    angleBAD = abs(thetaA - thetaAD)
    LengthBD = np.sqrt(L1**2 + L2**2 - 2*L1*L2*np.cos(angleBAD))
    angleADB = np.arccos((L1**2 + LengthBD**2 - L2**2)/2*L1*LengthBD)
    angleCBD = np.arccos((L3**2 + LengthBD**2 - L4**2)/2*L3*LengthBD)
    #angleBDC = np.arccos((L4**2 + LengthBD**2 - L3**2)/2*L4*LengthBD)

    if (angleBAD >=0 ) and (angleBAD < np.pi):
        thetaB = angleCBD - (angleADB - thetaAD)
        #thetaC = np.pi - angleBDC - (angleADB - thetaAD)
    else: # pi < angleBAD <2pi or -pi < angleBAD < 0
        thetaB = angleCBD + (angleADB + thetaAD)
        #thetaC = np.pi - angleBDC + (angleADB + thetaAD)

    xB = L2*np.cos(thetaA)
    yB = L2*np.sin(thetaA)
    xPosAnchor = LGripperArm*np.cos(thetaB-GripperArmAngle) + xB
    yPosAnchor = LGripperArm*np.sin(thetaB-GripperArmAngle) + yB

    return xPosAnchor,yPosAnchor

def getEECabletoDisk2Mapping():
    """
    obtain mapping of Disk 2 Angle vs EE Cable Displacement in the form of a lookup table
    dimensions in mm    
    """
    xRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]
    yRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]

    global EEmapping
    EEmapping = pd.DataFrame(columns = ['Disk2Angle','DeltaEECable'])

    ### cable delta reference length
    thetaA = -10*np.pi/180 #-10deg cable delta reference location
    xAnchorRef,yAnchorRef = calcEELinkageAnchorPos(thetaA)
    xCableDeltaRef = xAnchorRef - xRef
    yCableDeltaRef = yAnchorRef - yRef
    
    ### calculating mapping
    thetaA = -np.pi/2
    while thetaA < np.pi/2:
        xAnchor,yAnchor = calcEELinkageAnchorPos(thetaA)
        xCableDelta = xAnchor - xRef
        yCableDelta = yAnchor - yRef

        if (yAnchor >= yAnchorRef):      
            lengthEEDelta = np.sqrt((xCableDelta-xCableDeltaRef)**2 + (yCableDelta-yCableDeltaRef)**2)
        else: #yAnchor < yAnchorRef
            lengthEEDelta = -np.sqrt((xCableDelta-xCableDeltaRef)**2 + (yCableDelta-yCableDeltaRef)**2)

        entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
        mapping = pd.concat([mapping,entry],ignore_index=True)
        
        thetaA = thetaA + np.pi/360
    return EEmapping

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def DiskToCablefromLookUpTable(x):
    """
    interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
    """
    mask_lower = mapping['DiskAngle'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DiskAngle'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['DeltaCable'].max()
    y2 = mask_upper['DeltaCable'].min()
    x1 = mask_lower['DiskAngle'].max()
    x2 = mask_upper['DiskAngle'].min()
    #print("[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))
    
def Disk2ToEECablefromLookUpTable(x):
    """
    interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
    """
    mask_lower = mapping['Disk2Angle'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['Disk2Angle'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['DeltaEECable'].max()
    y2 = mask_upper['DeltaEECable'].min()
    x1 = mask_lower['Disk2Angle'].max()
    x2 = mask_upper['Disk2Angle'].min()
    #print("[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))

def DiskPosition_To_JointSpace(DiskPositions,h,y_,r):
    roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix
    EE_jaw = Disk2ToEECablefromLookUpTable(DiskPositions[1]) #linear interpolation from EE gripper linkage mapping
    
    #print(DiskPositions)
    
    if DiskPositions[2] > 0:
        gamma = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
        #print("gamma")
        beta = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
        #print("beta")
        alpha = 0
        #print("unused\nalpha")
    elif DiskPositions[3] < 0:
        gamma = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
        #print("gamma")
        beta = 0
        #print("unused\nbeta")
        alpha = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
        #print("alpha")
    else:
        gamma = 0
        #print("gamma")
        beta = DiskToCablefromLookUpTable(abs(DiskPositions[3]))
        #print("beta")
        alpha = DiskToCablefromLookUpTable(abs(DiskPositions[2]))
        #print("alpha")
    #print("\n")

    wrist_cable_deltas = np.array([gamma,beta,alpha])
    #if printout is True: print("Total Wrist Cable Deltas: \n" , wrist_cable_deltas)
    segment_deltas = allocate_deltaCables(wrist_cable_deltas)
    #if printout is True: print("3 Notch Segment Cable Deltas: \n" , segment_deltas)
    gamma = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[0])
    beta = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[1])
    alpha = get_NotchAngle_from_CableDelta(h, y_, r, segment_deltas[2])
    joint_values = [roll,EE_jaw,gamma,beta,alpha]
    return joint_values

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def CableToDiskfromLookUpTable(x):
    """
    interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
    """
    if x > np.pi/3: x = np.pi/3 #disk value 90deg ceiling limit 

    mask_lower = mapping['DeltaCable'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DeltaCable'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['DiskAngle'].max()
    y2 = mask_upper['DiskAngle'].min()
    x1 = mask_lower['DeltaCable'].max()
    x2 = mask_upper['DeltaCable'].min()
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))
    
def EECableToDisk2fromLookUpTable(x):
    """
    interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
    """
    if x <= -np.pi/2: x = -np.pi/2 #disk value 90deg floor limit
    elif x >= np.pi/2: x = np.pi/2 #disk value 90deg ceiling limit 
    
    mask_lower = mapping['DeltaEECable'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DeltaEECable'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    y1 = mask_lower['Disk2Angle'].max()
    y2 = mask_upper['Disk2Angle'].min()
    x1 = mask_lower['DeltaEECable'].max()
    x2 = mask_upper['DeltaEECable'].min()
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))

def get_Disk_Angles(roll,EE_pinch,deltaL0,deltaL1,deltaL2):
    """
    calculate Disk Angles from jointspace and cablespace inputs
    [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
    """
    deltaL = np.array([deltaL0,deltaL1,deltaL2])
    deltaL[deltaL<0] = 0 # not possible to extend length of cable, set to 0 displacement
    diff = (min(deltaL)) # set smallest cable displacement as reference length
    
    #deltaL[deltaL<=diff] = 0
    deltaL = deltaL - diff
    print("Cables Delta: \n", deltaL)
    
    if (deltaL[1] > 0):
        Disk3 = -CableToDiskfromLookUpTable(deltaL[1])
        if (deltaL[0] > 0):
            Disk4 = -CableToDiskfromLookUpTable(deltaL[0])
        else:
            Disk4 = CableToDiskfromLookUpTable(deltaL[2])
    
    elif (deltaL[2] >= 0):
        Disk4 = CableToDiskfromLookUpTable(deltaL[2])
        if (deltaL[0] >= 0):
            Disk3 = CableToDiskfromLookUpTable(deltaL[0])
        else:
            Disk3 = -CableToDiskfromLookUpTable(deltaL[1])
    else:
        print("whooosh")    
    Disk1 = -1.56323325*roll #from dVRK 8mm needle driver coupling matrix
    Disk2 = EECableToDisk2fromLookUpTable(EE_pinch) #linear interpolation from EE gripper linkage mapping
    
    return [Disk1,Disk2,Disk3,Disk4]

#getCabletoDiskMapping()
#mapping.to_csv("cableToDiskMapping.csv",index=True)
#print(mapping)
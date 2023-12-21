from dvrk_ctr_teleop.kinematics.utils import *
from dvrk_ctr_teleop.kinematics.plotting import *
from dvrk_ctr_teleop.kinematics.joint_space_to_cable_space import *

import numpy as np
import pandas as pd
import sys
#----------------------------------------------------------------------------------------------------------------------------------------------#
### Mapping Initialization ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def getCabletoDiskMapping():
    """
    #obtain mapping of Wiper Disk Angle vs Cable Displacement in the form of a lookup table
    """
    global mapping
    mapping = pd.DataFrame(columns = ['DiskAngle','DeltaCable'])
    Approximation = True

    if Approximation == False:
        print("trig model for dial 3 & 4 mapping")
        restingCableLength = 29 #mm
        diskLocation = 10 #mm
        wiperLength = 15 #mm
        wiperWidth = 6 #mm
        lengthAC = restingCableLength
        theta = 0
        
        """ calculating mapping  up to 90deg to prevent NaN crash error when interpolating for disk angles close to 60deg """
        while theta < np.pi/2:
            positionB = [wiperLength*np.sin(theta) + wiperWidth/2*np.cos(theta) , wiperLength*np.cos(theta) + wiperWidth/2*np.sin(theta)]
            lengthAB = np.sqrt((positionB[0] - wiperWidth/2)**2 + (lengthAC-diskLocation-positionB[1])**2)
            lengthBC = np.sqrt((positionB[0] - wiperWidth/2)**2 + (diskLocation+positionB[1])**2)
            lengthDelta = lengthAB + lengthBC - lengthAC
            
            entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
            mapping = pd.concat([mapping,entry],ignore_index=True)
            
            theta = theta + np.pi/360
    else:
        print("sinusoid model for dial 3 & 4 mapping")
        vscale = 4.55 # value determined by mechanism
        hscale = 0.9 # tunable parameter
        vshift = 4.55 # value determined by mechanism
        hshift = -1.5 # leave this alone

        theta = 0
        while theta < np.pi/2:
            lengthDelta = vscale*np.sin(hscale*theta + hshift) + vshift        
            entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
            mapping = pd.concat([mapping,entry],ignore_index=True)
            
            theta = theta + np.pi/360

    return mapping

def calcEELinkageAnchorPos(thetaA):
    """
    #perform positional calculation for EE linkage
    #dimensions in mm
    #A = dial spin axis
    #B = gripper arm pivot
    #C = gripper swing arm pivot
    #D = mounting spin axis
    """

    """ linkage parameter definition """
    L2 = 2 #gripper dial offset
    L3 = 23 #gripper arm length [dial pivot] to [swing arm pivot]
    L4 = 18 #gripper swing arm length [swing arm pivot] to [mounting pivot]
    L1 = 29.77 #distance from [gripper dial spin axis] to [mounting pivot spin axis]
    thetaAD = 50.77*np.pi/180 #in rads (50.77deg)

    xBtoAnchorX = 15
    yBtoAnchorY = 32
    GripperArmAngle = np.arctan2(xBtoAnchorX,yBtoAnchorY)
    LGripperArm = np.sqrt(xBtoAnchorX**2 + yBtoAnchorY**2)

    """ linkage joint angle calculations """
    angleBAD = abs(thetaA - thetaAD)
    LengthBD = np.sqrt(L1**2 + L2**2 - 2*L1*L2*np.cos(angleBAD))
    angleADB = np.arccos((L1**2 + LengthBD**2 - L2**2)/(2*L1*LengthBD))
    angleCBD = np.arccos((L3**2 + LengthBD**2 - L4**2)/(2*L3*LengthBD))
    #print("angleBAD: ", angleBAD)
    #print("LengthBD: ", LengthBD)
    #print("angleADB: ", angleADB)
    #print("angleCBD: ", angleCBD)
    
    #angleBDC = np.arccos((L4**2 + LengthBD**2 - L3**2)/2*L4*LengthBD)

    if (angleBAD >=0 ) and (angleBAD < np.pi):
        thetaB = angleCBD - (angleADB - thetaAD)
        #print("thetaB (angleBAD from 0 to pi): ", thetaB)
        #thetaC = np.pi - angleBDC - (angleADB - thetaAD)
    else: # pi < angleBAD <2pi or -pi < angleBAD < 0
        thetaB = angleCBD + (angleADB + thetaAD)
        #print("thetaB (angleBAD from 0 to -pi): ", thetaB)
        #thetaC = np.pi - angleBDC + (angleADB + thetaAD)

    """ position of cable attachment point """
    xB = L2*np.cos(thetaA)
    yB = L2*np.sin(thetaA)
    xPosAnchor = LGripperArm*np.cos(thetaB-GripperArmAngle) + xB
    yPosAnchor = LGripperArm*np.sin(thetaB-GripperArmAngle) + yB

    return xPosAnchor,yPosAnchor

def getEECabletoDisk2Mapping():
    """
    #obtain mapping of Disk 2 Angle vs EE Cable Displacement in the form of a lookup table
    #dimensions in mm    
    """
    global EEmapping
    EEmapping = pd.DataFrame(columns = ['Disk2Angle','DeltaEECable'])
    Approximation = True
    
    if Approximation == False:
        print("linkage kinematics model for dial 2 mapping")
        xRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]
        yRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]

        ### cable zero displacement reference length
        thetaA = 0*np.pi/180 #radians disk2 position for zero cable displacement (configuration with no wrist angle and fully open jaws)
        xAnchorRef,yAnchorRef = calcEELinkageAnchorPos(thetaA)
        xCableDeltaRef = xAnchorRef - xRef
        yCableDeltaRef = yAnchorRef - yRef
        
        """ calculating mapping """
        thetaA = -np.pi*3/4
        while thetaA < np.pi*3/4:
            xAnchor,yAnchor = calcEELinkageAnchorPos(-thetaA)
            xCableDelta = xAnchor - xRef
            yCableDelta = yAnchor - yRef

            if (yAnchor >= yAnchorRef):      
                lengthEEDelta = np.sqrt((xCableDelta-xCableDeltaRef)**2 + (yCableDelta-yCableDeltaRef)**2)
            else: #yAnchor < yAnchorRef
                lengthEEDelta = -np.sqrt((xCableDelta-xCableDeltaRef)**2 + (yCableDelta-yCableDeltaRef)**2)

            entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
            EEmapping = pd.concat([EEmapping,entry],ignore_index=True)
            
            thetaA = thetaA + np.pi/360
    else:
        
        zero_setpoint = 32.5*np.pi/180 #radians zero 
        breakover = -70*np.pi/180 #radians breakover distance
        arccos_model = True
        thetaA = -np.pi/2

        if arccos_model == True:
            print("simple sinusoidal model for dial 2 mapping")  
            vscale = 1.7 #1.6
            hscale = 1 #1.1
            vshift = -1.2 #-0.6
            hshift = -0.1
            Max = 4.1
            Min = 0
            while thetaA < np.pi/2:
                lengthEEDelta = vscale*np.arccos(hscale*thetaA + hshift) + vshift
                if lengthEEDelta < Min: lengthEEDelta = Min
                if lengthEEDelta > Max: lengthEEDelta = Max
                entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
                EEmapping = pd.concat([EEmapping,entry],ignore_index=True)
                thetaA = thetaA + np.pi/360

        else:
            print("piecewise linear model for dial 2 mapping")
            MaxDelta = 4.1
            MinDelta = 0
            vshift = 0 
            hshift = -0.9
            scale = (MinDelta-MaxDelta)/(-hshift - breakover)
            while thetaA < np.pi/2: 
                if (thetaA <= breakover): #max displacement
                    lengthEEDelta = MaxDelta
                elif (thetaA > breakover and thetaA < -(hshift)): #linear range
                    lengthEEDelta = scale*(thetaA + hshift) + vshift
                else: # zero displacement
                    lengthEEDelta = MinDelta
                entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
                EEmapping = pd.concat([EEmapping,entry],ignore_index=True)
                thetaA = thetaA + np.pi/360

    return EEmapping    

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def Disk_to_Cable_from_LookUpTable(x):
    """
    interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
    """
    mask_lower = mapping['DiskAngle'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DiskAngle'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    
    x1 = mask_lower['DiskAngle'].max()
    x2 = mask_upper['DiskAngle'].min()
    if mask_lower['DeltaCable'].max() < mask_upper['DeltaCable'].min(): # y1 < y2 increasing function
        y1 = mask_lower['DeltaCable'].max()
        y2 = mask_upper['DeltaCable'].min()
    else: # y1 > y2 decreasing function
        y1 = mask_lower['DeltaCable'].min()
        y2 = mask_upper['DeltaCable'].max()
    
    #print("dial3/4[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))
    
def Disk2_to_EECable_from_LookUpTable(x):
    """
    interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
    """
    mask_lower = EEmapping['Disk2Angle'].lt(x)
    mask_lower = EEmapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = EEmapping['Disk2Angle'].gt(x)
    mask_upper = EEmapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)
    
    x1 = mask_lower['Disk2Angle'].max()
    x2 = mask_upper['Disk2Angle'].min()
    if mask_lower['DeltaEECable'].max() < mask_upper['DeltaEECable'].min(): # y1 < y2 increasing function
        y1 = mask_lower['DeltaEECable'].max()
        y2 = mask_upper['DeltaEECable'].min()
    else: # y1 > y2 decreasing function
        y1 = mask_lower['DeltaEECable'].min()
        y2 = mask_upper['DeltaEECable'].max()
    
    #print("dial3/4[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        return (y1 + (x-x1)*(y2-y1)/(x2-x1))
    
def EECable_to_GripperAngle(TotalCableDelta,WristBendingCableDelta):
    """
    calculation for relationship between EE gripper angle and EE cable delta 
    """
    R = 1.4 #mm EE scissor linkage arm lengths
    Max_EE_pinch_angle = 45*np.pi/180  #max EE jaw angle
    Min_EE_linkage_length = 2*R*np.cos(Max_EE_pinch_angle) #length of scissor linkage when EE fully open
    Max_EE_linkage_length = 2*R #length of scissor linkage when EE fully closed

    EECableDelta = TotalCableDelta - WristBendingCableDelta #component of EE cable delta responsible for actuation
    if EECableDelta < 0: EECableDelta = 0 #actuation cable unable to apply compressive loads 
    linkage_length = Min_EE_linkage_length + EECableDelta #length of actuated scissor linkage
    
    if linkage_length <= Max_EE_linkage_length: # for EE pinch angles 0deg and greater
        EE_pinch_angle = np.arccos(linkage_length/2/R)
    else: # for EE pinch angles < 0deg when applying gripping force
        Equivalent_Length = Max_EE_linkage_length - (linkage_length - Max_EE_linkage_length)
        EE_pinch_angle = -abs(np.arccos(((Equivalent_Length)/2/R))) #excess cable delta for actuation past 0 deg results in linkage_length/2/R > 1 
        # domain of arccos func. is from -1 to 1 // addition of -1 accounts for spill over
        # -abs(arccos) ensures negative value which should always be the case due to if else condition checking
    return EE_pinch_angle

def DiskPosition_To_JointSpace(DiskPositions,h,y_,r):
    roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix    
    #print(DiskPositions)
    
    if DiskPositions[2] > 0:
        gamma = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[2]))
        #print("gamma")
        beta = 0
        #print("beta")
        alpha = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[3]))
        #print("unused\nalpha")
    elif DiskPositions[3] < 0:
        gamma = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[3]))
        #print("gamma")
        beta = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[2]))
        #print("unused\nbeta")
        alpha = 0
        #print("alpha")
    else:
        gamma = 0
        #print("gamma")
        beta = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[2]))
        #print("beta")
        alpha = Disk_to_Cable_from_LookUpTable(abs(DiskPositions[3]))
        #print("alpha")
    #print("\n")

    wrist_cable_deltas = np.array([gamma,beta,alpha])
    #print("Total Wrist Cable Deltas: \n [gamma,beta,alpha]\n" , wrist_cable_deltas)
    EECableWristComponent = max(wrist_cable_deltas)
    EECableDelta = Disk2_to_EECable_from_LookUpTable(DiskPositions[1])
    #print("EECableDelta: ", EECableDelta)
    EE_jaw = EECable_to_GripperAngle(EECableDelta,EECableWristComponent) #linear interpolation from EE gripper linkage mapping
    WristJointAngles = get_NotchAngle_from_TotalCableDeltas(h,wrist_cable_deltas)
    #gamma = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[0])
    #beta = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[1])
    #alpha = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[2])
    joint_values = [roll,EE_jaw,WristJointAngles[0],WristJointAngles[1],WristJointAngles[2]]
    return joint_values

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def Cable_to_Disk_from_LookUpTable(x):
    """
    interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
    """
    mask_lower = mapping['DeltaCable'].lt(x)
    mask_lower = mapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = mapping['DeltaCable'].gt(x)
    mask_upper = mapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)

    x1 = mask_lower['DeltaCable'].max()
    x2 = mask_upper['DeltaCable'].min()
    if mask_lower['DiskAngle'].max() < mask_upper['DiskAngle'].min(): # y1 < y2 increasing function
        y1 = mask_lower['DiskAngle'].max()
        y2 = mask_upper['DiskAngle'].min()
    else: # y1 > y2 decreasing function
        y1 = mask_lower['DiskAngle'].min()
        y2 = mask_upper['DiskAngle'].max()
    
    #print("dial3/4[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
        #if y > np.pi/3: y = np.pi/3 #disk value 90deg ceiling limit 
        return y
    
def EECable_to_Disk2_from_LookUpTable(x):
    """
    interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
    """    
    mask_lower = EEmapping['DeltaEECable'].lt(x)
    mask_lower = EEmapping.loc[mask_lower]
    #print("lower: \n" , mask_lower)
    mask_upper = EEmapping['DeltaEECable'].gt(x)
    mask_upper = EEmapping.loc[mask_upper]
    #print("upper: \n" , mask_upper)

    x1 = mask_lower['DeltaEECable'].max()
    x2 = mask_upper['DeltaEECable'].min()
    if mask_lower['Disk2Angle'].max() < mask_upper['Disk2Angle'].min(): # y1 < y2 increasing function
        y1 = mask_lower['Disk2Angle'].max()
        y2 = mask_upper['Disk2Angle'].min()
    else: # y1 > y2 decreasing function
        y1 = mask_lower['Disk2Angle'].min()
        y2 = mask_upper['Disk2Angle'].max()

    #print("dial2[x1,x2,y1,y2]",[x1,x2,y1,y2])
    if np.isnan(x1):
        return 0
    else:
        y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
        #if y <= -np.pi/2: y = -np.pi/2 #disk value 90deg floor limit
        #elif y >= np.pi/2: y = np.pi/2 #disk value 90deg ceiling limit 
        return y
    
def GripperAngle_to_EECable(EE_pinch_angle,WristBendingCableDelta):
    """
    calculation for relationship between EE gripper angle and EE cable delta 
    """
    R = 1.4 #mm EE scissor linkage arm lengths
    Max_EE_pinch_angle = 45*np.pi/180  #max EE jaw angle
    Min_EE_pinch_angle = -10*np.pi/180  #min EE jaw angle

    if EE_pinch_angle > Max_EE_pinch_angle: EE_pinch_angle = Max_EE_pinch_angle #EE pinch angle joint limit, not able to open further
    elif EE_pinch_angle < Min_EE_pinch_angle: EE_pinch_angle = Min_EE_pinch_angle #EE pinch angle joint limit, cannot clamp further

    if EE_pinch_angle >= 0: #normal actuation range open to closed
        EECableDelta = 2*R*np.cos(EE_pinch_angle) - 2*R*np.cos(Max_EE_pinch_angle) # scissor linkage length - scissor linkage min length
    else: #clamping actuation range <0 deg for gripping
        EECableDelta = 2*R*np.cos(0) - 2*R*np.cos(Max_EE_pinch_angle) - 2*R*np.sin(EE_pinch_angle) # scissor linkage max length - scissor linkage min length + additional scissor linkage "inverted length"
    
    TotalCableDelta = EECableDelta + WristBendingCableDelta
    #print("EECableDelta: ", EECableDelta)
    #print("WristComponentCableDelta: ", WristBendingCableDelta)
    #print("TotalCableDelta: ", TotalCableDelta)
    return TotalCableDelta

def get_Disk_Angles(roll,EE_pinch_Angle,deltaL0,deltaL1,deltaL2):
    """
    calculate Disk Angles from jointspace and cablespace inputs
    [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
    """
    deltaL = np.array([deltaL0,deltaL1,deltaL2])
    deltaL[deltaL<0] = 0 # not possible to extend length of cable, set to 0 displacement
    ref = (min(deltaL)) # set smallest cable displacement as reference length
    
    deltaL[deltaL<= ref] = 0
    #deltaL = deltaL - diff
    #print("Cables Delta: \n", deltaL)
    
    if (deltaL[1] > 0):
        Disk3 = -Cable_to_Disk_from_LookUpTable(deltaL[1])
        if (deltaL[0] > 0):
            Disk4 = -Cable_to_Disk_from_LookUpTable(deltaL[0])
        else:
            Disk4 = Cable_to_Disk_from_LookUpTable(deltaL[2])
    
    elif (deltaL[2] >= 0):
        Disk4 = Cable_to_Disk_from_LookUpTable(deltaL[2])
        if (deltaL[0] >= 0):
            Disk3 = Cable_to_Disk_from_LookUpTable(deltaL[0])
        else:
            Disk3 = -Cable_to_Disk_from_LookUpTable(deltaL[1])
    else:
        print("whooosh")    
    Disk1 = -1.56323325*roll #from dVRK 8mm needle driver coupling matrix
    EECableWristComponent = max(deltaL)
    
    if EE_pinch_Angle is None:
        Disk2 = 0
    else:
        EECableDelta = GripperAngle_to_EECable(EE_pinch_Angle,EECableWristComponent)
        Disk2 = EECable_to_Disk2_from_LookUpTable(EECableDelta) #linear interpolation from EE gripper linkage mapping
    
    #print("D1,D2,D3,D4:\n", [Disk1, Disk2, Disk3, Disk4])

    # dial limits
    upperLimitDial_2 = 135*np.pi/180
    lowerLimitDial_2 = -90*np.pi/180
    if Disk2 > upperLimitDial_2: 
        Disk2 = upperLimitDial_2
        print("Disk2 upper limit")
    elif Disk2 < lowerLimitDial_2: 
        Disk2 = lowerLimitDial_2
        print("Disk2 lower limit")

    upperLimitDial_3_4 = 60*np.pi/180
    lowerLimitDial_3_4 = -60*np.pi/180
    if Disk3 > upperLimitDial_3_4: 
        Disk3 = upperLimitDial_3_4
        print("Disk3 upper limit")
    elif Disk3 < lowerLimitDial_3_4: 
        Disk3 = lowerLimitDial_3_4
        print("Disk3 lower limit")
    if Disk4 > upperLimitDial_3_4: 
        Disk4 = upperLimitDial_3_4
        print("Disk4 upper limit")
    elif Disk4 < lowerLimitDial_3_4: 
        Disk4 = lowerLimitDial_3_4
        print("Disk4 lower limit")
    
    return [Disk1,Disk2,Disk3,Disk4]

getCabletoDiskMapping()
getEECabletoDisk2Mapping()

file_path = sys.path[0]
file_path = file_path.replace('\src\dvrk_planning\dvrk_planning\src\dvrk_planning\kinematics','')
#print(file_path)
#mapping.to_csv(file_path +'/dialmapping.csv')
#EEmapping.to_csv(file_path +'/EEmapping.csv')

"""
roll = 0
EE_pinch_Angle = 0*np.pi/180
deltaL0 = 0 #0.66*3
deltaL1 = 0
deltaL2 = 0
print("EE jaw angle: " , EE_pinch_Angle/np.pi*180)
DiskAngles = get_Disk_Angles(roll,EE_pinch_Angle,deltaL0,deltaL1,deltaL2)
print("DiskAngles: ", DiskAngles)
"""

"""
h = 0.66 #mm notch height
y_ =  0.56 #mm neutral bending plane 
OD = 1.37 #mm
ID = 0.94 #mm
r = OD/2

Dial2 = [0,-0.1852856218497767,-0.2] # [45deg , 0 deg, -12.6deg]
DialAngles = [0,Dial2[2],0,0]
print("Dial2 position: ", DialAngles[1])
JointAngles = DiskPosition_To_JointSpace(DialAngles,h,y_,r)
print("jaw angle: " , JointAngles[1]*180/np.pi)
"""
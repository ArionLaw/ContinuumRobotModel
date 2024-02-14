#from utils import *
#from RPR_joint_space_to_cable_space import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_joint_space_to_cable_space import *

import numpy as np
import pandas as pd
import sys
#----------------------------------------------------------------------------------------------------------------------------------------------#
### Mapping Initialization ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
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
        arccos_model = False
        breakover = -70*np.pi/180 #radians breakover distance
        Max = 5
        Min = 0

        if arccos_model == True:
            print("simple sinusoidal model for dial 2 mapping")  
            vscale = 1.8 #1.7 #1.6
            hscale = 1 #1 #1.1
            vshift = -0.8 #-1.2 #-0.6
            hshift = 0.05 #-0.1
            
            thetaA = -np.pi/2
            while thetaA < np.pi/2:
                lengthEEDelta = vscale*np.arccos(hscale*thetaA + hshift) + vshift
                if lengthEEDelta < Min: lengthEEDelta = Min
                if lengthEEDelta > Max: lengthEEDelta = Max
                entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
                EEmapping = pd.concat([EEmapping,entry],ignore_index=True)
                thetaA = thetaA + np.pi/360

        else:
            print("piecewise linear model for dial 2 mapping")
            vshift = 0 
            hshift = -1
            scale = (Min-Max)/(-hshift - breakover)

            thetaA = -np.pi/2
            while thetaA < np.pi/2: 
                if (thetaA <= breakover): #max displacement
                    lengthEEDelta = Max
                elif (thetaA > breakover and thetaA < -(hshift)): #linear range
                    lengthEEDelta = scale*(thetaA + hshift) + vshift
                else: # zero displacement
                    lengthEEDelta = Min
                entry = pd.DataFrame([[thetaA,lengthEEDelta]] , columns = ['Disk2Angle','DeltaEECable'])
                EEmapping = pd.concat([EEmapping,entry],ignore_index=True)
                thetaA = thetaA + np.pi/360

    return EEmapping    

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#    
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
    inner_roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix    
    outer_roll = DiskPositions[2]/-1.56323325 #from dVRK 8mm needle driver coupling matrix
    SmallCapstanRadius = 15 #mm
    pitch_cable_delta = DiskPositions[1]*SmallCapstanRadius
    
    EECableWristComponent = pitch_cable_delta
    EECableDelta = Disk2_to_EECable_from_LookUpTable(DiskPositions[3])
    EE_jaw = EECable_to_GripperAngle(EECableDelta,EECableWristComponent) #linear interpolation from EE gripper linkage mapping
    
    pitch_angle = get_PitchAngle_from_PitchCableDelta(h, y_, r, pitch_cable_delta)

    joint_values = [outer_roll,pitch_angle,inner_roll,EE_jaw]
    return joint_values

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
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

def get_Disk_Angles(outer_roll,pitch_angle,inner_roll,EE_pinch_Angle,current_jaw_angle,h,y_,r,w):
    """
    calculate Disk Angles from jointspace and cablespace inputs
    [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
    """ 
    Disk1 = -1.56323325*outer_roll #from dVRK 8mm needle driver coupling matrix
    Disk3 = -1.56323325*inner_roll #from dVRK 8mm needle driver coupling matrix
    PitchCableDelta = get_PitchCableDelta_from_PitchAngle(h,y_,r,w,pitch_angle)
    SmallCapstanRadius = 15 #mm
    Disk2 = PitchCableDelta/SmallCapstanRadius
    EECableWristComponent = PitchCableDelta
    
    if EE_pinch_Angle is None:
        Disk4 = current_jaw_angle
    else:
        EECableDelta = GripperAngle_to_EECable(EE_pinch_Angle,EECableWristComponent)
        Disk4 = EECable_to_Disk2_from_LookUpTable(EECableDelta) #linear interpolation from EE gripper linkage mapping
    #print("D1,D2,D3,D4:\n", [Disk1, Disk2, Disk3, Disk4])
    # dial limits
    upperLimitDial_4 = 135*np.pi/180
    lowerLimitDial_4 = -90*np.pi/180
    if Disk4 > upperLimitDial_4: 
        Disk4 = upperLimitDial_4
        print("Disk4 upper limit")
    elif Disk4 < lowerLimitDial_4: 
        Disk4 = lowerLimitDial_4
        print("Disk4 lower limit")
    return [Disk1,Disk2,Disk3,Disk4]

#getCabletoDiskMapping()
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
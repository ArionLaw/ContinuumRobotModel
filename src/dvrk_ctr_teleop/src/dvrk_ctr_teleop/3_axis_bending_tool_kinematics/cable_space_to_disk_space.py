from dvrk_ctr_teleop.kinematics.utils import *
from dvrk_ctr_teleop.kinematics.plotting import *
from dvrk_ctr_teleop.kinematics.joint_space_to_cable_space import *

import numpy as np
import pandas as pd
import sys

class CableToDiskSpaceSolver:
    def __init__(self, config_yaml):
        self.cable_to_disk_map = self._get_cable_disk_mapping(config_yaml["cable_to_disk_map"])
        self.ee_cable_to_disk_map = self._get_ee_cable_to_disk_mapping(config_yaml["ee_cable_to_disk_map"])
        self.compensation_factor = config_yaml["wrist_bending_compensation"] #factor for considering cable slack take-up due to wrist bending 

        self.Max_EE_pinch_angle = 45*np.pi/180  #max EE jaw angle
        self.Min_EE_pinch_angle = -30*np.pi/180  #min EE jaw angle

        self.ee_R = 1.4 #mm EE scissor linkage arm lengths
        self.two_times_ee_R = 2 * self.ee_R
        self.Min_EE_linkage_length = self.two_times_ee_R * np.cos(self.Max_EE_pinch_angle) #length of scissor linkage when EE fully open
        self.Max_EE_linkage_length = self.two_times_ee_R #length of scissor linkage when EE fully closed

        self.dial_upper_limits = np.array([ 1000.0, 135.0*np.pi/180.0,  40.0*np.pi/180.0,  40.0*np.pi/180.0])
        self.dial_lower_limits = np.array([-1000.0, -90.0*np.pi/180.0, -40.0*np.pi/180.0, -40.0*np.pi/180.0])

    def _get_cable_disk_mapping(self, yaml):
        """
        #obtain mapping of Wiper Disk Angle vs Cable Displacement in the form of a lookup table
        """
        mapping = pd.DataFrame(columns = ['DiskAngle','DeltaCable'])
        # Approximation = True
        Approximation = yaml["approximation"]

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
            # vscale = 4.55 # value determined by mechanism
            # hscale = 0.9 # tunable parameter
            # vshift = 4.55 # value determined by mechanism
            # hshift = -1.5 # leave this alone

            vscale = yaml["vscale"] # value determined by mechanism
            hscale = yaml["hscale"] # tunable parameter
            vshift = yaml["vshift"] # value determined by mechanism
            hshift = yaml["hshift"] # leave this alone

            theta = 0
            while theta < np.pi/2:
                lengthDelta = vscale*np.sin(hscale*theta + hshift) + vshift        
                entry = pd.DataFrame([[theta,lengthDelta]] , columns = ['DiskAngle','DeltaCable'])
                mapping = pd.concat([mapping,entry],ignore_index=True)

                theta = theta + np.pi/360

        return mapping

    def _calculate_ee_linkage_anchor_pos(self, thetaA):
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

    def _get_ee_cable_to_disk_mapping(self, yaml):
        """
        #obtain mapping of Disk 2 Angle vs EE Cable Displacement in the form of a lookup table
        #dimensions in mm
        """
        EEmapping = pd.DataFrame(columns = ['Disk2Angle','DeltaEECable'])
        # Approximation = True
        Approximation = yaml["approximation"]

        if Approximation == False:
            print("linkage kinematics model for dial 2 mapping")
            xRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]
            yRef = 16.25 #distance in mm from [gripper dial spin axis] to [origin]

            ### cable zero displacement reference length
            thetaA = 0*np.pi/180 #radians disk2 position for zero cable displacement (configuration with no wrist angle and fully open jaws)
            xAnchorRef,yAnchorRef = self._calculate_ee_linkage_anchor_pos(thetaA)
            xCableDeltaRef = xAnchorRef - xRef
            yCableDeltaRef = yAnchorRef - yRef

            """ calculating mapping """
            thetaA = -np.pi*3/4
            while thetaA < np.pi*3/4:
                xAnchor,yAnchor = self._calculate_ee_linkage_anchor_pos(-thetaA)
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
            # arccos_model = False
            arccos_model = yaml["arccos_model"]
            breakover = yaml["breakover"]*np.pi/180 #radians breakover distance
            Max = yaml["max"]
            Min = yaml["min"]

            if arccos_model == True:
                print("simple sinusoidal model for dial 2 mapping")  
                # vscale = 1.8 #1.7 #1.6
                # hscale = 1 #1 #1.1
                # vshift = -0.8 #-1.2 #-0.6
                # hshift = 0.05 #-0.1

                vscale = yaml["vscale"]
                hscale = yaml["hscale"]
                vshift = yaml["vshift"]
                hshift = yaml["hshift"]

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
                vshift = yaml["piecewise_vshift"]
                hshift = yaml["piecewise_hshift"]*np.pi/180
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

    def _disk_to_cable_from_lookup_table(self, x):
        """
        interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
        """
        mask_lower = self.cable_to_disk_map['DiskAngle'].lt(x)
        mask_lower = self.cable_to_disk_map.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.cable_to_disk_map['DiskAngle'].gt(x)
        mask_upper = self.cable_to_disk_map.loc[mask_upper]
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

    def _disk_to_ee_cable_from_lookup_table(self, x):
        """
        interpolation of Cable Displacement Output from Wiper Disk Angle input from lookup table
        """
        mask_lower = self.ee_cable_to_disk_map['Disk2Angle'].lt(x)
        mask_lower = self.ee_cable_to_disk_map.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.ee_cable_to_disk_map['Disk2Angle'].gt(x)
        mask_upper = self.ee_cable_to_disk_map.loc[mask_upper]
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

    def _ee_cable_to_gripper_angle(self, TotalCableDelta, WristBendingCableDelta):
        """
        calculation for relationship between EE gripper angle and EE cable delta 
        """

        EECableDelta = TotalCableDelta - WristBendingCableDelta * self.compensation_factor #component of EE cable delta responsible for actuation
        if EECableDelta < 0: EECableDelta = 0 #actuation cable unable to apply compressive loads 
        linkage_length = self.Min_EE_linkage_length + EECableDelta #length of actuated scissor linkage

        if linkage_length <= self.Max_EE_linkage_length: # for EE pinch angles 0deg and greater
            EE_pinch_angle = np.arccos(linkage_length/2/self.ee_R)
        else: # for EE pinch angles < 0deg when applying gripping force
            Equivalent_Length = self.Max_EE_linkage_length - (linkage_length - self.Max_EE_linkage_length)
            EE_pinch_angle = -abs(np.arccos(((Equivalent_Length)/2/self.ee_R))) #excess cable delta for actuation past 0 deg results in linkage_length/2/R > 1 
            # domain of arccos func. is from -1 to 1 // addition of -1 accounts for spill over
            # -abs(arccos) ensures negative value which should always be the case due to if else condition checking
        return EE_pinch_angle

    def disk_position_to_joint_space(self, DiskPositions, h, y_, r):
        roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix    
        #print(DiskPositions)

        if DiskPositions[2] > 0:
            gamma = self._disk_to_cable_from_lookup_table(abs(DiskPositions[2]))
            #print("gamma")
            beta = 0
            #print("beta")
            alpha = self._disk_to_cable_from_lookup_table(abs(DiskPositions[3]))
            #print("unused\nalpha")
        elif DiskPositions[3] < 0:
            gamma = self._disk_to_cable_from_lookup_table(abs(DiskPositions[3]))
            #print("gamma")
            beta = self._disk_to_cable_from_lookup_table(abs(DiskPositions[2]))
            #print("unused\nbeta")
            alpha = 0
            #print("alpha")
        else:
            gamma = 0
            #print("gamma")
            beta = self._disk_to_cable_from_lookup_table(abs(DiskPositions[2]))
            #print("beta")
            alpha = self._disk_to_cable_from_lookup_table(abs(DiskPositions[3]))
            #print("alpha")
        #print("\n")

        wrist_cable_deltas = np.array([gamma,beta,alpha])
        #print("Total Wrist Cable Deltas: \n [gamma,beta,alpha]\n" , wrist_cable_deltas)
        EECableWristComponent = max(wrist_cable_deltas)
        EECableDelta = self._disk_to_ee_cable_from_lookup_table(DiskPositions[1])
        #print("EECableDelta: ", EECableDelta)
        EE_jaw = self._ee_cable_to_gripper_angle(EECableDelta, EECableWristComponent) #linear interpolation from EE gripper linkage mapping
        WristJointAngles = get_NotchAngle_from_TotalCableDeltas(h,wrist_cable_deltas)
        #gamma = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[0])
        #beta = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[1])
        #alpha = get_NotchAngle_from_CableDelta(h, y_, r, allocated_cable_deltas[2])
        joint_values = [roll,EE_jaw,WristJointAngles[0],WristJointAngles[1],WristJointAngles[2]]
        return joint_values

    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### IK ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#

    def _cable_to_disk_from_lookup_table(self, x):
        """
        interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
        """
        mask_lower = self.cable_to_disk_map['DeltaCable'].lt(x)
        mask_lower = self.cable_to_disk_map.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.cable_to_disk_map['DeltaCable'].gt(x)
        mask_upper = self.cable_to_disk_map.loc[mask_upper]
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
        
    def _ee_cable_to_disk2_from_lookup_table(self, x):
        """
        interpolation of Wiper Disk Angle output from Cable Displacement input from lookup table
        """
        mask_lower = self.ee_cable_to_disk_map['DeltaEECable'].lt(x)
        mask_lower = self.ee_cable_to_disk_map.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.ee_cable_to_disk_map['DeltaEECable'].gt(x)
        mask_upper = self.ee_cable_to_disk_map.loc[mask_upper]
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

    def _gripper_angle_to_ee_cable(self, EE_pinch_angle, WristBendingCableDelta):
        """
        calculation for relationship between EE gripper angle and EE cable delta 
        """

        if EE_pinch_angle > self.Max_EE_pinch_angle: EE_pinch_angle = self.Max_EE_pinch_angle #EE pinch angle joint limit, not able to open further
        elif EE_pinch_angle < self.Min_EE_pinch_angle: EE_pinch_angle = self.Min_EE_pinch_angle #EE pinch angle joint limit, cannot clamp further

        if EE_pinch_angle >= 0: #normal actuation range open to closed
            EECableDelta = self.two_times_ee_R*np.cos(EE_pinch_angle) - self.Min_EE_linkage_length # scissor linkage length - scissor linkage min length
        else: #clamping actuation range <0 deg for gripping
            EECableDelta = self.two_times_ee_R - self.Min_EE_linkage_length - self.two_times_ee_R*np.sin(EE_pinch_angle) # scissor linkage max length - scissor linkage min length + additional scissor linkage "inverted length"

        TotalCableDelta = EECableDelta + WristBendingCableDelta * self.compensation_factor
        #print("EECableDelta: ", EECableDelta)
        #print("WristComponentCableDelta: ", WristBendingCableDelta)
        #print("TotalCableDelta: ", TotalCableDelta)
        return TotalCableDelta

    def get_disk_angles(self, roll, EE_pinch_Angle, deltaL0, deltaL1, deltaL2, current_jaw_angle):
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

        # disk 1 from dVRK 8mm needle driver coupling matrix
        disk_angles_rad = np.array([-1.56323325*roll, 0, 0, 0], dtype=float)
        if (deltaL[1] > 0):
            disk_angles_rad[2] = -self._cable_to_disk_from_lookup_table(deltaL[1])
            if (deltaL[0] > 0):
                disk_angles_rad[3] = -self._cable_to_disk_from_lookup_table(deltaL[0])
            else:
                disk_angles_rad[3] = self._cable_to_disk_from_lookup_table(deltaL[2])

        elif (deltaL[2] >= 0):
            disk_angles_rad[3] = self._cable_to_disk_from_lookup_table(deltaL[2])
            if (deltaL[0] >= 0):
                disk_angles_rad[2] = self._cable_to_disk_from_lookup_table(deltaL[0])
            else:
                disk_angles_rad[2] = -self._cable_to_disk_from_lookup_table(deltaL[1])
        else:
            print("whooosh")

        if EE_pinch_Angle is None:
            disk_angles_rad[1] = current_jaw_angle
        else:
            EECableWristComponent = max(deltaL)
            EECableDelta = self._gripper_angle_to_ee_cable(EE_pinch_Angle, EECableWristComponent)
            disk_angles_rad[1] = self._ee_cable_to_disk2_from_lookup_table(EECableDelta) #linear interpolation from EE gripper linkage mapping
        #print("D1,D2,D3,D4:\n", [Disk1, Disk2, Disk3, Disk4])
        disk_angles_rad = np.clip(disk_angles_rad, self.dial_lower_limits, self.dial_upper_limits)
        return disk_angles_rad

"""
roll = 0
EE_pinch_Angle = 0*np.pi/180
deltaL0 = 0 #0.66*3
deltaL1 = 0
deltaL2 = 0
print("EE jaw angle: " , EE_pinch_Angle/np.pi*180)
DiskAngles = get_disk_angles(roll,EE_pinch_Angle,deltaL0,deltaL1,deltaL2)
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
JointAngles = disk_position_to_joint_space(DialAngles,h,y_,r)
print("jaw angle: " , JointAngles[1]*180/np.pi)
"""
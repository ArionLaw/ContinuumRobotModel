#from utils import *
#from RPR_joint_space_to_cable_space import *

from dvrk_ctr_teleop.RPR_kinematics.utils import *
from dvrk_ctr_teleop.RPR_kinematics.RPR_joint_space_to_cable_space import *

import numpy as np
import pandas as pd
import sys
import os
import pdb

class CableToDiskSpaceSolver:
    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### Mapping Initialization ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#
    def __init__(self, config_yaml):
        self.compensation_factor = config_yaml["wrist_bending_compensation"] #factor for considering cable slack take-up due to wrist bending

        self.SmallCapstanDiameter = config_yaml["small_capstan_diameter"] #5.08 #mm
        self.SmallCapstanRadius = self.SmallCapstanDiameter/2
    
        self.Max_EE_pinch_angle = config_yaml["max_EE_pinch_angle_deg"]*np.pi/180  #max EE jaw angle
        self.Min_EE_pinch_angle = config_yaml["min_EE_pinch_angle_deg"]*np.pi/180  #min EE jaw angle
    
        self.ee_R = config_yaml["gripper_scissor_linkage_length"] #1.4mm EE scissor linkage arm lengths
        self.two_times_ee_R = 2 * self.ee_R
        self.Min_EE_linkage_length = self.two_times_ee_R * np.cos(self.Max_EE_pinch_angle) #length of scissor linkage when EE fully open
        self.Max_EE_linkage_length = self.two_times_ee_R #length of scissor linkage when EE fully closed

        self.dial_upper_limits = np.array([ 1000.0  ,  0                ,  1000.0   ,  0                ])
        self.dial_lower_limits = np.array([-1000.0  , -75.0*np.pi/180.0 , -1000.0   , -90.0*np.pi/180.0 ])

        self.excel_file_dir = "src/dvrk_ctr_teleop/src/dvrk_ctr_teleop/RPR_kinematics"
        self.getEECabletoDisk4Mapping()
        self.getWristAngletoWristCableDeltaMapping()
    
    def getEECabletoDisk4Mapping(self):
        
        """
        #obtain mapping of Disk 4 Angle vs EE Cable Displacement in the form of a lookup table
        #dimensions in mm    
        """
        file_name = 'EE_Linkage_Mapping.xlsx'
        file_name = os.path.join(os.getcwd(),self.excel_file_dir,file_name)
        self.EEmapping = pd.read_excel( file_name, usecols = ['Disk4Angle','DeltaEECable'])
        print(self.EEmapping)
        return self.EEmapping

    def getWristAngletoWristCableDeltaMapping(self):
        """
        #obtain mapping of Wrist Cable Delta vs Wrist Angle in the form of a lookup table
        #dimensions in mm    
        """
        file_name = 'Wrist_Bending_Mapping.xlsx'
        file_name = os.path.join(os.getcwd(),self.excel_file_dir,file_name)
        self.WristMapping = pd.read_excel(file_name, usecols = ['WristCableDelta','WristAngle'])
        print(self.WristMapping)
        return self.WristMapping

    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### FK ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#    
    def Disk4_to_EECable_from_LookUpTable(self,x):
        """
        linear interpolation of Cable Displacement Output from Disk Angle input from lookup table
        """
        mask_lower = self.EEmapping['Disk4Angle'].lt(x)
        mask_lower = self.EEmapping.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.EEmapping['Disk4Angle'].gt(x)
        mask_upper = self.EEmapping.loc[mask_upper]
        #print("upper: \n" , mask_upper)
        
        x1 = mask_lower['Disk4Angle'].max()
        x2 = mask_upper['Disk4Angle'].min()
        if mask_lower['DeltaEECable'].max() < mask_upper['DeltaEECable'].min(): # y1 < y2 increasing function
            y1 = mask_lower['DeltaEECable'].max()
            y2 = mask_upper['DeltaEECable'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['DeltaEECable'].min()
            y2 = mask_upper['DeltaEECable'].max()
        
        #print("dial4[x1,x2,y1,y2]",[x1,x2,y1,y2])
        if np.isnan(x1):
            return 0
        else:
            return (y1 + (x-x1)*(y2-y1)/(x2-x1))
        
    def WristCableDelta_to_WristAngle_from_LookUpTable(self,x):
        """
        linear interpolation of Wrist Angle from Disk Angle input from lookup table
        """
        mask_lower = self.WristMapping['WristCableDelta'].lt(x)
        mask_lower = self.WristMapping.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.WristMapping['WristCableDelta'].gt(x)
        mask_upper = self.WristMapping.loc[mask_upper]
        #print("upper: \n" , mask_upper)
        
        x1 = mask_lower['WristCableDelta'].max()
        x2 = mask_upper['WristCableDelta'].min()
        if mask_lower['WristAngle'].max() < mask_upper['WristAngle'].min(): # y1 < y2 increasing function
            y1 = mask_lower['WristAngle'].max()
            y2 = mask_upper['WristAngle'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['WristAngle'].min()
            y2 = mask_upper['WristAngle'].max()
        
        #print("dial2[x1,x2,y1,y2]",[x1,x2,y1,y2])
        if np.isnan(x1):
            return 0
        else:
            return (y1 + (x-x1)*(y2-y1)/(x2-x1))
        
    def EECable_to_GripperAngle(self,TotalCableDelta,WristBendingCableDelta):    
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

    def DiskPosition_To_JointSpace(self,DiskPositions,h,y_,r,n):
        inner_roll = DiskPositions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix    
        outer_roll = DiskPositions[2]/1.56323325 #from dVRK 8mm needle driver coupling matrix
        
        pitch_cable_delta = DiskPositions[1]*self.SmallCapstanRadius #calc pitch cable displacement
        #pitch_angle = get_PitchAngle_from_PitchCableDelta(h, y_, r, n, pitch_cable_delta)
        pitch_angle = self.WristCableDelta_to_WristAngle_from_LookUpTable(pitch_cable_delta)

        EECableWristComponent = -pitch_cable_delta #component of pitch compensation in EE cable displacement
        EECableDelta = self.Disk4_to_EECable_from_LookUpTable(DiskPositions[3]) #calc total EE cable displacement (linear interpolation from EE gripper linkage mapping)
        EE_jaw = self.EECable_to_GripperAngle(EECableDelta,EECableWristComponent) #calc component of total EE cable displacement towards EE actuation 
        
        joint_values = [outer_roll,pitch_angle,inner_roll,EE_jaw]
        return joint_values

    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### IK ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#
    def EECable_to_Disk4_from_LookUpTable(self,x):
        """
        linear interpolation of Disk 4 Angle output from EE Cable Displacement input from lookup table
        """    
        mask_lower = self.EEmapping['DeltaEECable'].lt(x)
        mask_lower = self.EEmapping.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.EEmapping['DeltaEECable'].gt(x)
        mask_upper = self.EEmapping.loc[mask_upper]
        #print("upper: \n" , mask_upper)

        x1 = mask_lower['DeltaEECable'].max()
        x2 = mask_upper['DeltaEECable'].min()
        if mask_lower['Disk4Angle'].max() < mask_upper['Disk4Angle'].min(): # y1 < y2 increasing function
            y1 = mask_lower['Disk4Angle'].max()
            y2 = mask_upper['Disk4Angle'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['Disk4Angle'].min()
            y2 = mask_upper['Disk4Angle'].max()

        #print("x" , x)
        #print("dial4[x1,x2,y1,y2]",[x1,x2,y1,y2])
        if np.isnan(x1):
            return 0
        else:
            y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
            #if y <= -np.pi/2: y = -np.pi/2 #disk value 90deg floor limit
            #elif y >= np.pi/2: y = np.pi/2 #disk value 90deg ceiling limit 
            return y
    
    def WristAngle_to_WristCableDelta_from_LookUpTable(self,x):
        """
        linear interpolation of Wrist Cable Delta output from Wrist Angle input from lookup table
        """    
        mask_lower = self.WristMapping['WristAngle'].lt(x)
        mask_lower = self.WristMapping.loc[mask_lower]
        #print("lower: \n" , mask_lower)
        mask_upper = self.WristMapping['WristAngle'].gt(x)
        mask_upper = self.WristMapping.loc[mask_upper]
        #print("upper: \n" , mask_upper)

        x1 = mask_lower['WristAngle'].max()
        x2 = mask_upper['WristAngle'].min()
        if mask_lower['WristCableDelta'].max() < mask_upper['WristCableDelta'].min(): # y1 < y2 increasing function
            y1 = mask_lower['WristCableDelta'].max()
            y2 = mask_upper['WristCableDelta'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['WristCableDelta'].min()
            y2 = mask_upper['WristCableDelta'].max()

        #print("dial2[x1,x2,y1,y2]",[x1,x2,y1,y2])
        # if np.isnan(x1):
        #     return 0
        if np.isnan(np.array([x1, x2, y1, y2])).any():
            return 0
        else:
            y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
            #if y <= -np.pi/2: y = -np.pi/2 #disk value 90deg floor limit
            #elif y >= np.pi/2: y = np.pi/2 #disk value 90deg ceiling limit 
            return y
        
    def GripperAngle_to_EECable(self,EE_pinch_angle,WristBendingCableDelta):
        """
        calculation for relationship between EE gripper angle and EE cable delta 
        """
        #print("EE_pinch_angle" , EE_pinch_angle)    
        if EE_pinch_angle > self.Max_EE_pinch_angle: EE_pinch_angle = self.Max_EE_pinch_angle #EE pinch angle joint limit, not able to open further
        elif EE_pinch_angle < self.Min_EE_pinch_angle: EE_pinch_angle = self.Min_EE_pinch_angle #EE pinch angle joint limit, cannot clamp further
        print("EE_pinch_angle clipped" , EE_pinch_angle) 
        if EE_pinch_angle >= 0: #normal actuation range open to closed
            EECableDelta = self.two_times_ee_R*np.cos(EE_pinch_angle) - self.Min_EE_linkage_length # scissor linkage length - scissor linkage min length
        else: #clamping actuation range <0 deg for gripping
            EECableDelta = self.two_times_ee_R - self.Min_EE_linkage_length + (self.Max_EE_linkage_length - self.two_times_ee_R*np.cos(-EE_pinch_angle)) # scissor linkage max length - scissor linkage min length + additional scissor linkage "inverted length"

        TotalCableDelta = EECableDelta + WristBendingCableDelta * self.compensation_factor
        print("EECableDelta: ", EECableDelta)
        #print("WristComponentCableDelta: ", WristBendingCableDelta)
        print("TotalCableDelta: ", TotalCableDelta)
        return TotalCableDelta

    def get_Disk_Angles(self,outer_roll,pitch_angle,inner_roll,EE_pinch_Angle,current_jaw_angle,h,y_,r,w,n):
        """
        calculate Disk Angles from jointspace and cablespace inputs
        [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
        """ 

        disk_angles_rad = [-1.56323325*outer_roll , 0 , 1.56323325*inner_roll, 0] #1.56323325 coeff from dVRK 8mm needle driver coupling matrix

        #PitchCableDelta = get_PitchCableDelta_from_PitchAngle(h,y_,r,w,n,pitch_angle)
        PitchCableDelta = self.WristAngle_to_WristCableDelta_from_LookUpTable(pitch_angle)
        #print("PitchCableDelta", PitchCableDelta)
        disk_angles_rad[1] = PitchCableDelta/self.SmallCapstanRadius
        
        EECableWristComponent = -PitchCableDelta
        if EE_pinch_Angle is None:
            disk_angles_rad[3] = current_jaw_angle
        else:
            EECableDelta = self.GripperAngle_to_EECable(EE_pinch_Angle,EECableWristComponent)
            disk_angles_rad[3] = self.EECable_to_Disk4_from_LookUpTable(EECableDelta) #linear interpolation from EE gripper linkage mapping
        disk_angles_rad = np.clip(disk_angles_rad, self.dial_lower_limits, self.dial_upper_limits)
        return disk_angles_rad.tolist()
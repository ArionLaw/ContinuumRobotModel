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

        self.small_capstan_diameter = config_yaml["small_capstan_diameter"] #5.08 #mm
        self.small_capstan_radius = self.small_capstan_diameter/2
    
        self.max_ee_pinch_angle = config_yaml["max_EE_pinch_angle_deg"]*np.pi/180  #max EE jaw angle
        self.min_ee_pinch_angle = config_yaml["min_EE_pinch_angle_deg"]*np.pi/180  #min EE jaw angle
    
        self.ee_R = config_yaml["gripper_scissor_linkage_length"] #1.4mm EE scissor linkage arm lengths
        self.two_times_ee_R = 2 * self.ee_R
        self.min_ee_linkage_length = self.two_times_ee_R * np.cos(self.max_ee_pinch_angle) #length of scissor linkage when EE fully open
        self.max_ee_linkage_length = self.two_times_ee_R #length of scissor linkage when EE fully closed

        self.dial_upper_limits = np.array([ 1000.0  ,  0                ,  1000.0   ,  0                ])
        self.dial_lower_limits = np.array([-1000.0  , -75.0*np.pi/180.0 , -1000.0   , -90.0*np.pi/180.0 ])

        self.excel_file_dir = "src/dvrk_ctr_teleop/src/dvrk_ctr_teleop/RPR_kinematics"
        self.get_ee_cable_to_disk_4_mapping()
        self.get_wrist_angle_to_wrist_cable_delta_mapping()
    
    def get_ee_cable_to_disk_4_mapping(self):
        
        """
        #obtain mapping of Disk 4 Angle vs EE Cable Displacement in the form of a lookup table
        #dimensions in mm    
        """
        file_name = 'EE_Linkage_Mapping.xlsx'
        file_name = os.path.join(os.getcwd(),self.excel_file_dir,file_name)
        self.ee_mapping = pd.read_excel( file_name, usecols = ['Disk4Angle','DeltaEECable'])
        return self.ee_mapping

    def get_wrist_angle_to_wrist_cable_delta_mapping(self):
        """
        #obtain mapping of Wrist Cable Delta vs Wrist Angle in the form of a lookup table
        #dimensions in mm    
        """
        file_name = 'Wrist_Bending_Mapping.xlsx'
        file_name = os.path.join(os.getcwd(),self.excel_file_dir,file_name)
        self.wrist_mapping = pd.read_excel(file_name, usecols = ['WristCableDelta','WristAngle'])
        return self.wrist_mapping

    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### FK ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#    
    def disk_4_to_ee_cable_from_lookup(self,x):
        """
        linear interpolation of Cable Displacement Output from Disk Angle input from lookup table
        """
        mask_lower = self.ee_mapping['Disk4Angle'].lt(x)
        mask_lower = self.ee_mapping.loc[mask_lower]
        mask_upper = self.ee_mapping['Disk4Angle'].gt(x)
        mask_upper = self.ee_mapping.loc[mask_upper]
        
        x1 = mask_lower['Disk4Angle'].max()
        x2 = mask_upper['Disk4Angle'].min()
        if mask_lower['DeltaEECable'].max() < mask_upper['DeltaEECable'].min(): # y1 < y2 increasing function
            y1 = mask_lower['DeltaEECable'].max()
            y2 = mask_upper['DeltaEECable'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['DeltaEECable'].min()
            y2 = mask_upper['DeltaEECable'].max()
        
        if np.isnan(x1):
            return 0
        else:
            return (y1 + (x-x1)*(y2-y1)/(x2-x1))
        
    def wrist_cable_delta_to_wrist_angle_lookup(self,x):
        """
        linear interpolation of Wrist Angle from Disk Angle input from lookup table
        """
        mask_lower = self.wrist_mapping['WristCableDelta'].lt(x)
        mask_lower = self.wrist_mapping.loc[mask_lower]
        mask_upper = self.wrist_mapping['WristCableDelta'].gt(x)
        mask_upper = self.wrist_mapping.loc[mask_upper]
        
        x1 = mask_lower['WristCableDelta'].max()
        x2 = mask_upper['WristCableDelta'].min()
        if mask_lower['WristAngle'].max() < mask_upper['WristAngle'].min(): # y1 < y2 increasing function
            y1 = mask_lower['WristAngle'].max()
            y2 = mask_upper['WristAngle'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['WristAngle'].min()
            y2 = mask_upper['WristAngle'].max()
        
        if np.isnan(x1):
            return 0
        else:
            return (y1 + (x-x1)*(y2-y1)/(x2-x1))
        
    def ee_cable_to_gripper_angle(self,total_cable_delta,wrist_bending_cable_delta):    
        """
        calculation for relationship between EE gripper angle and EE cable delta 
        """

        ee_cable_delta = total_cable_delta - wrist_bending_cable_delta * self.compensation_factor #component of EE cable delta responsible for actuation
        if ee_cable_delta < 0: ee_cable_delta = 0 #actuation cable unable to apply compressive loads 
        linkage_length = self.min_ee_linkage_length + ee_cable_delta #length of actuated scissor linkage

        if linkage_length <= self.max_ee_linkage_length: # for EE pinch angles 0deg and greater
            ee_pinch_angle = np.arccos(linkage_length/2/self.ee_R)
        else: # for EE pinch angles < 0deg when applying gripping force
            equivalent_length = self.max_ee_linkage_length - (linkage_length - self.max_ee_linkage_length)
            ee_pinch_angle = -abs(np.arccos(((equivalent_length)/2/self.ee_R))) #excess cable delta for actuation past 0 deg results in linkage_length/2/R > 1 
            # domain of arccos func. is from -1 to 1 // addition of -1 accounts for spill over
            # -abs(arccos) ensures negative value which should always be the case due to if else condition checking
        return ee_pinch_angle

    def disk_position_to_joint_space(self,disk_positions,h,y_,r,n):
        outer_roll = disk_positions[0]/-1.56323325 #from dVRK 8mm needle driver coupling matrix    
        inner_roll = disk_positions[2]/1.56323325 #from dVRK 8mm needle driver coupling matrix
        
        pitch_cable_delta = disk_positions[1]*self.small_capstan_radius #calc pitch cable displacement
        pitch_angle = self.wrist_cable_delta_to_wrist_angle_lookup(pitch_cable_delta)

        ee_cable_wrist_component = -pitch_cable_delta #component of pitch compensation in EE cable displacement
        ee_cable_delta = self.disk_4_to_ee_cable_from_lookup(disk_positions[3]) #calc total EE cable displacement (linear interpolation from EE gripper linkage mapping)
        ee_jaw = self.ee_cable_to_gripper_angle(ee_cable_delta,ee_cable_wrist_component) #calc component of total EE cable displacement towards EE actuation 
        
        joint_values = [outer_roll,pitch_angle,inner_roll,ee_jaw]
        return joint_values

    #----------------------------------------------------------------------------------------------------------------------------------------------#
    ### IK ###
    #----------------------------------------------------------------------------------------------------------------------------------------------#
    def ee_cable_to_disk_4_lookup(self,x):
        """
        linear interpolation of Disk 4 Angle output from EE Cable Displacement input from lookup table
        """    
        mask_lower = self.ee_mapping['DeltaEECable'].lt(x)
        mask_lower = self.ee_mapping.loc[mask_lower]
 
        mask_upper = self.ee_mapping['DeltaEECable'].gt(x)
        mask_upper = self.ee_mapping.loc[mask_upper]

        x1 = mask_lower['DeltaEECable'].max()
        x2 = mask_upper['DeltaEECable'].min()
        if mask_lower['Disk4Angle'].max() < mask_upper['Disk4Angle'].min(): # y1 < y2 increasing function
            y1 = mask_lower['Disk4Angle'].max()
            y2 = mask_upper['Disk4Angle'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['Disk4Angle'].min()
            y2 = mask_upper['Disk4Angle'].max()

  
        if np.isnan(x1):
            return 0
        else:
            y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
            return y
    
    def wrist_angle_to_wrist_cable_delta_lookup(self,x):
        """
        linear interpolation of Wrist Cable Delta output from Wrist Angle input from lookup table
        """    
        mask_lower = self.wrist_mapping['WristAngle'].lt(x)
        mask_lower = self.wrist_mapping.loc[mask_lower]
        mask_upper = self.wrist_mapping['WristAngle'].gt(x)
        mask_upper = self.wrist_mapping.loc[mask_upper]

        x1 = mask_lower['WristAngle'].max()
        x2 = mask_upper['WristAngle'].min()
        if mask_lower['WristCableDelta'].max() < mask_upper['WristCableDelta'].min(): # y1 < y2 increasing function
            y1 = mask_lower['WristCableDelta'].max()
            y2 = mask_upper['WristCableDelta'].min()
        else: # y1 > y2 decreasing function
            y1 = mask_lower['WristCableDelta'].min()
            y2 = mask_upper['WristCableDelta'].max()

        if np.isnan(np.array([x1, x2, y1, y2])).any():
            return 0
        else:
            y = (y1 + (x-x1)*(y2-y1)/(x2-x1))
            return y
        
    def gripper_angle_to_ee_cable(self,ee_pinch_angle,wrist_bending_cable_delta):
        """
        calculation for relationship between EE gripper angle and EE cable delta 
        """
  
        if ee_pinch_angle > self.max_ee_pinch_angle: ee_pinch_angle = self.max_ee_pinch_angle #EE pinch angle joint limit, not able to open further
        elif ee_pinch_angle < self.Min_ee_pinch_angle: ee_pinch_angle = self.min_ee_pinch_angle #EE pinch angle joint limit, cannot clamp further

        if ee_pinch_angle >= 0: #normal actuation range open to closed
            ee_cable_delta = self.two_times_ee_R*np.cos(ee_pinch_angle) - self.min_ee_linkage_length # scissor linkage length - scissor linkage min length
        else: #clamping actuation range <0 deg for gripping
            ee_cable_delta = self.two_times_ee_R - self.min_ee_linkage_length + (self.max_ee_linkage_length - self.two_times_ee_R*np.cos(-ee_pinch_angle)) # scissor linkage max length - scissor linkage min length + additional scissor linkage "inverted length"

        total_cable_delta = ee_cable_delta + wrist_bending_cable_delta * self.compensation_factor
        return total_cable_delta

    def get_disk_angles(self,outer_roll,pitch_angle,inner_roll,EE_pinch_Angle,current_jaw_angle,h,y_,r,w,n):
        """
        calculate Disk Angles from jointspace and cablespace inputs
        [roll (jointspace), end effector, (jointspace), Cable1 (cablespace), Cable2 (cablespace), Cable3 (cablespace)]
        """ 

        disk_angles_rad = [-1.56323325*outer_roll , 0 , 1.56323325*inner_roll, 0] #1.56323325 coeff from dVRK 8mm needle driver coupling matrix

        pitch_cable_delta = self.wrist_angle_to_wrist_cable_delta_lookup(pitch_angle)
        disk_angles_rad[1] = pitch_cable_delta/self.small_capstan_radius
        
        ee_cable_wrist_component = -pitch_cable_delta
        if EE_pinch_Angle is None:
            disk_angles_rad[3] = current_jaw_angle
        else:
            ee_cable_delta = self.gripper_angle_to_ee_cable(EE_pinch_Angle,ee_cable_wrist_component)
            disk_angles_rad[3] = self.ee_cable_to_disk_4_lookup(ee_cable_delta) #linear interpolation from EE gripper linkage mapping
        disk_angles_rad = np.clip(disk_angles_rad, self.dial_lower_limits, self.dial_upper_limits)
        return disk_angles_rad.tolist()
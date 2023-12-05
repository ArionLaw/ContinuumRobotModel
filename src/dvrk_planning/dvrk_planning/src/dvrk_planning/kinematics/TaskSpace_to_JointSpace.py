test_cases = True
#test_cases = False
if test_cases == True:
    from utils import *
    from plotting import *

else:
    from dvrk_planning.kinematics.utils import *
    from dvrk_planning.kinematics.plotting import *
    from PyKDL import Vector, Rotation, Frame, dot

import numpy as np
import time


#----------------------------------------------------------------------------------------------------------------------------------------------#
#desired end effector orientation rotation matrix description
#----------------------------------------------------------------------------------------------------------------------------------------------#


#angle = getAngleTwoVectors(pos_des,vec_z) #acos(dot(vec_des,vec_z)/norm(vec_z)/norm(vec_des))
#angle_degrees = np.degrees(angle) #angle*180/pi
#axis = getRotationAxis(pos_des, vec_z, angle)#cross(vec_des,vec_z)/norm(vec_z)/norm(vec_des)/sin(angle)

def get_R_desired(frame):
    """
    deprecated
    obtain rotation matrix of desired end effector frame
    """
    """
    np_mat = np.mat([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]], dtype=float)
    for i in range(3):
        for j in range(3):
            np_mat[i, j] = frame.M[(i, j)]
            # 3x3 rotation matrix

    for i in range(3):
        np_mat[i, 3] = frame.p[i]
        # 3x1 position vector
    """
    return frame.M()

def calc_R_desired(EE_orientation_desired, tip_desired):
    """
    deprecated
    obtain rotation matrix of desired end effector orientation  
    """
    vec_x = np.array([1,0,0])
    vec_z = np.array([0,0,1])

    # azimuth about z axis
    phi = EE_orientation_desired[1]/abs(EE_orientation_desired[1])*np.arccos(np.dot(EE_orientation_desired[:2],vec_x[:2])/np.linalg.norm(vec_x[:2])/np.linalg.norm(EE_orientation_desired[:2]))
    phi_degrees = phi*180/np.pi
    #print("phi(azimuth): \n", round(phi_degrees,3))

    # altitude about y axis
    proj_des = [np.sqrt(EE_orientation_desired[0]**2 + EE_orientation_desired[1]**2) , EE_orientation_desired[2]]
    proj_z = np.array([0,1])
    theta = np.arccos(np.dot(proj_des,proj_z)/np.linalg.norm(proj_z)/np.linalg.norm(proj_des))
    theta_degrees = theta*180/np.pi
    #print("theta(elevation): \n", round(theta_degrees,3))

    # tool tip roll about z axis
    tip_roll = tip_desired*np.pi/180
    #print("tool tip roll: \n", round(tip_desired,3))

    R_desired = RotMtx('z',phi)@RotMtx('y',theta)@RotMtx('z',tip_roll)
    #print("desired Rotation matrix: \n", R_desired)

    return R_desired

#----------------------------------------------------------------------------------------------------------------------------------------------#
### FK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#
def get_wristPosition_from_PSMjoints(psm_joints):
    """
    obtain position of wrist before notches from yaw, pitch and insertion joint values
    joint[0] = yaw
    joint[1] = pitch
    joint[2] = insertion
    """
    EE_x = psm_joints[2]*np.sin(psm_joints[0])*np.cos(psm_joints[1])
    EE_y = -psm_joints[2]*np.sin(psm_joints[1])
    EE_z = -psm_joints[2]*np.cos(psm_joints[0])*np.cos(psm_joints[1])
    return [EE_x,EE_y,EE_z]

def get_R_shaft(psm_joints):
    """
    calculates rotation matrix of instrument prior to roll joint
    derived from daVinci PSM modified DH convention
    """
    R = RotMtx('x',np.pi/2)@RotMtx('z',(psm_joints[0]+np.pi/2))@RotMtx('x',-np.pi/2)@RotMtx('z',(psm_joints[1]-np.pi/2))@RotMtx('x',np.pi/2)
    return R

def get_R_fullwristmodel(roll,gamma,beta,alpha):
    """
    calculates rotation matrix of instrument wrist starting from the roll joint
    """
    R = RotMtx('z',roll)@RotMtx('x',-np.pi/2)@RotMtx('z',-np.pi/2)@RotMtx('x',np.pi) #frame 5 orientation after frame 4 Outer Roll 
    R = R@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha) #serial chain of 3 (3 phase, 3 notch segments)
    R = R@RotMtx('x',-np.pi/2)@RotMtx('z',-np.pi/2)@RotMtx('x',-np.pi/2) #correction to align frame 7 end effector orientation
    return R

def get_R_segment3notch(gamma,beta,alpha):
    """
    calculates the rotation matrix of a repeating segment of square cut wrist notches
    3 notch segment, 120 degree out of phase
    """
    phase_offset = 120*np.pi/180
   
    #based off modified DH-Convention
    R = RotMtx('z',gamma)@RotMtx('x',phase_offset)@RotMtx('z',beta)@RotMtx('x',phase_offset)@RotMtx('z',alpha)@RotMtx('x',phase_offset)
    return R

#----------------------------------------------------------------------------------------------------------------------------------------------#
### IK ###
#----------------------------------------------------------------------------------------------------------------------------------------------#

def get_PSMjoints_from_wristPosition(EE_pos_desired):
    """
    obtain initial yaw, pitch and insertion joint values of PSM kinematic chain from desired EE_position
    """
    x = float(EE_pos_desired[0])
    y = float(EE_pos_desired[1])
    z = float(EE_pos_desired[2])
    psm_insertion = np.sqrt(x**2 + y**2 + z**2); #magnitude = psm_insertion
    psm_pitch = np.arcsin(-y/psm_insertion)
    psm_yaw = np.arcsin(x/np.cos(psm_pitch)/psm_insertion)
    return [psm_yaw,psm_pitch,psm_insertion]

def IK_update(R_desired,roll,gamma,beta,alpha,printout):
    """
    IK numerical soln for taskspace to joint space roll and notch angles
    """
    start_time = time.time()
    i=0
    orientation_error = 1 #arbitrary value to enter loop
    previous_error = 2 #arbitrary value to prevent triggering exit condition(stuck in local minima)
    exit = False
    while (i<25) and (orientation_error>0.005) and exit == False:
        i=i+1
        #if printout is True: print("i: ",i)
        orientation_error = get_O_error(R_desired,roll,gamma,beta,alpha)
        #if printout is True: print("orientation error: ", orientation_error)

        delta = 0.25*orientation_error
        if (abs(previous_error - orientation_error)) < 0.00001:
            exit = True
        
        d_roll = [get_O_error(R_desired,roll+delta,gamma,beta,alpha),get_O_error(R_desired,roll-delta,gamma,beta,alpha)]
        d_gamma = [get_O_error(R_desired,roll,gamma+delta,beta,alpha),get_O_error(R_desired,roll,gamma-delta,beta,alpha)]
        d_beta = [get_O_error(R_desired,roll,gamma,beta+delta,alpha),get_O_error(R_desired,roll,gamma,beta-delta,alpha)]
        d_alpha = [get_O_error(R_desired,roll,gamma,beta,alpha+delta),get_O_error(R_desired,roll,gamma,beta,alpha-delta)]
        
        roll = roll_update(d_roll,orientation_error,roll,delta)
        gamma = angle_update(d_gamma,orientation_error,gamma,delta)
        beta = angle_update(d_beta,orientation_error,beta,delta)
        alpha = angle_update(d_alpha,orientation_error,alpha,delta)
        #if printout is True: print([roll,gamma,beta,alpha])
        previous_error = orientation_error

    joint_angles = [roll,gamma,beta,alpha]
    if printout is True:
        print("\n--- %s seconds ---" % (time.time() - start_time))
        print("i: ",i)
        print("orientation error: ", orientation_error)
    return joint_angles

def angle_update(d_theta,orientation_error,theta,delta):
    """
    decision making for notch angle updates from numerical soln
    """
    if (d_theta[0] < d_theta[1]) and (d_theta[0] < orientation_error):
        return theta+delta
    elif (d_theta[1] < d_theta[0]) and (d_theta[1] < orientation_error) and (theta-delta > 0):
        return theta-delta
    else:
        return theta

def roll_update(d_roll,orientation_error,roll,delta):
    """
    decision making for roll angle update from numerical soln
    """
    if (d_roll[0] < d_roll[1]) and (d_roll[0] < orientation_error):
        return roll+delta
    elif (d_roll[1] < d_roll[0]) and (d_roll[1] < orientation_error):
        return roll-delta
    else:
        return roll

def get_O_error(R_desired,roll,gamma,beta,alpha):
    """
    calculates orientation error of current configuration relative to R_desired 
    
    error is expressed as the angle of difference between rotations 

    ### old method ### error is expressed as a pythagorean magnitude of 3 euler angle errors
    angles_error = getEulerAngles(get_R_error(R_desired,roll,gamma,beta,alpha))
    E = np.sqrt(abs(angles_error[0])**2 + abs(angles_error[1])**2 + abs(angles_error[2])**2)
    """
    R_error = get_R_error(R_desired,roll,gamma,beta,alpha)
    E = np.arccos((np.trace(R_error)-1)/2)
    
    #print("angle error : " , angles_error)
    #print("orientation error: ", E)
    return E

def get_R_error(R_desired,roll,gamma,beta,alpha):
    """
    calculates matrix representing orientation error
    if there is no orientation error, matrix should be an identity matrix
    """
    R = R_desired@np.transpose(get_R_fullwristmodel(roll,gamma,beta,alpha))
    return R
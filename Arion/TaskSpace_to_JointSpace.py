from utils import *
from plotting import *
import numpy as np
import time

def IK_update(R_desired,roll,gamma,beta,alpha):
    """
    IK numerical soln for taskspace to joint space roll and notch angles
    """
    start_time = time.time()
    i=0
    orientation_error = 1
    previous_error = 2
    exit = False
    while (i<25) and (orientation_error>0.005) and exit == False:
        i=i+1
        #print("i: ",i)
        orientation_error = get_O_error(R_desired,roll,gamma,beta,alpha)
        #print("orientation error: ", orientation_error)

        delta = 0.25*orientation_error
        if (abs(previous_error - orientation_error)) < 0.00001:
            exit = True
        
        d_roll = [get_O_error(R_desired,roll+delta,gamma,beta,alpha),get_O_error(R_desired,roll-delta,gamma,beta,alpha)]
        d_gamma = [get_O_error(R_desired,roll,gamma+delta,beta,alpha),get_O_error(R_desired,roll,gamma-delta,beta,alpha)]
        d_beta = [get_O_error(R_desired,roll,gamma,beta+delta,alpha),get_O_error(R_desired,roll,gamma,beta-delta,alpha)]
        d_alpha = [get_O_error(R_desired,roll,gamma,beta,alpha+delta),get_O_error(R_desired,roll,gamma,beta,alpha-delta)]
        
        roll = angle_update(d_roll,orientation_error,roll,delta)
        gamma = angle_update(d_gamma,orientation_error,gamma,delta)
        beta = angle_update(d_beta,orientation_error,beta,delta)
        alpha = angle_update(d_alpha,orientation_error,alpha,delta)
        #print([roll,gamma,beta,alpha])
        previous_error = orientation_error

    joint_angles = [roll,gamma,beta,alpha]
    print("--- %s seconds ---" % (time.time() - start_time))
    print("i: ",i)
    print("orientation error: ", orientation_error)
    return joint_angles

def angle_update(d_theta,orientation_error,theta,delta):
    """
    decision making for angle update from numerical soln
    """
    if (d_theta[0] < d_theta[1]) and (d_theta[0] < orientation_error):
        angle = theta+delta
    elif (d_theta[1] < d_theta[0]) and (d_theta[1] < orientation_error) and (theta-delta > 0):
        angle = theta-delta
    else:
        angle = theta
    return angle

def get_O_error(R_desired,roll,gamma,beta,alpha):
    """
    calculates orientation error of current configuration relative to R_desired 
    error is expressed as a pythagorean magnitude of 3 euler angle errors
    """
    angles_error = getEulerAngles(get_R_error(R_desired,roll,gamma,beta,alpha))
    E = np.sqrt(abs(angles_error[0])**2 + abs(angles_error[1])**2 + abs(angles_error[2])**2)
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

def get_R_shaft(psm_yaw,psm_pitch):
    """
    calculates rotation matrix of instrument prior to roll joint
    derived from daVinci PSM modified DH convention
    """
    R = RotMtx('x',np.pi/2)@RotMtx('z',(psm_yaw+np.pi/2))@RotMtx('x',-np.pi/2)@RotMtx('z',(psm_pitch-np.pi/2))@RotMtx('x',np.pi/2)
    return R

def get_R_fullwristmodel(roll,gamma,beta,alpha):
    """
    calculates rotation matrix of instrument wrist starting from the roll joint
    """
    R = RotMtx('z',roll)@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha)@get_R_segment3notch(gamma,beta,alpha)
    return R

def get_R_segment3notch(gamma,beta,alpha):
    """
    calculates the rotation matrix of a repeating segment of square cut wrist notches
    3 notch segment, 120 degree out of phase
    """
    phase_offset = 120*np.pi/180
    # "lazy method"
    #R = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset);
   
    #based off modified DH-Convention
    R = RotMtx('x',(-np.pi/2))@RotMtx('z',(gamma-np.pi/2))@RotMtx('x',phase_offset)@RotMtx('z',beta)@RotMtx('x',phase_offset)@RotMtx('z',alpha)@RotMtx('x',phase_offset)@RotMtx('z',np.pi/2)@RotMtx('x',np.pi/2)
    return R
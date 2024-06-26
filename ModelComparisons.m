clc
clear
%{
% wrist inputs
c1_Displacement = 0; %mm
c2_Displacement = 0; %mm
c3_Displacement = 0; %mm
l = [c1_Displacement,c2_Displacement,c3_Displacement];

% fake Modified DH
a = [0 , h , h , h];
alpha = [0 , -1/2*pi , 2/3*pi , 2/3*pi];
d = [c , 0 , 0 , 0];

%}

%% my own code

% wrist parameters
n = 3; % sets of 3 cuts
h = 0.66; %mm notch height
c = 0.66; %mm notch spacing
prevStraightLength = 5; %mm
postStraightLength = 1; %mm
y_ = 0.56; %mm neutral bending plane
g = 1.16; %mm notch depth
OD = 1.37; %mm
ID = 0.94; %mm
r = OD/2;
w = r*sind(30);

shaft_length = 200; %mm

% end effector orientation
vec_x = [1;0;0];
vec_z = [0;0;1];
vec_des = [1;1;0];%[1;1;2.44949]; %[2.991;9.521;0.644];


angle = acos(dot(vec_des,vec_z)/norm(vec_z)/norm(vec_des));
angle_degrees = angle*180/pi;
axis = cross(vec_des,vec_z)/norm(vec_z)/norm(vec_des)/sin(angle);

% azimuth about z axis
phi = vec_des([2])/abs(vec_des([2]))*acos(dot(vec_des([1,2]),vec_x([1,2]))/norm(vec_x([1,2]))/norm(vec_des([1,2])));
phi_degrees = phi*180/pi

% altitude about y axis
proj_des = [sqrt(vec_des([1])^2 + vec_des([2])^2);vec_des([3])];
proj_z = [0;1];
theta = acos(dot(proj_des,proj_z)/norm(proj_z)/norm(proj_des));
theta_degrees = theta*180/pi

%theta = acos(dot(vec_des([1,3]),vec_z([1,3]))/norm(vec_z([1,3]))/norm(vec_des([1,3])))
%need to implement signed consideration (asin?)
%}
R_desired = RotMtx('z',phi)*RotMtx('y',theta)

%{
% disproof of "simplified" assumption
theta = theta/n
phi = phi/n

R_AngleDiv = RotationMatrix('z',phi)*RotationMatrix('y',theta)
R_AngleDiv*vec_z
R_AngleDiv*R_AngleDiv*vec_z
R_AngleDiv*R_AngleDiv*R_AngleDiv*vec_z
%}

%% FK
roll = 0*pi/180;
gamma = 0; %35*pi/180; 
beta  = 0; %25*pi/180; 
alpha = 0; %0*pi/180;

psm_yaw = 0.7854; %0*pi/180;
psm_pitch = -0.6155; %0*pi/180;
psm_insertion = 43.3013; %100; 

% wrist position FK
EE_x = psm_insertion*sin(psm_yaw)*cos(psm_pitch);
EE_y = -psm_insertion*sin(psm_pitch);
EE_z = -psm_insertion*cos(psm_yaw)*cos(psm_pitch);
EE_pos_FK = [EE_x;EE_y;EE_z]
%magnitude = sqrt(EE_x^2 + EE_y^2 + EE_z^2) %check: magnitude = psm_insertion
%}

% orientation FK
R_shaft = get_R_shaft(psm_yaw,psm_pitch)
R_wrist = get_R_fullwristmodel(roll,gamma,beta,alpha)
R_currentFK = R_shaft*R_wrist
R_desired

% wrist position FK


%% IK
% wrist position IK
EE_pos_desired = [25;25;-25];
psm_insertion = sqrt(EE_pos_desired(1)^2 + EE_pos_desired(2)^2 + EE_pos_desired(3)^2); %magnitude = psm_insertion
psm_pitch = asin(-EE_pos_desired(2)/psm_insertion);
psm_yaw = asin(EE_pos_desired(1)/cos(psm_pitch)/psm_insertion);
psm_joints = [psm_yaw;psm_pitch;psm_insertion]

% EE_orientation IK
R_wrist_desired = inv(R_shaft)*R_desired
joint_angles = [roll;gamma;beta;alpha]
joint_angles = IK_update(R_wrist_desired,joint_angles(1),joint_angles(2),joint_angles(3),joint_angles(4));
R_wrist_IK = get_R_fullwristmodel(joint_angles(1),joint_angles(2),joint_angles(3),joint_angles(4));
R_updated = R_shaft*R_wrist_IK;
R_desired;
%}

%% Cable Displacement Calculation per segment
% per set of 3 cuts
R = abs(h/theta);
K = 1/R;
if theta > 0
    L1 = sqrt(2*(R-y_-r)^2*(1-cos(theta)));
    L2 = sqrt(2*(R-y_+w)^2*(1-cos(theta)));
    L3 = sqrt(2*(R-y_+w)^2*(1-cos(theta)));
else
    L1 = sqrt(2*(R-y_+r)^2*(1-cos(theta)));
    L2 = sqrt(2*(R-y_-w)^2*(1-cos(theta)));
    L3 = sqrt(2*(R-y_-w)^2*(1-cos(theta)));
end

displacementL1 = h - L1;
displacementL2 = h - L2;
displacementL3 = h - L3;

%% functions

function joint_angles = IK_update(R_desired,roll,gamma,beta,alpha)
    i=0;
    orientation_error = 1;
    while (i<20)&&(orientation_error>0.001)
        i=i+1
        orientation_error = get_O_error(R_desired,roll,gamma,beta,alpha)
        delta = 0.5*orientation_error;

        d_roll = [get_O_error(R_desired,roll+delta,gamma,beta,alpha),get_O_error(R_desired,roll-delta,gamma,beta,alpha)];
        d_gamma = [get_O_error(R_desired,roll,gamma+delta,beta,alpha),get_O_error(R_desired,roll,gamma-delta,beta,alpha)];
        d_beta = [get_O_error(R_desired,roll,gamma,beta+delta,alpha),get_O_error(R_desired,roll,gamma,beta-delta,alpha)];
        d_alpha = [get_O_error(R_desired,roll,gamma,beta,alpha+delta),get_O_error(R_desired,roll,gamma,beta,alpha-delta)];
        
        roll = d_angle(d_roll,orientation_error,roll,delta);
        gamma = d_angle(d_gamma,orientation_error,gamma,delta);
        beta = d_angle(d_beta,orientation_error,beta,delta);
        alpha = d_angle(d_alpha,orientation_error,alpha,delta);
    end
    joint_angles = [roll;gamma;beta;alpha];
end

function angle = d_angle(d_theta,orientation_error,theta,delta)
    if d_theta(1) < d_theta(2) && d_theta(1) < orientation_error
        angle = theta+delta;
    elseif d_theta(2) < d_theta(1) && d_theta(2) < orientation_error
        angle = theta-delta;
    else
        angle = theta;
    end
end

function E = get_O_error(R_desired,roll,gamma,beta,alpha)
    % self constructed function
    angles_error = getEulerAngles(get_R_error(R_desired,roll,gamma,beta,alpha))';
    
    % inbuilt function
    %angles = rotm2eul(get_R_error(R_desired,roll,gamma,beta,alpha))';
    E = sqrt(abs(angles_error(1))^2 + abs(angles_error(2))^2 + abs(angles_error(3))^2);
end

function R_error = get_R_error(R_desired,roll,gamma,beta,alpha)
    R_error = R_desired*transpose(get_R_fullwristmodel(roll,gamma,beta,alpha));
    % if there is no orientation error, matrix should be identity
end

function R = get_R_shaft(psm_yaw,psm_pitch)
    R = RotMtx('x',pi/2)*RotMtx('z',(psm_yaw+pi/2))*RotMtx('x',-pi/2)*RotMtx('z',(psm_pitch-pi/2))*RotMtx('x',pi/2);
    %derived from modified DH convention
end

function R = get_R_fullwristmodel(roll,gamma,beta,alpha)
    R = RotMtx('z',roll)*get_R_segment3notch(gamma,beta,alpha)*get_R_segment3notch(gamma,beta,alpha)*get_R_segment3notch(gamma,beta,alpha);
end

function R = get_R_segment3notch(gamma,beta,alpha)
    phase_offset = 120*pi/180;
    %R = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset);
    % "lazy method"
    
    R = RotMtx('x',(-pi/2))*RotMtx('z',(gamma-pi/2))*RotMtx('x',phase_offset)*RotMtx('z',beta)*RotMtx('x',phase_offset)*RotMtx('z',alpha)*RotMtx('x',phase_offset)*RotMtx('z',pi/2)*RotMtx('x',pi/2);
    % based off modified DH-Convention
end

function R = RotMtx(a,theta)
    if a == 'z'
        R = [cos(theta) , -sin(theta) , 0;
             sin(theta) ,  cos(theta) , 0;
             0          ,  0        , 1];
    elseif a == 'y'
        R = [cos(theta) , 0 , sin(theta);
             0          , 1 , 0;
            -sin(theta) , 0 , cos(theta)];
    elseif a == 'x'    
        R = [1 , 0          ,  0;
             0 , cos(theta) , -sin(theta);
             0 , sin(theta) ,  cos(theta)];
    else
        R = [1 , 0 , 0;
             0 , 1 , 0;
             0 , 0 , 1];
    end
end

function angles = getEulerAngles(R)
    if abs(R(3,1)) ~= 1
        y_angle_theta = [-asin(R(3,1)) , pi - (-asin(R(3,1)))];
        x_angle_psi = [atan2(R(3,2)/cos(y_angle_theta(1)),R(3,3)/cos(y_angle_theta(1))) , atan2(R(3,2)/cos(y_angle_theta(2)),R(3,3)/cos(y_angle_theta(2)))];
        z_angle_phi = [atan2(R(2,1)/cos(y_angle_theta(1)),R(1,1)/cos(y_angle_theta(1))) , atan2(R(2,1)/cos(y_angle_theta(2)),R(1,1)/cos(y_angle_theta(2)))];
        angle_set1 = [x_angle_psi(1),y_angle_theta(1),z_angle_phi(1)];
        angle_set2 = [x_angle_psi(2),y_angle_theta(2),z_angle_phi(2)];
        angles = angle_set1;
    else
        z_angle_phi = 0;
        if R(3,1) == -1
            y_angle_theta = pi/2;
            x_angle_psi = z_angle_phi + atan2(R(1,2),R(1,3));
        else
            y_angle_theta = -pi/2;
            x_angle_psi = -z_angle_phi + atan2(-R(1,2),-R(1,3));
        end
        angles = [x_angle_psi,y_angle_theta,z_angle_phi];
    end
end


clc
clear

%% FK
% piecewise constant curvature York et. al
%{
% wrist parameters
n = 3; % sets of 3 cuts
h = 0.66; %mm
c = 0.66; %mm
prevStraightLength = 5; %mm
postStraightLength = 1; %mm
y = 0.56; %mm
g = 1.16; %mm
OD = 1.25; %mm
ID = 0.8; %mm

% wrist inputs
c1_Displacement = 0; %mm
c2_Displacement = 0; %mm
c3_Displacement = 0; %mm
l = [c1_Displacement,c2_Displacement,c3_Displacement];
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

% fake Modified DH
a = [0 , h , h , h];
alpha = [0 , -1/2*pi , 2/3*pi , 2/3*pi];
d = [c , 0 , 0 , 0];

% end effector orientation
vec_x = [1;0;0];
vec_y = [0;1;0];
vec_z = [0;0;1];
vec_des = [1,1,2.44949]; %[2.991;9.521;0.644];

angle = acos(dot(vec_des,vec_z)/norm(vec_z)/norm(vec_des));
angle_degrees = angle*180/pi
axis = cross(vec_des,vec_z)/norm(vec_z)/norm(vec_des)/sin(angle)

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

R_total = RotMtx('z',phi)*RotMtx('y',theta)
%R_total*vec_z

%{
theta = theta/n
phi = phi/n

R_AngleDiv = RotationMatrix('z',phi)*RotationMatrix('y',theta)
R_AngleDiv*vec_z
R_AngleDiv*R_AngleDiv*vec_z
R_AngleDiv*R_AngleDiv*R_AngleDiv*vec_z
%}

%% FK

roll = 0*pi/180;
phase_offset = 120*pi/180;
gamma = 35*pi/180; 
beta  = 25*pi/180; 
alpha = 0*pi/180;

% cascaded rotations about new relative axis orientation
R_segment = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset);
R_simp = RotMtx('z',roll)*R_segment

gamma = gamma/3; 
beta  = beta/3; 
alpha = alpha/3;

R_segment = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset);
R_full = RotMtx('z',roll)*R_segment*R_segment*R_segment

%% IK

THETA_MIN = 0.001; %rad

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




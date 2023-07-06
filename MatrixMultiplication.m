clc
clear

%% multiplication
syms roll
syms gamma
syms beta
syms alpha

phase_offset = 120*pi/180;

R_straight = RotMtx('y',0)*RotMtx('z',phase_offset)
R_notch = RotMtx('y',gamma)*RotMtx('z',phase_offset)
R_segment = RotMtx('y',gamma)*RotMtx('z',phase_offset)*RotMtx('y',beta)*RotMtx('z',phase_offset)*RotMtx('y',alpha)*RotMtx('z',phase_offset)
R_simp = RotMtx('z',roll)*R_segment
R_full = RotMtx('z',roll)*R_segment*R_segment*R_segment


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
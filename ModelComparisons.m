clc
clear

%% FK
% piecewise constant curvature York et. al

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




clear
clc

%% angles
psi = pi/4; % yaw
theta = pi/4; % pitch
phi = 0; % roll

%% eul2rotm
Re = eul2rotm([psi theta phi], 'ZYX');
Re = Re'; % transpose because MATLAB rot mats are intrinsic

%% using formula
cp = cos(phi);      sp = sin(phi);
ct = cos(theta);    st = sin(theta);
cs = cos(psi);      ss = sin(psi);

Rz = [cs ss 0; -ss cs 0; 0 0 1];
Ry = [ct 0 -st; 0 1 0; st 0 ct];
Rx = [1 0 0; 0 cp sp; 0 -sp cp];

R = Rx*Ry*Rz;

%% check
all(R == Re, 'all')

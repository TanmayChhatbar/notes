clear
clc

x = [1; 0; 0];

q = plot3([0 x(1)], [0 x(2)], [0 x(3)], 'y');
hold on
xp = plot3([0 x(1)], [0 0], [0 0], 'r');
yp = plot3([x(1) x(1)], [0 x(2)], [0 0], 'g');
zp = plot3([x(1) x(1)], [x(2) x(2)], [0 x(3)], 'b');
daspect([1 1 1])
grid on
% xlim([0 1])
% ylim([0 1])
% zlim([0 1])

for psi = linspace(0, pi/4, 35)
    eul = [psi; 0; 0]; % Define Euler angles for rotation
    x_rotated = rot(eul, x); % Rotate vector x using the defined Euler angles

    % update vis
    q.XData = [0 x_rotated(1)];
    q.YData = [0 x_rotated(2)];
    q.ZData = [0 x_rotated(3)];

    % update vis2
    xp.XData = [0 x_rotated(1)];
    xp.YData = [0 0];
    xp.ZData = [0 0];
    yp.XData = [x_rotated(1) x_rotated(1)];
    yp.YData = [0 x_rotated(2)];
    yp.ZData = [0 0];
    zp.XData = [x_rotated(1) x_rotated(1)];
    zp.YData = [x_rotated(2) x_rotated(2)];
    zp.ZData = [0 x_rotated(3)];
    
    % draw now
    drawnow
end

for theta = linspace(0, pi/4, 35)
    eul = [psi; theta; 0]; % Define Euler angles for rotation
    x_rotated = rot(eul, x); % Rotate vector x using the defined Euler angles

    % update vis
    q.XData = [0 x_rotated(1)];
    q.YData = [0 x_rotated(2)];
    q.ZData = [0 x_rotated(3)];

    % update vis2
    xp.XData = [0 x_rotated(1)];
    xp.YData = [0 0];
    xp.ZData = [0 0];
    yp.XData = [x_rotated(1) x_rotated(1)];
    yp.YData = [0 x_rotated(2)];
    yp.ZData = [0 0];
    zp.XData = [x_rotated(1) x_rotated(1)];
    zp.YData = [x_rotated(2) x_rotated(2)];
    zp.ZData = [0 x_rotated(3)];

    % draw now
    drawnow
end
hold off

function v = rot(eul, vin)
%% angles
psi = eul(1); % yaw
theta = eul(2); % pitch
phi = eul(3); % roll

%% using formula
cp = cos(phi);      sp = sin(phi);
ct = cos(theta);    st = sin(theta);
cs = cos(psi);      ss = sin(psi);

Rz = [cs ss 0; -ss cs 0; 0 0 1];
Ry = [ct 0 -st; 0 1 0; st 0 ct];
Rx = [1 0 0; 0 cp sp; 0 -sp cp];

% Rz = [cs -ss 0; ss cs 0; 0 0 1];
% Ry = [ct 0 st; 0 1 0; -st 0 ct];
% Rx = [1 0 0; 0 cp -sp; 0 sp cp];

R = Rx*Ry*Rz;

v = R*vin;
end

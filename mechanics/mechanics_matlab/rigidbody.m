clear
close
clc

x0 = [0 0 0, 0 0.1 0, 0 0 0, 0 0 5]';
t_end = 20;
dt = 0.001;
t = 0:dt:t_end; % Time vector
x = zeros(height(x0), length(t));
x(:, 1) = x0;
u = zeros(6, length(t)); % Control input initialization

for i = 2:length(t)
    xdot = rb(x(:, i-1), u(:, i), 1, eye(3)*0.95 + ones(3)*0.05);
    x(:, i) = x(:, i-1) + xdot * dt; 
end

%% plot
figure(1)
subplot(221)
plot3(x(1, :), x(2, :), x(3, :), 'HandleVisibility','off')
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")
title("Trajectory")
hold on
grid on
scatter3(x(1,1), x(2,1), x(3,1), '.', 'displayname', 'start');
hold off
legend

subplot(222)
plot(t, x(4, :))
hold on
plot(t, x(5, :))
plot(t, x(6, :))
hold off
legend("roll", "pitch", "yaw", 'location', 'best')
grid on
title("Euler Angles")
xlabel("time [s]")
ylabel("euler angles [rad]")

subplot(223)
plot(t, x(7, :))
hold on
plot(t, x(8, :))
plot(t, x(9, :))
legend("vx", "vy", "vz", 'location', 'best')
hold off
grid on
title("Body velocities")
xlabel("time [s]")
ylabel("body velocities [m/s]")

subplot(224)
plot(t, x(10, :))
hold on
plot(t, x(11, :))
plot(t, x(12, :))
legend("p", "q", "r", 'location', 'best')
hold off
grid on
title("Angular Velocities")
xlabel("time [s]")
ylabel("angular velocities [rad/s]")

%% plot animate
v = VideoWriter('figures/video.avi');
v.FrameRate = 24;
open(v);
figure(2)
clf
axis equal
grid on
hold on
xlabel("X [m]")
ylabel("Y [m]")
zlabel("Z [m]")

% plot full trajectory (optional)
plot3(x(1,:), x(2,:), x(3,:), 'w--')

L = max(range(x(1:3, :)')) * 0.1;
R = eul2rotm(x(6:-1:4,1)');
% initial quivers
qx = quiver3(x(1,1), x(2,1), x(3,1), ...
             L*R(1,1), L*R(2,1), L*R(3,1), ...
             'r', 'LineWidth', 2);

qy = quiver3(x(1,1), x(2,1), x(3,1), ...
             L*R(1,2), L*R(2,2), L*R(3,2), ...
             'g', 'LineWidth', 2);

qz = quiver3(x(1,1), x(2,1), x(3,1), ...
             L*R(1,3), L*R(2,3), L*R(3,3), ...
             'b', 'LineWidth', 2);

view(3)
xlim([min(x(1,:))-L*1.5 max(x(1,:))+L*1.5])
ylim([min(x(2,:))-L*1.5 max(x(2,:))+L*1.5])
zlim([min(x(3,:))-L*1.5 max(x(3,:))+L*1.5])
% animation loop
for k = round(1+linspace(0, t_end, t_end*24/2)/dt)

    % position
    X = x(1,k);
    Y = x(2,k);
    Z = x(3,k);

    % rotation matrix
    R = eul2rotm(x(6:-1:4,k)');

    % update quivers
    qx.XData = X; qx.YData = Y; qx.ZData = Z;
    qx.UData = L*R(1,1); qx.VData = L*R(2,1); qx.WData = L*R(3,1);

    qy.XData = X; qy.YData = Y; qy.ZData = Z;
    qy.UData = L*R(1,2); qy.VData = L*R(2,2); qy.WData = L*R(3,2);

    qz.XData = X; qz.YData = Y; qz.ZData = Z;
    qz.UData = L*R(1,3); qz.VData = L*R(2,3); qz.WData = L*R(3,3);

    % drawnow
    title("t="+string(t(k)))
    frame = getframe(gcf);
    writeVideo(v, frame);
end
close(v);

%% functions
function xdot = rb(x, u, m, I)
% from x
X = x(1); Y = x(2); Z = x(3);           % [m] global position
phi = x(4); theta = x(5); psi = x(6);   % [rad] euler angles
vx = x(7); vy = x(8); vz = x(9);        % [m/s] body-frame linear velocities
p = x(10); q = x(11); r = x(12);        % [rad/s] body-frame angular velocities

% from u
Fx = u(1); Fy = u(2); Fz = u(3);
Mx = u(4); My = u(5); Mz = u(6);

% from I
Ixx = I(1, 1);
Iyy = I(2, 2);
Izz = I(3, 3);
Ixz = I(1, 3);

% constants
g = -9.81; % [m/s2] gravitational acceleration
R = eul2rotm(x(6:-1:4)');
xdot_globalPositions = R * x(7:9);
xdot_eulerAngles = [ ...
                    1,  sin(phi)*tan(theta),  cos(phi)*tan(theta);
                    0,  cos(phi),            -sin(phi);
                    0,  sin(phi)/cos(theta),  cos(phi)/cos(theta) ...
                ] * [p; q; r];
xdot_bodyVelocities = [ ...
                    Fx/m - g*sin(theta)          - q*vz + r*vy;
                    Fy/m + g*cos(theta)*sin(phi) - r*vx + p*vz;
                    Fz/m + g*cos(theta)*cos(phi) - p*vy + q*vx;
                ];
I_D = Ixx*Izz - Ixz^2;

xdot_bodyRates = [ ...
                    Izz/I_D * (Mx + Ixz*p*q - (Izz-Iyy)*q*r) ...
                    + Ixz/I_D * (Mz - Ixz*q*r - (Iyy-Ixx)*p*q);
                    1/Iyy * (My-(Ixx-Izz)*p*r - Ixz*(p^2-r^2));
                    Ixz/I_D * (Mx + Ixz*p*q - (Izz-Iyy)*q*r) ...
                    + Ixx/I_D * (Mz - Ixz*q*r - (Iyy-Ixx)*p*q);
            ];

xdot = [xdot_globalPositions;
        xdot_eulerAngles;
        xdot_bodyVelocities;
        xdot_bodyRates
        ];

end
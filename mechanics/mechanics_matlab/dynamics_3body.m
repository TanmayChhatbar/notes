clc; clear;

% sim params
dt = 0.01;
tmax = 100;
t = 0:dt:tmax;

% masses
m1 = 1; m2 = 1; m3 = 0.2;

% initial positions
x1 = [0 1 0]';
x2 = -x1;
x3 = [2  0 0]';

% initial velocities
v1 = [0 0 1]';
v2 = -v1;
v3 = [0 0.001 0]';

% spring rest lengths
L12 = norm(x1 - x2);
L23 = norm(x2 - x3);
L31 = norm(x3 - x1);

% spring params
k = 10;
c = 0.1;

% state
state = zeros(18, length(t));
state(:,1) = [x1; x2; x3; v1; v2; v3];

for i = 2:length(t)

    % unpack
    x1 = state(1:3,i-1);  x2 = state(4:6,i-1);  x3 = state(7:9,i-1);
    v1 = state(10:12,i-1);v2 = state(13:15,i-1);v3 = state(16:18,i-1);

    % relative vectors
    r12 = x1 - x2;  r23 = x2 - x3;  r31 = x3 - x1;
    L12c = norm(r12); L23c = norm(r23); L31c = norm(r31);

    e12 = r12 / L12c;
    e23 = r23 / L23c;
    e31 = r31 / L31c;

    % deformation
    d12 = L12c - L12;
    d23 = L23c - L23;
    d31 = L31c - L31;

    % deformation rates
    d12dot = dot(v1 - v2, e12);
    d23dot = dot(v2 - v3, e23);
    d31dot = dot(v3 - v1, e31);

    % spring forces
    F12 = (k*d12 + c*d12dot) * e12;
    F23 = (k*d23 + c*d23dot) * e23;
    F31 = (k*d31 + c*d31dot) * e31;

    % acceleration
    a1 = (-F12 + F31) / m1;
    a2 = ( F12 - F23) / m2;
    a3 = ( F23 - F31) / m3;

    % integrate
    v1 = v1 + a1*dt;
    v2 = v2 + a2*dt;
    v3 = v3 + a3*dt;

    x1 = x1 + v1*dt;
    x2 = x2 + v2*dt;
    x3 = x3 + v3*dt;

    % pack
    state(:,i) = [x1; x2; x3; v1; v2; v3];
end

plot(t, state(1,:))
hold on
plot(t, state(2,:))
plot(t, state(3,:))
hold off

clear p
p(1) = plot3(state(1, i), state(2, 1), state(3, 1), 'r.', 'markersize', 20);
hold on
p(2) = plot3(state(4, 1), state(5, 1), state(6, 1), 'g.', 'markersize', 20);
p(3) = plot3(state(7, 1), state(8, 1), state(9, 1), 'b.', 'markersize', 20);
p(4) = plot3(state([4 7], 1),  state([5 8], 1), state([6 9], 1), 'k');
p(5) = plot3(state([7 10], 1), state([8 11], 1), state([9 12], 1), 'k');
p(6) = plot3(state([10 4], 1),  state([11 5], 1), state([12 6], 1), 'k');

hold off
daspect([1 1 1])
xlim([-2 2])
ylim([-2 2])
zlim([-2 2])
grid on
for i = 1:length(t)
  for j=1:3
    % set point coords
    set(p(j), ...
        'XData', state(1+(j-1)*3, i), ...
        'YData', state(2+(j-1)*3, i), ...
        'ZData', state(3+(j-1)*3, i));

    % set line coords
    if j ~= 3
      set(p(j+3), ...
        'XData', state([1 4]+(j-1)*3, i), ...
        'YData', state([2 5]+(j-1)*3, i), ...
        'ZData', state([3 6]+(j-1)*3, i));
    else
        set(p(j+3), ...
        'XData', state([1 -5]+(j-1)*3, i), ...
        'YData', state([2 -4]+(j-1)*3, i), ...
        'ZData', state([3 -3]+(j-1)*3, i));
    end
  end
  drawnow
end

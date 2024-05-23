% Clear workspace, command window, and close all figures
clear;
clc;
close all;

%% Extract states from saved file
load('states.mat');
states = states_struct;

%% Plot the results
figure;

% Plot payload position
subplot(2, 2, 1);
hold on;
plot(states.t, states.x0(:, 1), 'b');
plot(states.t, states.x0(:, 2), 'r--');
plot(states.t, -states.x0(:, 3), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Position (m)', 'Interpreter', 'latex', 'FontSize', 10);
legend('x', 'y', 'z', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Payload Position');
grid on;

% Plot payload velocity
subplot(2, 2, 2);
hold on;
plot(states.t, states.v0(:, 1), 'b');
plot(states.t, states.v0(:, 2), 'r--');
plot(states.t, -states.v0(:, 3), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (m/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('$v_{x}$', '$v_{y}$', '$v_{z}$', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Payload Velocity');
grid on;

% Plot payload angular velocity
subplot(2, 2, 3);
hold on;
plot(states.t, states.omega0(:, 1), 'b');
plot(states.t, states.omega0(:, 2), 'r--');
plot(states.t, -states.omega0(:, 3), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (rad/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('${\omega}_{x}$', '${\omega}_{y}$', '${\omega}_{z}$', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Payload Angular Velocity');
grid on;

% Plot payload attitude (Euler angles)
subplot(2, 2, 4);
hold on;
plot(states.t, states.eulers0(1, :), 'b');
plot(states.t, states.eulers0(2, :), 'r--');
plot(states.t, -states.eulers0(3, :), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Euler Angles (degree)', 'Interpreter', 'latex', 'FontSize', 10);
legend('${\psi}$', '${\theta}$', '${\phi}$', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Payload Attitude');
grid on;

% Plot quadrotor link orientation
figure;
subplot(3, 1, 1);
hold on;
plot(states.t, -squeeze(states.q(1, 1, :)), 'b');
plot(states.t, -squeeze(states.q(1, 2, :)), 'r--');
plot(states.t, -squeeze(states.q(1, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Link Orientation (x)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Link 1', 'Link 2', 'Link 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Link Orientation (x)');
grid on;

subplot(3, 1, 2);
hold on;
plot(states.t, -squeeze(states.q(2, 1, :)), 'b');
plot(states.t, -squeeze(states.q(2, 2, :)), 'r--');
plot(states.t, -squeeze(states.q(2, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Link Orientation (Y)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Link 1', 'Link 2', 'Link 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Link Orientation (y)');
grid on;

subplot(3, 1, 3);
hold on;
plot(states.t, -squeeze(states.q(3, 1, :)), 'b');
plot(states.t, -squeeze(states.q(3, 2, :)), 'r--');
plot(states.t, -squeeze(states.q(3, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Link Orientation (Z)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Link 1', 'Link 2', 'Link 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Link Orientation (z)');
grid on;

% Plot quadrotor angular velocity
figure;
subplot(3, 1, 1);
hold on;
plot(states.t, squeeze(states.omega(1, 1, :)), 'b');
plot(states.t, squeeze(states.omega(1, 1, :)), 'r--');
plot(states.t, squeeze(states.omega(1, 1, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (x) (rad/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Angular Velocity (x)');
grid on;

subplot(3, 1, 2);
hold on;
plot(states.t, squeeze(states.omega(2, 1, :)), 'b');
plot(states.t, squeeze(states.omega(2, 1, :)), 'r--');
plot(states.t, squeeze(states.omega(2, 1, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (y) (rad/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Angular Velocity (y)');
grid on;

subplot(3, 1, 3);
hold on;
plot(states.t, squeeze(states.omega(3, 1, :)), 'b');
plot(states.t, squeeze(states.omega(3, 1, :)), 'r--');
plot(states.t, squeeze(states.omega(3, 1, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (z) (rad/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Angular Velocity (z)');
grid on;

% Plot quadrotor positions
figure;
subplot(3, 1, 1);
hold on;
plot(states.t, squeeze(states.x(1, 1, :)), 'b');
plot(states.t, squeeze(states.x(1, 2, :)), 'r--');
plot(states.t, squeeze(states.x(1, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Position (x) (m)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Position (x)');
grid on;

subplot(3, 1,2);
hold on;
plot(states.t, squeeze(states.x(2, 1, :)), 'b');
plot(states.t, squeeze(states.x(2, 2, :)), 'r--');
plot(states.t, squeeze(states.x(2, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Position (y) (m)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Position (y)');
grid on;

subplot(3, 1, 3);
hold on;
plot(states.t, -squeeze(states.x(3, 1, :)), 'b');
plot(states.t, -squeeze(states.x(3, 2, :)), 'r--');
plot(states.t, -squeeze(states.x(3, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Position (z) (m)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Position (z)');
grid on;
ytickformat('%.1f'); % Formats y-axis tick labels to display with two decimal places

% Plot quadrotor velocities
figure;
subplot(3, 1, 1);
hold on;
plot(states.t, squeeze(states.v(1, 1, :)), 'b');
plot(states.t, squeeze(states.v(1, 2, :)), 'r--');
plot(states.t, squeeze(states.v(1, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (x) (m/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Velocity (x)');
grid on;

subplot(3, 1, 2);
hold on;
plot(states.t, squeeze(states.v(2, 1, :)), 'b');
plot(states.t, squeeze(states.v(2, 2, :)), 'r--');
plot(states.t, squeeze(states.v(2, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (y) (m/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10,'location','west');
title('Quadrotor Velocity (y)');
grid on;

subplot(3, 1, 3);
hold on;
plot(states.t, -squeeze(states.v(3, 1, :)), 'b');
plot(states.t, -squeeze(states.v(3, 2, :)), 'r--');
plot(states.t, -squeeze(states.v(3, 3, :)), 'k-.');
hold off;
xlabel('Time', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Velocity (z) (m/s)', 'Interpreter', 'latex', 'FontSize', 10);
legend('Quadrotor 1', 'Quadrotor 2', 'Quadrotor 3', 'Interpreter', 'latex', 'FontSize', 10, 'location','west');
title('Quadrotor Velocity (z)');
grid on;

% Plot trajectory of payload and quadrotors
figure;
plot3(states.x0(:, 1), states.x0(:, 2), -states.x0(:, 3), 'k');
hold on;
for i = 1:3
plot3(squeeze(states.x(1, i,:)), squeeze(states.x(2, i, :)), -squeeze(states.x(3, i, :)));
end
hold off;
grid on;
xlabel('X (m)', 'Interpreter', 'latex', 'FontSize', 10);
ylabel('Y (m)', 'Interpreter', 'latex', 'FontSize', 10);
zlabel('Z (m)', 'Interpreter', 'latex', 'FontSize', 10);

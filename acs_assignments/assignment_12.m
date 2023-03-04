clc;
clear;

%% impedance control
% pd controller params
Kp = 1000; %diag([10 10 10 10 1 1])*1000;
Kd = 100; %diag([2 2 2 2 2 2])*100;

% environment spring stiffness
K_env = 100;

% impedance params
Md = eye(6) * 1;

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired joint positions in the joint space
q_rest_env = [0; 0; pi/100];
x_rest_env = direct_kinematics(q_rest_env);

q_contact_1 = pi/80; % interaction with environment
q_contact_2 = pi/90; % interaction with environment
q_no_contact = pi/200; % no interaction with environment

% trajectory
q1_d = [0 0 0 0 0 0];
q2_d = [0 0 0 0 0 0];
q3_d = [0 q_no_contact q_contact_1 q_no_contact q_contact_2 0];
traj = [q1_d; q2_d; q3_d];
time_vector = [0, 2, 4, 6, 8, 10];

%%
% show(robot, q_rest_env');
% hold on;
% [x, ~] = meshgrid(-2:1:2); % Generate x and y data
% [~, y] = meshgrid(3:1:7); % Generate x and y data
% z = ones(size(x, 1)); % Generate z data
% surf(x, y, z); % Plot the surface
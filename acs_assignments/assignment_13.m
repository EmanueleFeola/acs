clc;
clear;

%% Force Control with Inner Position Loop
% force desired value
% Fd = [0; 6; 3]; % joint space torques
Fd = [0; 0; 3];

% position pd controller params
Kp_pos = 10;
Kd_pos = 10;

% force pi controller params
% Kp_force = 0.1; % joint space
% Ki_force = 0.01; % joint space
Kp_force = 1;
Ki_force = 0.1;

% environment spring stiffness
K_env = 100;

% impedance params
Md = eye(3) * 1;

% initial joint conditions
q0 = [0; pi/2; -pi/2]; %[0;0;0];
dq0 = [0;0;0];

% desired joint positions in the joint space
% q_home = [0; pi/2; -pi/2];
q_rest_env = [0; pi/2; -pi/2.1]; % environment rest position

x_home = direct_kinematics([0;0;0]);
x_home = x_home(1:3, 1);

x_rest_env = direct_kinematics(q_rest_env);
x_rest_env = x_rest_env(1:3, 1);

x_ref = x_rest_env;

%%
show(robot, q0');
hold on;
show(robot, q_rest_env');

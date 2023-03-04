clc;
clear;

%% compliance control
% pd controller params
Kp = 10000; %diag([10 10 10 10 1 1])*1000;
Kd = 1000; %diag([2 2 2 2 2 2])*100;

% environment spring stiffness
K_env = 1000;

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired joint positions in the joint space
q_home = [0; 0; 0];
q_rest_env = [0; 0; pi/100]; % environment rest position
q_desired = [0; 0; pi/80]; % interaction with environment
% q_desired = [0; 0; pi/200]; % no interaction with environment

% desired ee position in the operational space
x_home = direct_kinematics(q_home);
x_rest_env = direct_kinematics(q_rest_env);
x_desired = direct_kinematics(q_desired);

fprintf("start position is: %.3f\n", x_home(3, 1));
fprintf("target position is: %.3f\n", x_desired(3, 1));
fprintf("env. rest position is: %.3f\n", x_rest_env(3, 1));

%% plot manipulator
%%config = homeConfiguration(robot);
% show(robot, q_home');
% hold on;
% show(robot, q_rest_env');
% show(robot, q_desired');

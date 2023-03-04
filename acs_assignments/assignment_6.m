clc;
clear;

% pd controller params
Kp = 1000;
Kd = 1000;

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired joint positions
q_d = [pi/2; pi/4; -pi/3];

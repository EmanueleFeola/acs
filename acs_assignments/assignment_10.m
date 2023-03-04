clc;
clear;

% pd controller params
Kp = 1000; %diag([100 100 100]);
Kd = 100; %diag([100 100 100]);

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired trajectory in the joint space
time_vector = [0 2 4 6 8 10];
q1_d = [0 -pi/2 pi/2 -pi/2 -pi/6 -pi/2];
q2_d = [0 -pi/2 0 pi/4 -pi/6 0];
q3_d = [0 pi/4 -pi/2 pi 0 pi/4];
traj = [q1_d; q2_d; q3_d];
time_vector = [0, 2, 4, 6, 8, 10];
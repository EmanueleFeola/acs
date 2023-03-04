clc;
clear;

w = [10 10 10];
xi = [0.99 0.99 0.99];

% pd controller params
Kp = diag(w .* w); %1000 * eye(3,3);
Kd = diag(2 * xi .* w); %500 * eye(3,3);

% Kp = 100 * eye(3,3);
% Kd = 1 * eye(3,3);

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired trajectory
time_vector = [0 2 4 6 8 10];
q1_d = [0 pi/4 -pi/2 pi -pi/2 0];
q2_d = [0 pi/2 -pi/4 pi -pi 0];
q3_d = [0 pi 0 0 pi 0];
traj = [q1_d; q2_d; q3_d];
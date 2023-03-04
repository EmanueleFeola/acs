clc;
% clear;

%% Operational Space PD control law with gravity compensation
% pd controller params
% Kp = 1000000;
% Kd = 10000;
Kp = 10000;
Kd = 1000;

% initial joint conditions
q0 = [0;0;0];
dq0 = [0;0;0];

% desired joint positions in the joint space
% q_d = [pi/2; -pi; pi/12];
q_d = [pi/6; pi/4; -pi/2]; % funzia
% q_d = [pi/2; pi; pi/12]; % non funzia
% q_d = [0; 0; pi/100]; % testing

% desired ee position in the operational space
q_d_operational_space = direct_kinematics(q_d);

% check rank of J_A(q)?
jacobian_rank = rank(get_JA(q_d));
jacobian_rank = rank(get_JA(q_d)');
% jacobian_kernel = null(get_JA(q_d));

% show(robot, q_d');


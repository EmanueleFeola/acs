clc
clear

%% import fixed params (link lenghts and masses), symbolic params (joint variables, ...)
[joint_params, link_params, joints_q, joints_q_dot, joints_q_dot_dot, joints_type, links_length, gravity_base, tau, Fv, Fs, f_ext] = manipulator_params();

%% import robot, create random joint configuration
robot = importrobot('../RRR_zxx_modified_lengths.urdf');
robot.DataFormat = 'row';
% robot_config = randomConfiguration(robot);
robot_config = homeConfiguration(robot);
q_test = [robot_config(1) robot_config(2) robot_config(3)]; 

%% Kinematics (direct kin, jacobians)
a_list     = [0, links_length(2), links_length(3), links_length(4) 0];
alpha_list = [0, pi/2, 0, 0, 0];
d_list     = [links_length(1), 0, 0, 0, 0];
theta_list = [pi/2, joints_q(2), joints_q(3), joints_q(4), 0];
DH = [a_list' alpha_list' d_list' theta_list'];

% syms L1 L2 L3 L4 real
% a_list     = [0, L2, L3, L4 0];
% alpha_list = [0, pi/2, 0, 0, 0];
% d_list     = [L1, 0, 0, 0, 0];
% theta_list = [pi/2, joints_q(2), joints_q(3), joints_q(4), 0];
% DH = [a_list' alpha_list' d_list' theta_list'];

% kinematics: homogeneous matrix, geometric jacobian, analytic jacobian
kin = Kinematics(DH, joints_q, joints_type);

assignment_1;

%% energy (K, U (and B))
assignment_2;

%% equations of motion (C, G, lagrange, rne)
num_q = [0, 0, pi/2];
num_q_d = [0, 0, 0];
num_q_dd = [0, 0, 0];
num_f_ext = [0 0 0 0 0 0];

q = joints_q(1, 2:end);
q_d = joints_q_dot(1, 2:end);
q_dd = joints_q_dot_dot(1, 2:end);

assignment_3_4;
%% dynamic model in the operational space
% assignment_5;


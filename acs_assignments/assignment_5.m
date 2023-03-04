% Compute the dynamic model in the operational space

clc

syms x x_d x_dd real % end effector cartesian position, velocity, acceleration
syms fx fy fz tqx tqy tqz % symbolic forces due to the joints

h = [fx fy fz tqx tqy tqz]';
he = f_ext; % already defined symbolic external forces

% transf. matrix that binds the 2 jacobians
T_A = Kinematics.computeTa();

% operational space version of the matrixes
B_A = kin.computeBa();

q_dot = joints_q_dot(1, 2:end);
C_A_xd = kin.computeCa_xd(B_A, q_dot');

g_A = kin.computeGa();

u = T_A' * h; % force contribution due to the joint forces
ue = T_A' * he; % force contribution due to the interaction forces with the environment

% operational space dynamic model
equation = eq(B_A * x_dd + C_A_xd * x_d + g_A, u - ue);
clc;
clear;

%% note
% regressore Y: [ddq, dq, sin(q)]
% vettore parametri da stimare theta: [I; F; G]. (I=inertia, ovvero B nel caso 1-dof)
%%

q0 = 0;
dq0 = 0;

A=1;
Fc=1;

lambda=50;

K_theta=diag([0.01 0.01 0.01])*100;
K_theta_inv=inv(K_theta);
K_D = 10;

B_hat_initial = get_B_one_dof()*0.7;
F_hat_initial = get_F_one_dof()*0.7;
G_hat_initial = get_G_one_dof()*0.7;

theta = [get_B_one_dof(); get_F_one_dof(); get_G_one_dof];

%%
% syms A
% syms Fc
% syms t
% q = A * sin(Fc*t);
% % differentiate w.r.t t
% dq = diff(q, t);
% % differentiate w.r.t y
% ddq = diff(dq, t);
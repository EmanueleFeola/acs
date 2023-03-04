clc
clear
% robotics toolbox for matlab, by peter corke

%% define manipulator
base_len = 0.5;
len_1 = 0.25;
len_2 = 0.5;
len_3 = 0.75;

% theta, d, a, alpha
DH_table = [
    [0 base_len 0 0]
    [0 0 len_1 pi/2]
    [0 0 len_2 0]
    [0 0 len_3 0]
    ];
DH_offsets = [pi/2; 0; 0; 0;];

syms('fixed_frame_q', 'real')
syms('q1', 'real') 
syms('q2', 'real') 
syms('q3', 'real')
joint_vars_sym = [fixed_frame_q, q1, q2, q3];

% build manipulator from DH table
for i = 1:size(DH_table, 1)
    L(i) = Link(DH_table(i, :));
    L(i).offset = DH_offsets(i);

    L_sym(i) = Link(DH_table(i, :));
    L_sym(i).offset = DH_offsets(i);
end

manipulator = SerialLink(L, 'name', 'manipulator');
manipulator_sym = SerialLink(L_sym, 'name', 'manipulator');

manipulator.plot([0 0 0 0], 'jaxes', 'joints');

% TAU = R.rne(X, OPTIONS) as above where X=[Q,QD,QDD] (1x3N).
% manipulator.rne([0, pi/2, pi/3, -pi/4])

% figure();
% manipulator.teach
clc
clear
close all

% theta, d, a, alpha
DH_table = [
    [0 1 0 0]
    [0 0 0 pi/2]
    [0 1 0 -pi/2]
    [0 0 1 0]
    ];
DH_offsets = [0, 0, 0, -pi/2];

syms('q1', 'real')
syms('q2', 'real')
syms('q3', 'real')
syms('q4', 'real')
% joint_vars_sym = [q1 q2 q3 q4];

for i = 1:size(DH_table, 1)
    L(i) = Link(DH_table(i, :));
    L(i).offset = DH_offsets(i);

    L_sym(i) = Link(DH_table(i, :));
    L_sym(i).offset = DH_offsets(i);
end

manipulator = SerialLink(L, 'name', 'arm');
manipulator.teach;
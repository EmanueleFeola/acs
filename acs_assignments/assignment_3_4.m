% equations of motion (dynamic model)
%% lagrangian approach
J_G = kin.J_G(:, 2:end);
C = compute_coriolis_torque(B, joints_q(1, 2:end), joints_q_dot(1, 2:end));
G_sym = compute_gravity_torque(links(2:end), gravity_base);

kin.C = simplify(vpa(C));
kin.G = simplify(vpa(G_sym));

left_handside = B * q_dd.' + C * q_d.' + G_sym; % + Fv * q_d + Fs * sign(q_d).';
right_handside = tau - J_G.' * f_ext;

left_handside = Kinematics.simplifyMatrix(left_handside);
right_handside = Kinematics.simplifyMatrix(right_handside);

% check B properties
B_num = double(kin.evalMatrix(kin.B, q, num_q, 4));
check_is_symm = issymmetric(B_num);
eigs = eig(B_num);
check_positive_def = all(eigs > 0);

% G_sym
% B
% C

%% rne
tau_sym = rne(kin, gravity_base, links(2:end), joints_q(2:end), joints_q_dot(2:end), joints_q_dot_dot(2:end), f_ext);

%% evaluate results
tau_num = Kinematics.evalMatrix(right_handside-left_handside, [q, q_d, q_dd, f_ext.'], [num_q, num_q_d, num_q_dd, num_f_ext], 4);
tau_1 = solve(tau_num(1, 1), tau(1));
tau_2 = solve(tau_num(2, 1), tau(2));
tau_3 = solve(tau_num(3, 1), tau(3));
tau_lagrangian = round([tau_1; tau_2; tau_3], 4);
latex(sym(tau_lagrangian));

% show(robot, num_q);

% G = rne(q, zero, zero, g0);
% C = rne(q, dq, zero, zero);
% Bi (q) = rne(q; 0; ei ; 0); ei = i-th element equal to 1
tau_rne = Kinematics.evalMatrix(tau_sym, [q, q_d, q_dd, f_ext.'], [num_q, num_q_d, num_q_dd, num_f_ext], 4);

delta = tau_lagrangian - tau_rne;
fprintf("lagrangian - rne delta = \n");
disp(delta);
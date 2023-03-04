function [joint_params, link_params, joints_q, joints_q_dot, joints_q_dot_dot, joints_type, links_length, gravity_base, tau, Fv, Fs, f_ext] = manipulator_params()
    %% generic variables
    syms('tau_joint_1', 'real');
    syms('tau_joint_2', 'real');
    syms('tau_joint_3', 'real');
    syms('Fv', 'real'); % friction
    syms('Fs', 'real'); % friction
    syms('f_x', 'real'); % external force (x component)
    syms('f_y', 'real'); % external force (y component)
    syms('f_z', 'real'); % external force (z component)
    syms('tq_x', 'real'); % external torque (x component)
    syms('tq_y', 'real'); % external torque (y component)
    syms('tq_z', 'real'); % external torque (z component)

    gravity_base = [0; 0; -9.81]; % gravity vector expressed wrt base frame %% correct one
%     gravity_base = [0; -9.81; 0]; % gravity vector expressed wrt base frame %% the one used in simulink
    % tau = [0; tau_joint_1; tau_joint_2; tau_joint_3];
    tau = [tau_joint_1; tau_joint_2; tau_joint_3];
    f_ext = [f_x; f_y; f_z; tq_x; tq_y; tq_z;];

    %% manipulator RRR variables (info about joints and links)
    joint_1 = struct('type', 1, 'q', 0, 'q_dot', 0, 'q_dot_dot', 0); % type -1 = fixed
    joint_2 = struct('type', 1, 'q', sym('t1', 'real'), 'q_dot', sym('t1_d', 'real'), 'q_dot_dot', sym('t1_d_d', 'real')); % type 1 = revolute
    joint_3 = struct('type', 1, 'q', sym('t2', 'real'), 'q_dot', sym('t2_d', 'real'), 'q_dot_dot', sym('t2_d_d', 'real')); % type 1 = revolute
    joint_4 = struct('type', 1, 'q', sym('t3', 'real'), 'q_dot', sym('t3_d', 'real'), 'q_dot_dot', sym('t3_d_d', 'real')); % type 1 = revolute
    joint_params = {joint_1, joint_2, joint_3, joint_4};

    len_1 = 1;
    len_2 = 1.2;
    len_3 = 2;
    len_4 = 3.7;
    
    m_1 = 0;
    m_2 = 10;
    m_3 = 5;
    m_4 = 8.2;
        
    link_1 = struct('name', 'Link1', 'mass', m_1, 'length', len_1, 'radius', 1, 'CoM', [0; 0; -len_1 / 2]);    
    link_2 = struct('name', 'Link2', 'mass', m_2, 'length', len_2, 'radius', 1, 'CoM', [-len_2 / 2; 0; 0]);
    link_3 = struct('name', 'Link3', 'mass', m_3, 'length', len_3, 'radius', 1, 'CoM', [-len_3 / 2; 0; 0]);
    link_4 = struct('name', 'Link4', 'mass', m_4, 'length', len_4, 'radius', 1, 'CoM', [-len_4 / 2; 0; 0]);
    
%     link_2 = struct('mass', 3, 'length', len_2, 'radius', 1, 'CoM', [-len_2 / 2; 0; 0]);
%     link_3 = struct('mass', 2, 'length', len_3, 'radius', 1, 'CoM', [-len_3 / 2; 0; 0]);
%     link_4 = struct('mass', 1, 'length', len_4, 'radius', 1, 'CoM', [-len_4 / 2; 0; 0]);
    link_params = {link_1, link_2, link_3, link_4};

    %%
    % quick info about joints and links
    joints_q = sym(zeros(1, size(joint_params, 2)));
    joints_q_dot = sym(zeros(1, size(joint_params, 2)));
    joints_q_dot_dot = sym(zeros(1, size(joint_params, 2)));
    joints_type = sym(zeros(1, size(joint_params, 2)));
    links_length = sym(zeros(1, size(link_params, 2)));

    for i=1:size(joint_params, 2)
        joint_i = joint_params{i};
        joints_q(i) = joint_i.q;
        joints_type(i) = joint_i.type;
        joints_q_dot(i) = joint_i.q_dot;
        joints_q_dot_dot(i) = joint_i.q_dot_dot;
    end

    for i=1:size(link_params, 2)
        link_i = link_params{i};
        links_length(i) = link_i.length;
    end

end
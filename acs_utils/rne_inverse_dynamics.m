function [tau] = rne_inverse_dynamics(kin, gravity_base, links, joints_q, joints_q_dot, joints_q_dot_dot, f_ext)
    % forward: propagate link velocities and accelerations from i=0 to i=n
    % from joint q, q_dot, q_dot_dot and base link w0, w0_dot, compute:
    % wi, wi_dot, ..., from the base link to the ee
    
    % data structure
    n_links = size(links, 2);
    arr_w_i_i = sym(zeros(3, n_links)); % angular vel. of link i wrt frame i
    arr_w_i_i_d = sym(zeros(3, n_links)); % angular acc. of link i wrt frame i 
    arr_p_i_i_dd = sym(zeros(3, n_links)); % linear acc. of frame i wrt frame i
    arr_p_i_Ci_dd = sym(zeros(3, n_links)); % linear acc. of CoM i wrt frame i
    % initial conditions
    z_0 = [0; 0; 1];
    arr_w_i_i(:, 1) = 0; % angular vel. of frame base is 0
    arr_w_i_i_d(:, 1) = 0; % angular acc. of frame base is 
    arr_p_i_i_dd(:, 1) = -gravity_base; % =[0 0 +9.81] 
    arr_p_i_Ci_dd(:, 1) = 0;

    % i = index of links
    for i=2:n_links
        % link indexes 2, 3, 4
        % link 1 is 'base link' (w_0, w_0_dot, ...)

        % theta_i velocity and acceleration
        th_i_d = joints_q_dot(i);
        th_i_dd = joints_q_dot_dot(i);

        % previous link velocity and acceleration
        w_iprev_iprev = arr_w_i_i(:, i-1);
        w_iprev_iprev_d = arr_w_i_i_d(:, i-1);
        p_iprev_iprev_dd = arr_p_i_i_dd(:, i-1);    
        p_iprev_Ciprev_dd = arr_p_i_Ci_dd(:, i-1);    

        % line 6
        H_iprev_i = kin.H(i-1, i); % from i to i-1
        R_iprev_i = H_iprev_i(1:3, 1:3); % from i to i-1
        R_from_iprev_to_i = (R_iprev_i.'); % from i-1 to i

        % line 7: angular velocity of link i
        w_i_i = R_from_iprev_to_i * w_iprev_iprev + ... % angular velocity due to link_i-1 angular velocity
              + R_from_iprev_to_i * th_i_d * z_0; % angular velocity due to theta_i angular velocity
        % line 8: angular acceleration of link i
        w_i_i_d = R_from_iprev_to_i * w_iprev_iprev_d + ... % angular acceleration due to link_i-1 angular acceleration
                + R_from_iprev_to_i * (th_i_dd * z_0) + ... % angular acceleration due to theta_i
                + R_from_iprev_to_i * cross(th_i_d * w_iprev_iprev, z_0); % coriolis?
                % + R_from_iprev_to_i * (th_i_dd * z_0 + cross(th_i_d * w_iprev_iprev, z_0)); % original formula from slide

        % line 9: linear acceleration of link i
        frames_distance_wrt_i = 2 * links{i}.CoM_i_i;
        p_i_i_dd = R_from_iprev_to_i * p_iprev_iprev_dd + ...
                 + cross(w_i_i_d, frames_distance_wrt_i) + ...
                 + cross(w_i_i, cross(w_i_i, frames_distance_wrt_i));
        % line 10: linear acceleration of CoM of link i
        CoM_frame_distance_wrt_i = links{i}.CoM_i_i;
        p_i_Ci_dd = p_i_i_dd + ... 
            + cross(w_i_i_d, CoM_frame_distance_wrt_i) + ...
            + cross(w_i_i, cross(w_i_i, CoM_frame_distance_wrt_i));

        % save variable of this link
        arr_w_i_i(:, i) = w_i_i; 
        arr_w_i_i_d(:, i) = w_i_i_d; 
        arr_p_i_i_dd(:, i) = p_i_i_dd;
        arr_p_i_Ci_dd(:, i) = p_i_Ci_dd;
    end 

    % backward
    arr_force = sym(zeros(3, n_links));
    arr_torque = sym(zeros(3, n_links));
    arr_tau = sym(zeros(3, 1));

    h_e = f_ext; %zeros(6, 1);

    for i=n_links-1:-1:1
        % frames indixes = 3,2,1 = for loop index
        % link indexes = 4,3,2 = for loop index + 1

        % current link velocity and acceleration
        w_i_i = arr_w_i_i(:, i);
        w_i_i_d = arr_w_i_i_d(:, i);
        p_i_i_dd = arr_p_i_i_dd(:, i);    
        p_i_Ci_dd = arr_p_i_Ci_dd(:, i);   

        % next link force/torque
        % #links - 1 = #dof
        if i == n_links-1
            % if computing tau of last dof, get external forces
            % because there is no "next dof"
            % this is the recursion base case
            f_inext_inext = h_e(1:3, 1);
            torque_inext_inext = h_e(4:6, 1);
        else
            % else, get "next dof" forces (already computed)
            f_inext_inext = arr_force(:, i+1);
            torque_inext_inext = arr_torque(:, i+1);
        end

        % link to be considered (the one on the right of the joint frame)
        child_link = links{i+1}; % links{1+1} to skip base link

        % line 6
        H_i_inext = kin.H(i, i+1);
        R_i_inext = H_i_inext(1:3, 1:3);

        % line 7
        f_i_i = R_i_inext * f_inext_inext + child_link.link_mass * p_i_Ci_dd;

        % line 8
        frames_distance_wrt_i = 2 * child_link.CoM_i_i; 
        CoM_frame_distance_wrt_i = child_link.CoM_i_i;

        torque_i_i = cross(-f_i_i, frames_distance_wrt_i + CoM_frame_distance_wrt_i) + ...
            + R_i_inext * torque_inext_inext + ...
            + cross((R_i_inext * f_inext_inext), CoM_frame_distance_wrt_i) + ...
            + child_link.I_num * w_i_i + ...
            + cross(w_i_i, (child_link.I_num * w_i_i));

        % line 9
        H_iprev_i = kin.H(i-1, i);
        R_i_iprev = H_iprev_i(1:3, 1:3);
        %tau_i = (torque_i_i.')*(R_i_iprev.')*z_0;
        tau_i = (torque_i_i.')*z_0;

        % save variable of this link
        arr_force(:, i) = f_i_i;
        arr_torque(:, i) = torque_i_i;
        arr_tau(i, 1) = tau_i;
    end
    
    tau = arr_tau;
end
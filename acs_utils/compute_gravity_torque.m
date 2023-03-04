function [G] = compute_gravity_torque(links, gravity_base)
    % compute gravity contributions
    
    n_links = size(links, 2);
    
    G = sym(zeros(n_links, 1)); % 3x1 vector (one scalar value for each dof)

    for i=1:n_links
        g_i = 0;

        for j=1:n_links
            mass_j = links{j}.link_mass;

            partial_jacobian_j_pos = links{j}.partial_J;
            partial_jacobian_j_pos = partial_jacobian_j_pos(1:3, :); % position partial jacobian of link j

            jacob_lj_pi = partial_jacobian_j_pos(:, i); % contribution of i-th link to j-th link

            g_j = mass_j * gravity_base.' * jacob_lj_pi;
            g_i = g_i + g_j;
        end
        G(i, 1) = -g_i;
    end
    
    G = vpa(simplify(G));
end
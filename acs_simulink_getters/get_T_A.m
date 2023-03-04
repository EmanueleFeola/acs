function T_A = get_T_A(phi, theta)
    % T_A = [I | 0;
    %        0 | T(euler_angles)]

    T = [[0 -sin(phi), cos(phi)*sin(theta)]
         [0 cos(phi), sin(phi)*sin(theta)]
         [1,         0,          cos(theta)]];

    T_A = [eye(3, 3),   zeros(3, 3);
           zeros(3, 3), T];
end
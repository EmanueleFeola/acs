function [J_A] = get_JA(q)
    t1 = q(1, 1);
    t2 = q(2, 1);
    t3 = q(3, 1);

    % homogeneous transf. matrix from frame 0 (index 1) to ee frame (index 4)
    % remark: frame 0 is not base frame
    H_1_ee = [[cos(t1)*cos(t2)*cos(t3) - cos(t1)*sin(t2)*sin(t3), - cos(t1)*cos(t2)*sin(t3) - cos(t1)*cos(t3)*sin(t2),  sin(t1), (6*cos(t1))/5 + 2*cos(t1)*cos(t2) - (37*cos(t1)*sin(t2)*sin(t3))/10 + (37*cos(t1)*cos(t2)*cos(t3))/10]
        [cos(t2)*cos(t3)*sin(t1) - sin(t1)*sin(t2)*sin(t3), - cos(t2)*sin(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2), -cos(t1), (6*sin(t1))/5 + 2*cos(t2)*sin(t1) - (37*sin(t1)*sin(t2)*sin(t3))/10 + (37*cos(t2)*cos(t3)*sin(t1))/10]
        [                cos(t2)*sin(t3) + cos(t3)*sin(t2),                   cos(t2)*cos(t3) - sin(t2)*sin(t3),        0,                                         2*sin(t2) + (37*cos(t2)*sin(t3))/10 + (37*cos(t3)*sin(t2))/10]
        [                                                0,                                                   0,        0,                                                                                                     1]];
    rot = double(H_1_ee(1:3, 1:3));
    
    % convert rot matrix (3x3) to minimal representation with euler angles (3x1)
    euler_angles = rotm2eul(rot, 'ZYZ');
    phi   = euler_angles(1);
    theta = euler_angles(2);
    psi   = euler_angles(3); % not used

    % previously computed kin.J_A_1_ee
    J_A = [
        [-(sin(t1)*(37*cos(t2 + t3) + 20*cos(t2) + 12))/10, -cos(t1)*((37*sin(t2 + t3))/10 + 2*sin(t2)),         -(37*sin(t2 + t3)*cos(t1))/10]
        [ (cos(t1)*(37*cos(t2 + t3) + 20*cos(t2) + 12))/10, -sin(t1)*((37*sin(t2 + t3))/10 + 2*sin(t2)),         -(37*sin(t2 + t3)*sin(t1))/10]
        [                                                0,            (37*cos(t2 + t3))/10 + 2*cos(t2),                  (37*cos(t2 + t3))/10]
        [                                                1,       (sin(phi - t1)*cos(theta))/sin(theta), (sin(phi - t1)*cos(theta))/sin(theta)]
        [                                                0,                              -cos(phi - t1),                        -cos(phi - t1)]
        [                                                0,                   -sin(phi - t1)/sin(theta),             -sin(phi - t1)/sin(theta)]
            ];

%     J_A = J_A(1:3, :);
end


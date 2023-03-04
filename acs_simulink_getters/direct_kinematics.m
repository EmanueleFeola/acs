function [x] = direct_kinematics(q)
    % given joint configuration q
    % returns the ee pose in the operational space

    t1 = q(1, 1);
    t2 = q(2, 1);
    t3 = q(3, 1);

    % copy paste from kinematics computed before
    % kin.H(1, 4);
    H = [[cos(t1)*cos(t2)*cos(t3) - cos(t1)*sin(t2)*sin(t3), - cos(t1)*cos(t2)*sin(t3) - cos(t1)*cos(t3)*sin(t2),  sin(t1), (6*cos(t1))/5 + 2*cos(t1)*cos(t2) - (37*cos(t1)*sin(t2)*sin(t3))/10 + (37*cos(t1)*cos(t2)*cos(t3))/10]
        [cos(t2)*cos(t3)*sin(t1) - sin(t1)*sin(t2)*sin(t3), - cos(t2)*sin(t1)*sin(t3) - cos(t3)*sin(t1)*sin(t2), -cos(t1), (6*sin(t1))/5 + 2*cos(t2)*sin(t1) - (37*sin(t1)*sin(t2)*sin(t3))/10 + (37*cos(t2)*cos(t3)*sin(t1))/10]
        [                cos(t2)*sin(t3) + cos(t3)*sin(t2),                   cos(t2)*cos(t3) - sin(t2)*sin(t3),        0,                                         2*sin(t2) + (37*cos(t2)*sin(t3))/10 + (37*cos(t3)*sin(t2))/10]
        [                                                0,                                                   0,        0,                                                                                                     1]];

    translation_vector = H(1:3, 4);
    rot_matrix = H(1:3, 1:3);

    x_p = translation_vector;
    x_o = rotm2eul(rot_matrix, 'ZYZ');

    x = [x_p; x_o'];
end


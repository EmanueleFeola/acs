function [J_A_dot] = get_JA_dot(q, dq)
    syms t real
    syms t1 t2 t3 real
    syms dt1 dt2 dt3 real
    t1(t) = symfun(str2sym('t1(t)'), t);
    t2(t) = symfun(str2sym('t2(t)'), t);
    t3(t) = symfun(str2sym('t3(t)'), t);
% test values
%     q = [pi/3; pi/4; pi/6];
%     dq = [1; 2; 3];
    % this is different from the matrix in get_JA
    % here the joint variables are a function of time    
    H_1_ee = [[cos(t1(t))*cos(t2(t))*cos(t3(t)) - cos(t1(t))*sin(t2(t))*sin(t3(t)), - cos(t1(t))*cos(t2(t))*sin(t3(t)) - cos(t1(t))*cos(t3(t))*sin(t2(t)),  sin(t1(t)), (6*cos(t1(t)))/5 + 2*cos(t1(t))*cos(t2(t)) - (37*cos(t1(t))*sin(t2(t))*sin(t3(t)))/10 + (37*cos(t1(t))*cos(t2(t))*cos(t3(t)))/10]
    [cos(t2(t))*cos(t3(t))*sin(t1(t)) - sin(t1(t))*sin(t2(t))*sin(t3(t)), - cos(t2(t))*sin(t1(t))*sin(t3(t)) - cos(t3(t))*sin(t1(t))*sin(t2(t)), -cos(t1(t)), (6*sin(t1(t)))/5 + 2*cos(t2(t))*sin(t1(t)) - (37*sin(t1(t))*sin(t2(t))*sin(t3(t)))/10 + (37*cos(t2(t))*cos(t3(t))*sin(t1(t)))/10]
    [                cos(t2(t))*sin(t3(t)) + cos(t3(t))*sin(t2(t)),                   cos(t2(t))*cos(t3(t)) - sin(t2(t))*sin(t3(t)),        0,                                         2*sin(t2(t)) + (37*cos(t2(t))*sin(t3(t)))/10 + (37*cos(t3(t))*sin(t2(t)))/10]
    [                                                0,                                                   0,        0,                                                                                                     1]];
    
    H_1_ee_dot = diff(H_1_ee, t);
    H_1_ee_dot = subs(H_1_ee_dot, [diff(t1(t), t), diff(t2(t), t), diff(t3(t), t)], [dq(1), dq(2), dq(3)]);
    H_1_ee_dot = subs(H_1_ee_dot, [t1(t), t2(t), t3(t)], [q(1), q(2), q(3)]);
    rot = double(H_1_ee_dot(1:3, 1:3));

    % convert rot matrix (3x3) to minimal representation with euler angles (3x1)
    euler_angles = rotm2eul(rot, 'ZYZ');
    phi   = euler_angles(1);
    theta = euler_angles(2);
    psi   = euler_angles(3); % not used

    % this is different from the matrix in get_JA
    % here the joint variables are a function of time
    J_A = [
    [-(sin(t1(t))*(37*cos(t2(t) + t3(t)) + 20*cos(t2(t)) + 12))/10, -cos(t1(t))*((37*sin(t2(t) + t3(t)))/10 + 2*sin(t2(t))),         -(37*sin(t2(t) + t3(t))*cos(t1(t)))/10]
    [ (cos(t1(t))*(37*cos(t2(t) + t3(t)) + 20*cos(t2(t)) + 12))/10, -sin(t1(t))*((37*sin(t2(t) + t3(t)))/10 + 2*sin(t2(t))),         -(37*sin(t2(t) + t3(t))*sin(t1(t)))/10]
    [                                                0,            (37*cos(t2(t) + t3(t)))/10 + 2*cos(t2(t)),                  (37*cos(t2(t) + t3(t)))/10]
    [                                                1,       (sin(phi - t1(t))*cos(theta))/sin(theta), (sin(phi - t1(t))*cos(theta))/sin(theta)]
    [                                                0,                              -cos(phi - t1(t)),                        -cos(phi - t1(t))]
    [                                                0,                   -sin(phi - t1(t))/sin(theta),             -sin(phi - t1(t))/sin(theta)]
        ];

    J_A_dot = diff(J_A,t);
    J_A_dot = subs(J_A_dot, [diff(t1(t), t), diff(t2(t), t), diff(t3(t), t)], [dq(1), dq(2), dq(3)]);
    J_A_dot = subs(J_A_dot, [t1(t), t2(t), t3(t)], [q(1), q(2), q(3)]);
end


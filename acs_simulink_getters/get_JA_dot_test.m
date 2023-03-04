function [J_A_dot] = get_JA_dot_test(q, dq)
    th1 = q(1);
    th2 = q(2);
    th3 = q(3);
    dt1 = dq(1);
    dt2 = dq(2);
    dt3 = dq(3);

    % compute phi, theta
    H_1_ee = [
        [cos(th1)*cos(th2)*cos(th3) - cos(th1)*sin(th2)*sin(th3), - cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2),  sin(th1), (6*cos(th1))/5 + 2*cos(th1)*cos(th2) - (37*cos(th1)*sin(th2)*sin(th3))/10 + (37*cos(th1)*cos(th2)*cos(th3))/10]
        [cos(th2)*cos(th3)*sin(th1) - sin(th1)*sin(th2)*sin(th3), - cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2), -cos(th1), (6*sin(th1))/5 + 2*cos(th2)*sin(th1) - (37*sin(th1)*sin(th2)*sin(th3))/10 + (37*cos(th2)*cos(th3)*sin(th1))/10]
        [                cos(th2)*sin(th3) + cos(th3)*sin(th2),                   cos(th2)*cos(th3) - sin(th2)*sin(th3),        0,                                         2*sin(th2) + (37*cos(th2)*sin(th3))/10 + (37*cos(th3)*sin(th2))/10]
        [                                                0,                                                   0,        0,                                                                                                     1]
    ];
    rot = double(H_1_ee(1:3, 1:3));
    euler_angles = rotm2eul(rot, 'ZYZ');
    phi   = euler_angles(1);
    theta = euler_angles(2);

    % compute phi dot, theta dot
    rot_dot = [
        [dt1^2*cos(th1)*sin(th2)*sin(th3) - dt2^2*cos(th1)*cos(th2)*cos(th3) - dt3^2*cos(th1)*cos(th2)*cos(th3) - dt1^2*cos(th1)*cos(th2)*cos(th3) + dt2^2*cos(th1)*sin(th2)*sin(th3) + dt3^2*cos(th1)*sin(th2)*sin(th3) - 2*dt2*dt3*cos(th1)*cos(th2)*cos(th3) + 2*dt1*dt2*cos(th2)*sin(th1)*sin(th3) + 2*dt1*dt2*cos(th3)*sin(th1)*sin(th2) + 2*dt1*dt3*cos(th2)*sin(th1)*sin(th3) + 2*dt1*dt3*cos(th3)*sin(th1)*sin(th2) + 2*dt2*dt3*cos(th1)*sin(th2)*sin(th3), dt1^2*cos(th1)*cos(th2)*sin(th3) + dt1^2*cos(th1)*cos(th3)*sin(th2) + dt2^2*cos(th1)*cos(th2)*sin(th3) + dt2^2*cos(th1)*cos(th3)*sin(th2) + dt3^2*cos(th1)*cos(th2)*sin(th3) + dt3^2*cos(th1)*cos(th3)*sin(th2) + 2*dt1*dt2*cos(th2)*cos(th3)*sin(th1) + 2*dt1*dt3*cos(th2)*cos(th3)*sin(th1) + 2*dt2*dt3*cos(th1)*cos(th2)*sin(th3) + 2*dt2*dt3*cos(th1)*cos(th3)*sin(th2) - 2*dt1*dt2*sin(th1)*sin(th2)*sin(th3) - 2*dt1*dt3*sin(th1)*sin(th2)*sin(th3), -dt1^2*sin(th1)]
        [dt1^2*sin(th1)*sin(th2)*sin(th3) - dt2^2*cos(th2)*cos(th3)*sin(th1) - dt3^2*cos(th2)*cos(th3)*sin(th1) - dt1^2*cos(th2)*cos(th3)*sin(th1) + dt2^2*sin(th1)*sin(th2)*sin(th3) + dt3^2*sin(th1)*sin(th2)*sin(th3) - 2*dt1*dt2*cos(th1)*cos(th2)*sin(th3) - 2*dt1*dt2*cos(th1)*cos(th3)*sin(th2) - 2*dt1*dt3*cos(th1)*cos(th2)*sin(th3) - 2*dt1*dt3*cos(th1)*cos(th3)*sin(th2) - 2*dt2*dt3*cos(th2)*cos(th3)*sin(th1) + 2*dt2*dt3*sin(th1)*sin(th2)*sin(th3), dt1^2*cos(th2)*sin(th1)*sin(th3) + dt1^2*cos(th3)*sin(th1)*sin(th2) + dt2^2*cos(th2)*sin(th1)*sin(th3) + dt2^2*cos(th3)*sin(th1)*sin(th2) + dt3^2*cos(th2)*sin(th1)*sin(th3) + dt3^2*cos(th3)*sin(th1)*sin(th2) - 2*dt1*dt2*cos(th1)*cos(th2)*cos(th3) - 2*dt1*dt3*cos(th1)*cos(th2)*cos(th3) + 2*dt1*dt2*cos(th1)*sin(th2)*sin(th3) + 2*dt1*dt3*cos(th1)*sin(th2)*sin(th3) + 2*dt2*dt3*cos(th2)*sin(th1)*sin(th3) + 2*dt2*dt3*cos(th3)*sin(th1)*sin(th2),  dt1^2*cos(th1)]
        [                                                                                                                                                                                                                                                                                      - dt2^2*cos(th2)*sin(th3) - dt2^2*cos(th3)*sin(th2) - dt3^2*cos(th2)*sin(th3) - dt3^2*cos(th3)*sin(th2) - 2*dt2*dt3*cos(th2)*sin(th3) - 2*dt2*dt3*cos(th3)*sin(th2),                                                                                                                                                                                                                                                                                         dt2^2*sin(th2)*sin(th3) - dt3^2*cos(th2)*cos(th3) - dt2^2*cos(th2)*cos(th3) + dt3^2*sin(th2)*sin(th3) - 2*dt2*dt3*cos(th2)*cos(th3) + 2*dt2*dt3*sin(th2)*sin(th3),               0]
    ];
    euler_angles = rotm2eul(rot_dot, 'ZYZ');
    dphi   = euler_angles(1);
    dtheta = euler_angles(2);

    J_A_dot = [
        [  (sin(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*cos(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                  dt1*sin(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)) - cos(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)),                                                                    (37*dt1*sin(th2 + th3)*sin(th1))/10 - (37*cos(th2 + th3)*cos(th1)*(dt2 + dt3))/10]
        [- (cos(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*sin(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                - sin(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)) - dt1*cos(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)),                                                                  - (37*cos(th2 + th3)*sin(th1)*(dt2 + dt3))/10 - (37*dt1*sin(th2 + th3)*cos(th1))/10]
        [                                                                                                                           0,                                                                                                - (37*sin(th2 + th3)*(dt2 + dt3))/10 - 2*dt2*sin(th2),                                                                                                                  -(37*sin(th2 + th3)*(dt2 + dt3))/10]
        [                                                                                                                           0, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2]
        [                                                                                                                           0,                                                                                                                      -sin(th1 - phi)*(dphi - dt1),                                                                                                                      -sin(th1 - phi)*(dphi - dt1)]
        [                                                                                                                           0,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2] 
    ];

end

% syms t real
% syms th1 th2 th3 real
% syms dt1 dt2 dt3 dphi dtheta real
% t1(t) = symfun(str2sym('t1(t)'), t);
% t2(t) = symfun(str2sym('t2(t)'), t);
% t3(t) = symfun(str2sym('t3(t)'), t);
% phi(t) = symfun(str2sym('phi(t)'), t);
% theta(t) = symfun(str2sym('theta(t)'), t);
% 
% H_1_ee = [[cos(t1(t))*cos(t2(t))*cos(t3(t)) - cos(t1(t))*sin(t2(t))*sin(t3(t)), - cos(t1(t))*cos(t2(t))*sin(t3(t)) - cos(t1(t))*cos(t3(t))*sin(t2(t)),  sin(t1(t)), (6*cos(t1(t)))/5 + 2*cos(t1(t))*cos(t2(t)) - (37*cos(t1(t))*sin(t2(t))*sin(t3(t)))/10 + (37*cos(t1(t))*cos(t2(t))*cos(t3(t)))/10]
%     [cos(t2(t))*cos(t3(t))*sin(t1(t)) - sin(t1(t))*sin(t2(t))*sin(t3(t)), - cos(t2(t))*sin(t1(t))*sin(t3(t)) - cos(t3(t))*sin(t1(t))*sin(t2(t)), -cos(t1(t)), (6*sin(t1(t)))/5 + 2*cos(t2(t))*sin(t1(t)) - (37*sin(t1(t))*sin(t2(t))*sin(t3(t)))/10 + (37*cos(t2(t))*cos(t3(t))*sin(t1(t)))/10]
%     [                cos(t2(t))*sin(t3(t)) + cos(t3(t))*sin(t2(t)),                   cos(t2(t))*cos(t3(t)) - sin(t2(t))*sin(t3(t)),        0,                                         2*sin(t2(t)) + (37*cos(t2(t))*sin(t3(t)))/10 + (37*cos(t3(t))*sin(t2(t)))/10]
%     [                                                0,                                                   0,        0,                                                                                                     1]];
% rot = H_1_ee(1:3, 1:3);
% diff_rot = diff(rot, t);
% 
% diff_rot = diff(diff_rot, t);
% diff_rot = subs(diff_rot, diff(t1(t), t), dt1);
% diff_rot = subs(diff_rot, diff(t2(t), t), dt2);
% diff_rot = subs(diff_rot, diff(t3(t), t), dt3);
% diff_rot = subs(diff_rot, diff(phi(t), t), dphi);
% diff_rot = subs(diff_rot, diff(theta(t), t), dtheta);
% diff_rot = subs(diff_rot, t1(t), th1);
% diff_rot = subs(diff_rot, t2(t), th2);
% diff_rot = subs(diff_rot, t3(t), th3);
% 
% euler_angles = rotm2eul(diff_rot, 'ZYZ');
% phi   = euler_angles(1);
% theta = euler_angles(2);
% psi   = euler_angles(3); % not used
% 
% ja = [
%     [-(sin(t1(t))*(37*cos(t2(t) + t3(t)) + 20*cos(t2(t)) + 12))/10, -cos(t1(t))*((37*sin(t2(t) + t3(t)))/10 + 2*sin(t2(t))),         -(37*sin(t2(t) + t3(t))*cos(t1(t)))/10]
%     [ (cos(t1(t))*(37*cos(t2(t) + t3(t)) + 20*cos(t2(t)) + 12))/10, -sin(t1(t))*((37*sin(t2(t) + t3(t)))/10 + 2*sin(t2(t))),         -(37*sin(t2(t) + t3(t))*sin(t1(t)))/10]
%     [                                                0,            (37*cos(t2(t) + t3(t)))/10 + 2*cos(t2(t)),                  (37*cos(t2(t) + t3(t)))/10]
%     [                                                1,       (sin(phi(t) - t1(t))*cos(theta(t)))/sin(theta(t)), (sin(phi(t) - t1(t))*cos(theta(t)))/sin(theta(t))]
%     [                                                0,                              -cos(phi(t) - t1(t)),                        -cos(phi(t) - t1(t))]
%     [                                                0,                   -sin(phi(t) - t1(t))/sin(theta(t)),             -sin(phi(t) - t1(t))/sin(theta(t))]
%     ];
% diff_ja = diff(ja, t);
% diff_ja = subs(diff_ja, diff(t1(t), t), dt1);
% diff_ja = subs(diff_ja, diff(t2(t), t), dt2);
% diff_ja = subs(diff_ja, diff(t3(t), t), dt3);
% diff_ja = subs(diff_ja, diff(phi(t), t), dphi);
% diff_ja = subs(diff_ja, diff(theta(t), t), dtheta);
% diff_ja = subs(diff_ja, t1(t), th1);
% diff_ja = subs(diff_ja, t2(t), th2);
% diff_ja = subs(diff_ja, t3(t), th3);

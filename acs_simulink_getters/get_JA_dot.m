% function [J_A_dot] = get_JA_dot(q, dq)
%     th1 = q(1);
%     th2 = q(2);
%     th3 = q(3);
%     dt1 = dq(1);
%     dt2 = dq(2);
%     dt3 = dq(3);
% 
%     % compute phi, theta
%     H_1_ee = [
%         [cos(th1)*cos(th2)*cos(th3) - cos(th1)*sin(th2)*sin(th3), - cos(th1)*cos(th2)*sin(th3) - cos(th1)*cos(th3)*sin(th2),  sin(th1), (6*cos(th1))/5 + 2*cos(th1)*cos(th2) - (37*cos(th1)*sin(th2)*sin(th3))/10 + (37*cos(th1)*cos(th2)*cos(th3))/10]
%         [cos(th2)*cos(th3)*sin(th1) - sin(th1)*sin(th2)*sin(th3), - cos(th2)*sin(th1)*sin(th3) - cos(th3)*sin(th1)*sin(th2), -cos(th1), (6*sin(th1))/5 + 2*cos(th2)*sin(th1) - (37*sin(th1)*sin(th2)*sin(th3))/10 + (37*cos(th2)*cos(th3)*sin(th1))/10]
%         [                cos(th2)*sin(th3) + cos(th3)*sin(th2),                   cos(th2)*cos(th3) - sin(th2)*sin(th3),        0,                                         2*sin(th2) + (37*cos(th2)*sin(th3))/10 + (37*cos(th3)*sin(th2))/10]
%         [                                                0,                                                   0,        0,                                                                                                     1]
%     ];
%     rot = double(H_1_ee(1:3, 1:3));
%     euler_angles = rotm2eul(rot, 'ZYZ');
%     phi   = euler_angles(1);
%     theta = euler_angles(2);
% 
%     % compute phi dot, theta dot
%     rot_dot = [
%         [dt1^2*cos(th1)*sin(th2)*sin(th3) - dt2^2*cos(th1)*cos(th2)*cos(th3) - dt3^2*cos(th1)*cos(th2)*cos(th3) - dt1^2*cos(th1)*cos(th2)*cos(th3) + dt2^2*cos(th1)*sin(th2)*sin(th3) + dt3^2*cos(th1)*sin(th2)*sin(th3) - 2*dt2*dt3*cos(th1)*cos(th2)*cos(th3) + 2*dt1*dt2*cos(th2)*sin(th1)*sin(th3) + 2*dt1*dt2*cos(th3)*sin(th1)*sin(th2) + 2*dt1*dt3*cos(th2)*sin(th1)*sin(th3) + 2*dt1*dt3*cos(th3)*sin(th1)*sin(th2) + 2*dt2*dt3*cos(th1)*sin(th2)*sin(th3), dt1^2*cos(th1)*cos(th2)*sin(th3) + dt1^2*cos(th1)*cos(th3)*sin(th2) + dt2^2*cos(th1)*cos(th2)*sin(th3) + dt2^2*cos(th1)*cos(th3)*sin(th2) + dt3^2*cos(th1)*cos(th2)*sin(th3) + dt3^2*cos(th1)*cos(th3)*sin(th2) + 2*dt1*dt2*cos(th2)*cos(th3)*sin(th1) + 2*dt1*dt3*cos(th2)*cos(th3)*sin(th1) + 2*dt2*dt3*cos(th1)*cos(th2)*sin(th3) + 2*dt2*dt3*cos(th1)*cos(th3)*sin(th2) - 2*dt1*dt2*sin(th1)*sin(th2)*sin(th3) - 2*dt1*dt3*sin(th1)*sin(th2)*sin(th3), -dt1^2*sin(th1)]
%         [dt1^2*sin(th1)*sin(th2)*sin(th3) - dt2^2*cos(th2)*cos(th3)*sin(th1) - dt3^2*cos(th2)*cos(th3)*sin(th1) - dt1^2*cos(th2)*cos(th3)*sin(th1) + dt2^2*sin(th1)*sin(th2)*sin(th3) + dt3^2*sin(th1)*sin(th2)*sin(th3) - 2*dt1*dt2*cos(th1)*cos(th2)*sin(th3) - 2*dt1*dt2*cos(th1)*cos(th3)*sin(th2) - 2*dt1*dt3*cos(th1)*cos(th2)*sin(th3) - 2*dt1*dt3*cos(th1)*cos(th3)*sin(th2) - 2*dt2*dt3*cos(th2)*cos(th3)*sin(th1) + 2*dt2*dt3*sin(th1)*sin(th2)*sin(th3), dt1^2*cos(th2)*sin(th1)*sin(th3) + dt1^2*cos(th3)*sin(th1)*sin(th2) + dt2^2*cos(th2)*sin(th1)*sin(th3) + dt2^2*cos(th3)*sin(th1)*sin(th2) + dt3^2*cos(th2)*sin(th1)*sin(th3) + dt3^2*cos(th3)*sin(th1)*sin(th2) - 2*dt1*dt2*cos(th1)*cos(th2)*cos(th3) - 2*dt1*dt3*cos(th1)*cos(th2)*cos(th3) + 2*dt1*dt2*cos(th1)*sin(th2)*sin(th3) + 2*dt1*dt3*cos(th1)*sin(th2)*sin(th3) + 2*dt2*dt3*cos(th2)*sin(th1)*sin(th3) + 2*dt2*dt3*cos(th3)*sin(th1)*sin(th2),  dt1^2*cos(th1)]
%         [                                                                                                                                                                                                                                                                                      - dt2^2*cos(th2)*sin(th3) - dt2^2*cos(th3)*sin(th2) - dt3^2*cos(th2)*sin(th3) - dt3^2*cos(th3)*sin(th2) - 2*dt2*dt3*cos(th2)*sin(th3) - 2*dt2*dt3*cos(th3)*sin(th2),                                                                                                                                                                                                                                                                                         dt2^2*sin(th2)*sin(th3) - dt3^2*cos(th2)*cos(th3) - dt2^2*cos(th2)*cos(th3) + dt3^2*sin(th2)*sin(th3) - 2*dt2*dt3*cos(th2)*cos(th3) + 2*dt2*dt3*sin(th2)*sin(th3),               0]
%     ];
%     euler_angles = rotm2eul(rot_dot, 'ZYZ');
%     dphi   = euler_angles(1);
%     dtheta = euler_angles(2);
% 
%     J_A_dot = [
%         [  (sin(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*cos(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                  dt1*sin(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)) - cos(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)),                                                                    (37*dt1*sin(th2 + th3)*sin(th1))/10 - (37*cos(th2 + th3)*cos(th1)*(dt2 + dt3))/10]
%         [- (cos(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*sin(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                - sin(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)) - dt1*cos(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)),                                                                  - (37*cos(th2 + th3)*sin(th1)*(dt2 + dt3))/10 - (37*dt1*sin(th2 + th3)*cos(th1))/10]
%         [                                                                                                                           0,                                                                                                - (37*sin(th2 + th3)*(dt2 + dt3))/10 - 2*dt2*sin(th2),                                                                                                                  -(37*sin(th2 + th3)*(dt2 + dt3))/10]
%         [                                                                                                                           0, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2]
%         [                                                                                                                           0,                                                                                                                      -sin(th1 - phi)*(dphi - dt1),                                                                                                                      -sin(th1 - phi)*(dphi - dt1)]
%         [                                                                                                                           0,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2] 
%     ];
% 
% end

function [J_A_dot] = get_JA_dot(q, dq)
    th1 = q(1);
    th2 = q(2);
    th3 = q(3);
    dt1 = dq(1);
    dt2 = dq(2);
    dt3 = dq(3);

    % compute euler angles phi, theta using rotm2eul
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

    % compute phi dot, theta dot using dx = JA(q) * dq
    dx = get_JA(q) * dq;
    % dphi   = dx(6);
    % dtheta = dx(5);
    
    JG = get_JG(q); % geometric jacobian
    w = JG(4:6, :) * dq;
    T_A = get_T_A(phi, theta);
    T = T_A(4:6, 4:6);
    deuler = pinv(T) * w;
    dphi   = deuler(1);
    dtheta = deuler(2);

    % sanity check of dx computation
    epsilon = 0.0000001;
    dx_diff = abs(dx(4:6) - deuler);
    if(sum(dx_diff) > epsilon)
        %fprintf("[get_JA_dot] dx difference: %.2f\n", sum(dx_diff));
    end

    % derivata del jacobiano analitico in cui i giunti e gli angoli di eulero sono funzione del tempo
    J_A_dot = [
        [  (sin(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*cos(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                  dt1*sin(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)) - cos(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)),                                                                    (37*dt1*sin(th2 + th3)*sin(th1))/10 - (37*cos(th2 + th3)*cos(th1)*(dt2 + dt3))/10]
        [- (cos(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*sin(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                - sin(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)) - dt1*cos(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)),                                                                  - (37*cos(th2 + th3)*sin(th1)*(dt2 + dt3))/10 - (37*dt1*sin(th2 + th3)*cos(th1))/10]
        [                                                                                                                           0,                                                                                                - (37*sin(th2 + th3)*(dt2 + dt3))/10 - 2*dt2*sin(th2),                                                                                                                  -(37*sin(th2 + th3)*(dt2 + dt3))/10]
        [                                                                                                                           0, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2]
        [                                                                                                                           0,                                                                                                                      -sin(th1 - phi)*(dphi - dt1),                                                                                                                      -sin(th1 - phi)*(dphi - dt1)]
        [                                                                                                                           0,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2] 
    ];

end

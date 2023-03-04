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
    dphi   = dx(4);
    dtheta = euler_angles(5);

    J_A_dot = [
        [  (sin(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*cos(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                  dt1*sin(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)) - cos(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)),                                                                    (37*dt1*sin(th2 + th3)*sin(th1))/10 - (37*cos(th2 + th3)*cos(th1)*(dt2 + dt3))/10]
        [- (cos(th1)*(37*sin(th2 + th3)*(dt2 + dt3) + 20*dt2*sin(th2)))/10 - (dt1*sin(th1)*(37*cos(th2 + th3) + 20*cos(th2) + 12))/10,                                - sin(th1)*((37*cos(th2 + th3)*(dt2 + dt3))/10 + 2*dt2*cos(th2)) - dt1*cos(th1)*((37*sin(th2 + th3))/10 + 2*sin(th2)),                                                                  - (37*cos(th2 + th3)*sin(th1)*(dt2 + dt3))/10 - (37*dt1*sin(th2 + th3)*cos(th1))/10]
        [                                                                                                                           0,                                                                                                - (37*sin(th2 + th3)*(dt2 + dt3))/10 - 2*dt2*sin(th2),                                                                                                                  -(37*sin(th2 + th3)*(dt2 + dt3))/10]
        [                                                                                                                           0, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2, dtheta*sin(th1 - phi) + (cos(theta)*cos(th1 - phi)*(dphi - dt1))/sin(theta) + (dtheta*cos(theta)^2*sin(th1 - phi))/sin(theta)^2]
        [                                                                                                                           0,                                                                                                                      -sin(th1 - phi)*(dphi - dt1),                                                                                                                      -sin(th1 - phi)*(dphi - dt1)]
        [                                                                                                                           0,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2,                                          - (cos(th1 - phi)*(dphi - dt1))/sin(theta) - (dtheta*cos(theta)*sin(th1 - phi))/sin(theta)^2] 
    ];

end

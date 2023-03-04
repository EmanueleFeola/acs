function [JG] = get_JG(q)
    % return geometric jacobian evaluated in q
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);

    JG = [
    [-(sin(t1)*(37*cos(t2 + t3) + 20*cos(t2) + 12))/10, -cos(t1)*((37*sin(t2 + t3))/10 + 2*sin(t2)), -(37*sin(t2 + t3)*cos(t1))/10]
    [ (cos(t1)*(37*cos(t2 + t3) + 20*cos(t2) + 12))/10, -sin(t1)*((37*sin(t2 + t3))/10 + 2*sin(t2)), -(37*sin(t2 + t3)*sin(t1))/10]
    [                                                0,            (37*cos(t2 + t3))/10 + 2*cos(t2),          (37*cos(t2 + t3))/10]
    [                                                0,                                     sin(t1),                       sin(t1)]
    [                                                0,                                    -cos(t1),                      -cos(t1)]
    [                                                1,                                           0,                             0]
    ];
end


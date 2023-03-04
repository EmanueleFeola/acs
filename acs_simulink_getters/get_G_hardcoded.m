function [G] = get_G_hardcoded(q)
    t1 = q(1);
    t2 = q(2);
    t3 = q(3);

    % gravity_base = [0; -9.81; 0];
    %     G = [0.0981*cos(t1)*(1517.0*cos(t2 + t3) + 2140.0*cos(t2) + 2184.0);
    %         -0.0981*sin(t1)*(1517.0*sin(t2 + t3) + 2140.0*sin(t2));
    %                                 -148.8177*sin(t2 + t3)*sin(t1)
    %         ];
    
    % gravity_base = [0; 0; -9.81];
    G = [0
        148.8177*cos(t2 + t3) + 209.934*cos(t2)
        148.8177*cos(t2 + t3)];
end
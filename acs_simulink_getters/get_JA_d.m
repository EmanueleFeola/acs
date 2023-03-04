% function [JA_d] = get_JA_d(q, Td, x_tilde)
%     JG = get_JG(q);
% 
%     R_d = eul2rotm(x_tilde(4:6)');
%     
%     Td_e = get_T_d_e(q, Td);
%     euler_angles_Td_e = rotm2eul(Td_e(1:3, 1:3));
%     phi   = euler_angles_Td_e(1);
%     theta = euler_angles_Td_e(2);
% 
%     R = [R_d' zeros(3,3); zeros(3,3) R_d'];
%     JA_d = pinv(get_T_A(phi, theta)) * R * JG;
% end

function [Jad] = get_JA_d(q, x_tilde, xd, xe) 
    % Rotation matrix obtained from xd (fixed) in regulation problem
    Rd = eul2rotm(-x_tilde(4:6)','ZYZ');
    angles = -x_tilde(4:6);
    
    % Compute Ta(phi_de) in this case ZYZ
    ph = -angles(1);
    th = -angles(2);
    Ta =   [1, 0, 0, 0, 0, 0;
            0, 1, 0, 0, 0, 0;
            0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, -sin(ph), cos(ph) * sin(th);
            0, 0, 0, 0, cos(ph), sin(ph) * sin(th);
            0, 0, 0, 1, 0, cos(th)];

    Jad = pinv(Ta)*[Rd', zeros(3,3); zeros(3,3), Rd']*get_JG(q);
end
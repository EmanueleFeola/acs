function sigma_d  = get_T_d(x_d)
    % given a pose expressed in cartesian coordinates
    % return 4x4 homog. transf. matrix
    
    R_d = eul2rotm(x_d(4:6)','ZYZ');
    o_d = x_d(1:3);
    sigma_d = [R_d,        o_d;
           zeros(1,3), 1];
end
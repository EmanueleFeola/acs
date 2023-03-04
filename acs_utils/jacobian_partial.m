function [JL] = jacobian_partial(kin, pL)
%JACOBIAN_PARTIAL compute partial Jacobians of each link given the
%homogeneous matrices and the position of the CoMs of each link  w.r.t. frame 0

z0 = [0; 0; 1];

JL = sym(zeros(6, 3, 3));
for i=1:kin.dofs
    % velocity of the i-th joint
    for j=1:i
        % contribution of the j-th joint on the i-th joint velocity (j < i)
        H0_j = kin.H(1, j);
        R0_j = H0_j(1:3,1:3);
        
        % is rotational joint
        d0_j = H0_j(1:3,4);
        JL(:,j,i) = [cross(R0_j * z0, pL - d0_j);
                     R0_j * z0];

    end
end

end
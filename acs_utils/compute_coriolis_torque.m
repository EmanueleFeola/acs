function [C] = compute_coriolis_torque(B, q, q_dot)
    % compute Coriolis contributions
    
    C = sym(zeros(size(B, 2)));

    for i=1:size(B, 2)
        for j=1:size(B, 2)
            for k=1:size(B, 2)
                % partial derivative of Bij wrt qk
                bij_qk = diff(B(i, j), q(k));
                % partial derivative of Bik wrt qj
                bik_qj = diff(B(i, k), q(j));
                % partial derivative of Bjk wrt qi
                bjk_qi = diff(B(j, k), q(i));

                c_i_j_k = (1/2) * (bij_qk + bik_qj - bjk_qi);
                C(i,j) = C(i,j) + c_i_j_k * q_dot(k);
            end
        end
    end
    
    C = vpa(simplify(C));
end


classdef Kinematics < handle
    properties
        dofs uint8 % number of dofs
        DH(:,4) sym % DH table
        
        % kinematics variables
        J_G(6, :) sym % symbolic geometric jacobian
        J_G_1_ee(6, :) sym % symbolic geometric jacobian
        J_A (6, :) sym % symbolic analitic jacobian with ZYZ euler angles
        J_A_1_ee (6, :) sym 
        pinv_JA sym % inverse of J_A       
        pinv_JA_transpose sym % inverse of J_A'
        
        H_map containers.Map % 'superscript_x_subscript_y'
        joint_vars(:, :) sym % list of joint variables
        joint_types(:, :) sym % list of joint types
        
        % dinamics variables
        B(3,:) sym % inertia matrix
        C(3,:) sym % corriolis matrix
        G(3,:) sym % gravity matrix
    end
    
    methods
        function this = Kinematics(DH, joint_vars, joint_types)
            this.DH = DH; % a,A,d,T
            this.dofs = size(joint_vars, 2);
            this.H_map = containers.Map('KeyType','char', 'ValueType','any');
            this.joint_vars = joint_vars;
            this.joint_types = joint_types;
            
            % compute jacobians (base to ee)
            H_0_ee = this.H(0, size(DH, 2));
            Pee = H_0_ee(1:3, 4); % Pee is the translation vector of the ee
            this.J_G = this.computeJG(size(DH, 2), Pee);            
            this.J_A = this.computeEulerJA();

            % compute jacobians (1 to ee)
            H_1_ee = this.H(1, size(DH, 2));
            Pee = H_1_ee(1:3, 4); % Pee is the translation vector of the ee
            this.J_G_1_ee = this.computeJG_test(size(DH, 2) - 1, Pee);            
            this.J_A_1_ee = this.computeEulerJA_test();
        end
    end
    
    methods (Access = public)
        %% homogenoeus transformations
        function [H] = H(this, superscript, subscript)
            % return homogeneous transf. matrix from subscript to superscript
            
            % if same frame, return identity
            if superscript == subscript
                H = eye(4);
                return;
            end
            
            map_key = join(['superscript_' num2str(superscript) '_subscript_' num2str(subscript)], '');
            
            if isKey(this.H_map, map_key)
                % fprintf("(%s) already computed\n", map_key);
                H = this.H_map(map_key);
                return;
            end
            
            start_index = superscript + 1;
            end_index = subscript;
            
            % sanity check of indexes
            if end_index > size(this.DH, 1)+1 || start_index > end_index
                ME = MException('myComponent:inputError', 'indexes are not in range start=%d, end=%d', start_index, end_index);
                throw(ME);
            end
            
            % fprintf("(%s) %d %d: %d %d\n", map_key, superscript, subscript, start_index, end_index);
            H = eye(4);
            for i=start_index:end_index
                d_i = this.DH(i, 3);
                theta_i = this.DH(i, 4);
                a_i = this.DH(i, 1);
                alpha_i = this.DH(i, 2);
                
                H = H * Kinematics.computeHomogeneous(a_i, alpha_i, d_i, theta_i);
            end
            
            % save the result for later use
            this.H_map(map_key) = H;
        end
        
        %% jacobians
        function [J_G] = computeJG(this, ndofs, Pee)
            % jacobian from 0 to ee
            % NB: not from base to ee
            J_G = sym(zeros(6, ndofs));
            
            % check frames correlated to dofs: frame 0, 1, 2
            for i=1:ndofs
                index_of_joint_frame = i-1; % the joint frame 1 has index 0, 2 has index 1, ...
                H_0_i = this.H(0, index_of_joint_frame);
                                               
                z = H_0_i(1:3, 1:3) * [0 0 1]';
                p = H_0_i(1:3, 4);
                
                if this.joint_types(i) == 1
                    J_pos = cross(z, (Pee - p));
                    J_angles = z;
                else
                    J_pos = z;
                    J_angles = [0 0 0]';
                end

                J_G(:, i) = [J_pos; J_angles];
            end
            
            J_G = simplify(J_G);
        end

        function [J_G] = computeJG_test(this, ndofs, Pee)
            % jacobian from 0 to ee
            % NB: not from base to ee
            J_G = sym(zeros(6, ndofs));
            
            % check frames correlated to dofs: frame 0, 1, 2
            for i=1:ndofs
                index_of_joint_frame = i; % the joint frame 1 has index 0, 2 has index 1, ...
                H_0_i = this.H(1, index_of_joint_frame);
                                               
                z = H_0_i(1:3, 1:3) * [0 0 1]';
                p = H_0_i(1:3, 4);
                
                if this.joint_types(i) == 1
                    J_pos = cross(z, (Pee - p));
                    J_angles = z;
                else
                    J_pos = z;
                    J_angles = [0 0 0]';
                end

                J_G(:, i) = [J_pos; J_angles];
            end
            
            J_G = simplify(J_G);
        end
        
        function [J_G] = computePartialJ(this, n, com)
            J_G = sym(zeros(6, 3));
            for i=1:n                
                for j=1:i
                    H = this.H(1, j);
                    z = H(1:3, 1:3) * [0 0 1]';
                    p = H(1:3, 4);
                    J_G(1:3, j) = cross(z, com - p);
                    J_G(4:6, j) = z;
                end
            end
        end
        
        function [J_A] = computeJA(this)
            % computed by differenttiation
            
            J_A = sym(zeros(6, this.dofs));
            H_b_ee = this.H(0, size(this.DH, 2));
            H_b_ee_transl = H_b_ee(1:3, 4);

            for i=1:3
                pos_trans = H_b_ee_transl(i);

                for j=1:size(this.joint_vars, 2)
                    J_A(i, j) = diff(pos_trans, this.joint_vars(j));
                end
            end
            
            J_A = simplify(J_A);
        end
        
        function [J_A] = computeEulerJA(this)
            % symbolic analytical jacobian computed as the
            % product: inv(Ta) * J_G
            
            Ta = Kinematics.computeTa();
            J_A = pinv(Ta)*this.J_G;
            J_A(:, 1) = [];
            J_A = simplify(J_A);
        end

        function [J_A] = computeEulerJA_test(this)
            % symbolic analytical jacobian computed as the
            % product: inv(Ta) * J_G
            
            Ta = Kinematics.computeTa();
            J_A = pinv(Ta)*this.J_G_1_ee;
            J_A = simplify(J_A);
        end
        
        function [JA_num, euler_angles] = evaluateEulerJA(this, q_vars, q_values)
            % returns the analytical jacobian evaluated with
            % the joint configuration q_values passed as a parameter
            
            % rot matrix from base to ee
            H_0_4 = this.H(0, 4);
            H_0_4_num = Kinematics.evalMatrix(H_0_4, q_vars, q_values, 5);
            rot = double(H_0_4_num(1:3, 1:3));
            
            % convert rot matrix (3x3) to minimal representation with euler angles (3x1)
            euler_angles = rotm2eul(rot, 'ZYZ');
            val.phi   = euler_angles(1);
            val.theta = euler_angles(2);
            val.psi   = euler_angles(3);
            
            % evaluate jacobian with q
            JA_num = Kinematics.evalMatrix(this.J_A, q_vars, q_values, 4);
            % evaluate jacobian with euler angles
            JA_num = double(subs(JA_num, val));
        end

        %% operational space
        % we have simplified expressions because the manipulator is non-redundant
        function [pinv_J_A] = getPinvJA(this)
            if ~isempty(this.pinv_JA)
                pinv_J_A = this.pinv_JA;
            else
                tic
                pinv_J_A = pinv(this.J_A);
                toc
                tic
                %pinv_J_A = simplify(pinv_J_A);
                toc
                this.pinv_JA = pinv_J_A;
            end
        end
        
        function [pinv_JA_transpose] = getPinvJA_T(this)
            if ~isempty(this.pinv_JA_transpose)
                pinv_JA_transpose = this.pinv_JA_transpose;
            else
                tic
                pinv_JA_transpose = pinv(this.J_A');
                toc
                tic
                %pinv_JA_transpose = simplify(pinv_JA_transpose);
                toc
                this.pinv_JA_transpose = pinv_JA_transpose;
            end
        end
        
        function [B_A] = computeBa(this)
            pinv_J_A = this.getPinvJA();
            pinv_J_A_transpose = this.getPinvJA_T();
            
            B_A = pinv_J_A_transpose * this.B * pinv_J_A;
            %B_A = simplify(B_A);
        end
        
        function [C_A] = computeCa_xd(this, B_A, q_dot)            
            pinv_J_A_transpose = this.getPinvJA_T();
            JA = this.J_A;
            JA_dot = diff(JA);
            
            C_A = pinv_J_A_transpose * this.C * q_dot - B_A * JA_dot * q_dot;
            %C_A = simplify(C_A);
        end
        
        function [G_A] = computeGa(this)
            pinv_J_A_transpose = this.getPinvJA_T();

            G_A = pinv_J_A_transpose * this.G;
            %G_A = simplify(G_A);
        end
        
        %% utils
        function [] = myPlot(this, links, q_vars, q_values)
            figure();
            hold on;
            axis padded

            dx = 0.01;
            
            for i=1:size(this.DH, 1)+1   
                i-1
                
                % position of frame i-1 wrt frame 0
                H_0_iminus1 = this.H(0, i-1); % homog. from i to 0
                
                frame_iminus1_0 = H_0_iminus1(1:3, 4); % symbolic position of frame i-1 wrt frame 0
                frame_iminus1_0 = subs(frame_iminus1_0, q_vars, q_values); % numerical position of frame i-1 wrt frame 0
                
                x = frame_iminus1_0(1);
                y = frame_iminus1_0(2);
                z = frame_iminus1_0(3);
                plot3(x, y, z, '-o', 'MarkerSize', 20, 'Color', 'r');
                text(x+dx, y+dx, z+dx, ['Σ_' num2str(i-1)],'FontSize',10);

                if i-1 == size(links, 2)
                    % return after plot of ee frame
                    % sigma i-1 = sigma 5-1 = sigma 4 = frame of ee
                    % there is no link i (link 5)
                    return;
                end
                
                % CoM_i position wrt frame 0
                CoM_i_0 = links{i}.CoM_i_0;
                CoM_i_0 = subs(CoM_i_0, q_vars, q_values); % numerical position of frame i-1 wrt frame 0
                
                x = CoM_i_0(1);
                y = CoM_i_0(2);
                z = CoM_i_0(3);
                plot3(x, y, z, '-o', 'Color', 'b', 'MarkerSize', 10);
                text(x+dx, y+dx, z+dx, ['CoM_' num2str(i)],'FontSize',10);
            end
            
        end
    end
    
    methods (Static, Access = private)       
        function [T] = computeHomogeneous(a_i, alpha_i, d_i, theta_i)       
            T = [
                 [cos(theta_i)   -sin(theta_i)*cos(alpha_i)  sin(theta_i)*sin(alpha_i)   a_i*cos(theta_i)]; 
                 [sin(theta_i)   cos(theta_i)*cos(alpha_i)   -cos(theta_i)*sin(alpha_i)  a_i*sin(theta_i)]; 
                 [   0           sin(alpha_i)                cos(alpha_i)                d_i    ]; 
                 [   0           0                           0                           1    ]
             ];
         
            T = simplify(T);
        end
    end
    
    methods (Static, Access = public)
        function [Ta] = computeTa()
            syms phi theta real;
            T = [0 -sin(phi) cos(phi)*sin(theta)
                 0 cos(phi)  sin(phi)*sin(theta)
                 1 0         cos(theta)];
            Ta = [eye(3)  zeros(3)
                 zeros(3) T];
        end
        
        function [M_num] = evalMatrix(M_sym, sym_vars, values, round_digits)
            % perform symbolic variable substitution (subs), simplify (simplify), resolve fractions (vpa)
            % result matrix is approximated with round_digits decimal digits
            % M_sym must be symbolic matrix
            
            M_num = subs(M_sym, sym_vars, values);
            M_num = simplify(M_num);
            M_num = vpa(M_num);
            M_num = round(M_num, round_digits);
        end
        
        function [M] = simplifyMatrix(M_sym)
            % perform simplify (simplify), resolve fractions (vpa)
            % M_sym must be symbolic matrix
        
            M = simplify(M_sym);
            M = vpa(M);
        end
                
    end
end
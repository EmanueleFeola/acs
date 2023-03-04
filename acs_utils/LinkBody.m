classdef LinkBody < handle
    properties
        link_mass double
        
        CoM_i_i sym % center of mass of this link expressed wrt "frame i"
        CoM_i_0 sym % center of mass of this link expressed wrt "frame 0". Used for energy
        
        H_0_i sym % homog. transf. superscript=0 subscript=i (H that brings frame i into 0 frame)
        
        I_num double % symbolic inertia tensor (wrt frame i)
        I_num_0_i sym % symbolic inertia tensor (wrt frame 0)
        B_sym sym % symbolic inertia matrix (wrt frame i)
        K_sym sym % symbolic kinetic energy (wrt frame i)
        U_sym sym % symbolic potential energy (wrt frame i)
        
        partial_J sym % partial jacobian of link i
    end
    
    methods
        function this = LinkBody()
            % empty constructor
        end
    end
    
    methods (Access = public)
        function [] = setCoM_i_i(this, CoM)
            % set the center of mass vector wrt frame i
            this.CoM_i_i = CoM;
        end
        
        function [] = setCoM_i_0(this, H_0_i)
            % set the center of mass vector wrt frame 0
            this.H_0_i = H_0_i; % store it for later use

            this.CoM_i_0 = H_0_i * [this.CoM_i_i; 1];
            this.CoM_i_0 = this.CoM_i_0(1:3);
            
            this.CoM_i_0 = simplify(this.CoM_i_0);
        end
        
        function [] = setPartialJacobian(this, J)
            this.partial_J = J;
        end
        
        function [] = applySteiner(this)
            % compute the inertia tensor wrt the frame i
            
            r = this.CoM_i_i;
            steiner_contribution = this.link_mass*(r.'*r * eye(3,3) - r*r.');
            I = this.I_num + steiner_contribution;
            
            I = simplify(I);
            
            this.I_num = I;
        end
        
        function [] = computeInertiaMatrix(this)
            % computes inertia matrix for this link given:
            % - J_link_i: the partial jacobian for this link
            % - R_0_i: the rotation matrix from "frame 0" to "frame i"
            % 
            % notes:
            % the inertia matrix is a property of the link
            % the inertia matrix depends on the configuration q
            %   - dependency on the configuration q is taken into account in:
            %       - the partial jacobian (J_p and J_o)
            %       - the rotation matrices (R_0_i)
            
            R_0_i = this.H_0_i(1:3, 1:3);
            this.I_num_0_i = R_0_i * this.I_num * R_0_i.'; % inertia tensor wrt frame 0

            m_link_i = this.link_mass;
            J_p = this.partial_J(1:3, :); % first 3 rows
            J_o = this.partial_J(4:6, :); % last 3 rows
            
            B_i = m_link_i * (J_p.') * J_p + (J_o.') * this.I_num_0_i * J_o;
            B_i = simplify(B_i);
            
            % syms('t2', 'real'); syms('t3', 'real'); syms('t4', 'real');
            % test = (J_p.') * J_p + (J_o.');
            % vpa(simplify(subs(test(3, 3), [t2, t3, t4], [-6, 2, 1])));
            
            this.B_sym = B_i;
        end
        
        function [] = computeKineticEnergy(this, q_dot)
            % computes kinetic energy of this link given generalized joint velocities
            
            % the parameter is passed as a row vector
            % respect slide notation by transforming it into a column vector
            q_dot = q_dot.';
            
            K = q_dot.' * this.B_sym * q_dot;
            K = (1/2) * K;
            K = simplify(K);
            
            this.K_sym = K;
        end
        
        function [] = computePotentialEnergy(this, gravity_base)
            % computes potential of this link energy given
            % - gravity vector expressed in base frame coordinates
            % - CoM of this link expressed in base frame coordinates
            
            U = this.link_mass * gravity_base.' * this.CoM_i_0;
            U = simplify(U);
            
            this.U_sym = U;
        end  
        
        function [I_num] = evaluateIsym(this, q_sym, q_num)
            I_num = subs(this.I_num, q_sym, q_num);            
        end
        
        function [] = flipAxis(this)
            % flip X and Z axis of inertia tensor
            I = this.I_num;
            old_xx = I(1, 1);
            old_zz = I(3, 3);
            
            this.I_num(1, 1) = old_zz;
            this.I_num(3, 3) = old_xx;
        end
    end
    
    methods (Abstract)
        computeInertiaTensor() % Abstract method without implementation
    end
end
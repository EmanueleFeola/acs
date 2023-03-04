% Compute the kinetic energy, Compute the potential energy
% prepara data structure for kinetic energy, potential energy and inertia matrix
K = 0;
U = 0;
B = zeros(3,3);

% build link objects, compute inertia tensor, assign CoM
links = cell(1, kin.dofs); % cell array with all the links
for i=2:size(DH, 2)
    link_params_i = link_params{i}; % skip base link
    
    % my links
    if kin.joint_types(i) == 1
        link_i = CylinderLink(link_params_i.name, link_params_i.mass, link_params_i.radius, link_params_i.length);
    else
        link_i = CuboidLink(link_params_i.name, link_params_i.mass, link_params_i.width, link_params_i.height, link_params_i.length);
    end
    
    % compute homogeneous matrix from frame i to frame 1 (not base frame)
    H_0_i = kin.H(1, i);

    % set CoM properties of the current link
    CoM_i_i = link_params_i.CoM;
    link_i.setCoM_i_i(CoM_i_i);
    link_i.setCoM_i_0(H_0_i);
    
    partial_jacobian_i = kin.computePartialJ(i-1, link_i.CoM_i_0);
    partial_jacobian_i = simplify(partial_jacobian_i);
    link_i.setPartialJacobian(partial_jacobian_i);
    
    links{i} = link_i;

%     link_params_i.name
%     disp(latex(sym(CoM_i_i)))
%     disp(latex(partial_jacobian_i))
end

for i=1:size(DH, 2) - 1
    link_i = links{i+1}; % skip base link (Link1)
    
    % compute inertia tensor
    link_i.computeInertiaTensor(); % inertia tensor of link i wrt CoM 
    link_i.applySteiner(); % inertia tensor of link i wrt frame i

    % inertia matrix, kinetic and potential energy
    link_i.computeInertiaMatrix();
    link_i.computeKineticEnergy(joints_q_dot(1, 2:end));
    link_i.computePotentialEnergy(gravity_base);
    
    B_i = link_i.B_sym;
    
    B = B + B_i; % total intertia matrix
    K = K + link_i.K_sym; % total kinetic energy
    U = U + link_i.U_sym; % total potenatial energy
end

U = -U;

B = simplify(vpa(B)); % inertia matrix
K = simplify(vpa(K)); % kinetic energy
U = simplify(vpa(U)); % potential energy

kin.B = B;

%% check max values of potential energy
f = @(t2,t3) 148.8177*sin(t2 + t3) + 209.934*sin(t2); % copy from symbolic value of U
x0 = [rand,rand]; % random initial conditions
[xmin, fval] = fminsearch(@(t) -f(t(1),t(2)), x0);
% xmin è xmax perchè sto minimizzando la funzione negata (quindi trovo il max e non il min)
show(robot, [0 pi/2 0]);
xlim([-1 1]);
ylim([-1 2]);
zlim([0 7]);
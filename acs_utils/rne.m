function [tau] = rne(kin, gravity_base, links, q, d_q, dd_q, f_ext)
z0 = [0 0 1]';
w0_0 = [0 0 0]';
d_w0_0 = [0 0 0]';
dd_p0_0 = -gravity_base;

H0_1 = kin.H(1,2);
H1_2 = kin.H(2,3);
H2_3 = kin.H(3,4);
H3_e = kin.H(4,5);

R0_1 = H0_1(1:3,1:3);
R1_2 = H1_2(1:3,1:3);
R2_3 = H2_3(1:3,1:3);
R3_e = H3_e(1:3,1:3);

% Distances links 
r1_01 = R0_1'*H0_1(1:3,4);
r2_12 = R1_2'*H1_2(1:3,4);
r3_23 = R2_3'*H2_3(1:3,4);
r{1} = r1_01;
r{2} = r2_12;
r{3} = r3_23;

% Distances CoM and links
r1_1c1 = links{1}.CoM_i_i;
r2_2c2 = links{2}.CoM_i_i;
r3_3c3 = links{3}.CoM_i_i;
coms{1} = r1_1c1;
coms{2} = r2_2c2;
coms{3} = r3_3c3;

arr_w_prev{1} = w0_0;
arr_d_w_prev{1} = d_w0_0;
arr_dd_p_prev{1} = dd_p0_0;

for i=1:3
    H = kin.H(i, i+1);
    R = H(1:3, 1:3);
    w_prev = arr_w_prev{i}; % should be i-1 but matlab array start at 1
    d_w_prev = arr_d_w_prev{i};
    dd_p_prev = arr_dd_p_prev{i};

    r_i = r{i}; 
    com_i = coms{i}; 
    
    w_i_i = simplify(R' * w_prev + R' *d_q(i) * z0);
    d_wi_i = simplify(R' * d_w_prev + R'*(dd_q(i)*z0+cross(d_q(i)*w_prev,z0)));
    dd_pi_i =  simplify(R' * dd_p_prev +cross(d_wi_i,r_i)+cross(w_i_i,cross(w_i_i,r_i)));
    dd_pi_ci =  simplify(dd_pi_i + cross(d_wi_i,com_i) + cross(w_i_i,cross(w_i_i,com_i)));
%     d_w_prev =  simplify(d_w_prev);
    
    arr_w_prev{i+1} = w_i_i;
    arr_d_w_prev{i+1} = d_wi_i;
    arr_dd_p_prev{i+1} = dd_pi_i;
    arr_dd_pc_prev{i+1} = dd_pi_ci;
end

f_e = [0;0;0];
mu_e = [0;0;0];
arr_f{4} = f_e;
arr_mu{4} = mu_e;

for i=4:-1:2
    H = kin.H(i, i+1);
    H_prev = kin.H(i-1, i);
    R = H(1:3, 1:3);
    R_prev = H_prev(1:3, 1:3);
    
    f_next = arr_f{i};
    mu_next = arr_mu{i};
    
    d_w_prev = arr_d_w_prev{i};
    dd_p_prev = arr_dd_pc_prev{i};
    IL = links{i-1}.I_num;
    w_prev = arr_w_prev{i};
    
    f = R*f_next+links{i-1}.link_mass*dd_p_prev;
    mu = -cross(f, (r{i-1}+coms{i-1}))+R*mu_next+cross(R*f_next,coms{i-1})+IL*d_w_prev+cross(w_prev, IL*w_prev);
    tau = mu'*R_prev'*z0;

    arr_tau{i-1} = tau;
    arr_f{i-1} = f;
    arr_mu{i-1} = mu;
end
tau = [arr_tau{1}; arr_tau{2}; arr_tau{3}];


clc

% assignment 1
% computes symbolic direct kinematics, geometric jacobian, analytical jacobian
% evaluates symbolic results with specific joint q configuration and compares the results with the toolbox results
% NB: prima esegui main poi esegui questo script

%% test direct kinematics (compared to robotics toolbox)
H_b_ee_numeric = kin.H(0, size(DH, 1)); 
H_b_ee_numeric = Kinematics.evalMatrix(H_b_ee_numeric, joints_q(1, 2:end), q_test, 10);

ee_pose = double(simplify(H_b_ee_numeric));
ee_pose_toolbox = getTransform(robot, robot_config, 'ee', 'Link1');

%% test jacobian (compared to robotics toolbox)
J_G_numeric = Kinematics.evalMatrix(kin.J_G, joints_q(1, 2:end), q_test, 4);

test = Kinematics.evalMatrix(kin.J_A_1_ee, joints_q(1, 2:end), q_test, 4);
Hb_0 = kin.H(0, 1);
Rb_0 = Hb_0(1:3,1:3);
TJb_0 = [    Rb_0  zeros(3); 
         zeros(3)      Rb_0];

test = TJb_0 * test;

J_G_toolbox = geometricJacobian(robot, robot_config, 'ee');
J_G_toolbox = [J_G_toolbox(4:6,:); J_G_toolbox(1:3,:)];

%% printing results
% disp("Given joint configuration");
% disp(q_test);
% 
% disp("My direct kinematics");
% disp(ee_pose);
% disp("Toolbox direct kinematics");
% disp(ee_pose_toolbox);
% 
% disp("Given joint configuration");
% disp(q_test);
% disp("My geometric jacobian");
% disp(J_G_numeric(:, 2:end));
% disp("Toolbox geometric jacobian");
% disp(J_G_toolbox);

% % disp(test(1:3, :));
% % disp("Geometric jacobian computed from analytical jacobian and transformation matrix");
% % disp(T_times_JA);
% % disp("My analytical jacobian");
% % disp(J_A_euler);





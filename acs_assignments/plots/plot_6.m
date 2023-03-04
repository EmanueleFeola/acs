%% plot simulink output
out_gravity_ok = importdata("out_6_gravity_ok.mat");

x_axis = out.qd.Time;
y_axis_1 = out_gravity_ok.q.data; % out.theta.data;
y_axis_2 = out.qd.data; % out.theta_hat.data;
y_axis_3 = out.q_const.data; % out.theta_hat.data;

y_axis_1 = squeeze(y_axis_1);
% y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);
y_axis_2 = y_axis_2;
y_axis_3 = squeeze(y_axis_3);

figure();
subplot(1,3,1);
hold on;
grid on;
% plot(x_axis, y_axis_1(:,1)*ones(size(out.theta_hat.data, 1)));
plot(x_axis, y_axis_1(:,1));
plot(x_axis, y_axis_2(:,1), 'r');
plot(x_axis, y_axis_3(:,1), 'b');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q 1', 'q desired 1', 'q 1 constant compensation');
% ylabel('x [m]');
% legend('x desired', 'x');
% ylabel('phi [rad]');
% legend('phi', 'phi desired');

subplot(1,3,2);
hold on;
grid on;
% plot(x_axis, y_axis_1(:,2)*ones(size(out.theta_hat.data, 1)));
plot(x_axis, y_axis_1(:,2));
plot(x_axis, y_axis_2(:,2), 'r');
plot(x_axis, y_axis_3(:,2), 'b');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q 2', 'q desired 2', 'q 2 constant compensation');
% ylabel('y [m]');
% legend('y desired', 'y');
% ylabel('theta [rad]');
% legend('theta', 'theta desired');

subplot(1,3,3);
hold on;
grid on;
% plot(x_axis, y_axis_1(:,3)*ones(size(out.theta_hat.data, 1)));
plot(x_axis, y_axis_1(:,3));
plot(x_axis, y_axis_2(:,3), 'r');
plot(x_axis, y_axis_3(:,3), 'b');
xlabel('time [s]');
ylabel('q [rad/s]');
legend('q 3', 'q desired 3', 'q 3 constant compensation');
% ylabel('z [m]');
% legend('z desired', 'z 3');
% ylabel('psi [rad]');
% legend('psi', 'psi desired');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
% close
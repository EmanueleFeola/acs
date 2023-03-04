%% plot simulink output
x_axis = out.q.Time;
y_axis_1 = out.q.data; % out.theta.data;
y_axis_2 = out.qd.data; % out.theta_hat.data;
y_axis_3 = out.theta.data;
y_axis_4 = out.theta_hat.data;

y_axis_1 = squeeze(y_axis_1);
% y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);
y_axis_2 = y_axis_2;

% figure();
% hold on;
% grid on;
% % plot(x_axis, y_axis_1(:,1)*ones(size(out.theta_hat.data, 1)));
% plot(x_axis, y_axis_1(:,1));
% plot(x_axis, y_axis_2(:,1), 'r');
% xlabel('time [s]');
% ylabel('q [rad/s]');
% legend('q 1', 'q desired 1');

figure();
subplot(1,3,1);
hold on;
grid on;
yline(y_axis_3(:,1));
plot(x_axis, y_axis_4(:,1), 'b');
xlabel('time [s]');
legend('theta 1', 'theta hat 1');

subplot(1,3,2);
hold on;
grid on;
yline(y_axis_3(:,2));
plot(x_axis, y_axis_4(:,2), 'b');
xlabel('time [s]');
legend('theta 2', 'theta hat 2');

subplot(1,3,3);
hold on;
grid on;
yline(y_axis_3(:,3));
plot(x_axis, y_axis_4(:,3), 'b');
xlabel('time [s]');
legend('theta 3', 'theta hat 3');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
% close
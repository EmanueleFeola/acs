%% plot simulink output
x_axis = out.x.Time;
y_axis_1 = out.x.data;
y_axis_2 = out.xd.data;

y_axis_1 = squeeze(y_axis_1);
y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);
y_axis_2 = y_axis_2;

figure();
subplot(1,3,1);
hold on;
grid on;
plot(x_axis, y_axis_2(:,1), 'black');
plot(x_axis, y_axis_1(:,1), 'b');
xlabel('time [s]');
ylabel('x [m]');
legend('x desired', 'x');
% ylabel('phi [rad]');
% legend('phi', 'phi desired');

subplot(1,3,2);
hold on;
grid on;
plot(x_axis, y_axis_2(:,2), 'black');
plot(x_axis, y_axis_1(:,2), 'b');
xlabel('time [s]');
ylabel('y [m]');
legend('y desired', 'y');
% ylabel('theta [rad]');
% legend('theta', 'theta desired');

subplot(1,3,3);
hold on;
grid on;
plot(x_axis, y_axis_2(:,3), 'black');
plot(x_axis, y_axis_1(:,3), 'b');
xlabel('time [s]');
ylabel('z [m]');
legend('z desired', 'z');
% ylabel('psi [rad]');
% legend('psi', 'psi desired');

set(gcf, 'Position', get(0, 'Screensize'));
% export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
% close
x_axis = out.x_ee.Time;
y_axis_1 = out.x_d.data;
y_axis_2 = out.x_ee.data;
y_axis_3 = out.x_rest_env.data;

y_axis_1 = squeeze(y_axis_1);
y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);
y_axis_2 = y_axis_2;
y_axis_3 = squeeze(y_axis_3);
y_axis_3 = y_axis_3;

%% plot 12
% position
figure();
subplot(3, 1, 1);
hold on;
grid on;
plot(x_axis, y_axis_1(1,:), 'b');
plot(x_axis, y_axis_2(1,:), 'r');
xlabel('time [s]');
ylabel('x [m]');
legend('x desired', 'x ee');

subplot(3, 1, 2);
hold on;
grid on;
plot(x_axis, y_axis_1(2,:), 'b');
plot(x_axis, y_axis_2(2,:), 'r');
xlabel('time [s]');
ylabel('y [m]');
legend('y desired', 'y ee');

subplot(3, 1, 3);
hold on;
grid on;
plot(x_axis, y_axis_1(3,:), 'b');
plot(x_axis, y_axis_2(3,:), 'r');
yline(y_axis_3(1, 3), 'g');
xlabel('time [s]');
ylabel('z [m]');
legend('z desired', 'z ee');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');

% orientation
figure();
subplot(3, 1, 1);
hold on;
grid on;
plot(x_axis, y_axis_1(4,:), 'b');
plot(x_axis, y_axis_2(4,:), 'r');
xlabel('time [s]');
ylabel('phi [m]');
legend('phi desired', 'phi ee');

subplot(3, 1, 2);
hold on;
grid on;
plot(x_axis, y_axis_1(5,:), 'b');
plot(x_axis, y_axis_2(5,:), 'r');
xlabel('time [s]');
ylabel('theta [m]');
legend('theta desired', 'theta ee');

subplot(3, 1, 3);
hold on;
grid on;
plot(x_axis, y_axis_1(6,:), 'b');
plot(x_axis, y_axis_2(6,:), 'r');
xlabel('time [s]');
ylabel('psi [m]');
legend('psi desired', 'psi ee');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
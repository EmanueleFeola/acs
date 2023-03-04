x_axis = out.fe.Time;
y_axis_1 = out.fd.data;
y_axis_2 = out.fe.data;
y_axis_3 = out.pi_out.data;
y_axis_4 = out.xee.data;

y_axis_1 = squeeze(y_axis_1);
y_axis_2 = squeeze(y_axis_2);
y_axis_3 = squeeze(y_axis_3);
y_axis_4 = squeeze(y_axis_4);
y_axis_4 = y_axis_4';

%% plot 13
% position
figure();
subplot(3, 1, 1);
hold on;
grid on;
yline(y_axis_1(1,1), 'b');
plot(x_axis, y_axis_2(1,:), 'r');
xlabel('time [s]');
ylabel('F [N]');
legend('fx desired', 'fx');

subplot(3, 1, 2);
hold on;
grid on;
yline(y_axis_1(1,2), 'b');
plot(x_axis, y_axis_2(2,:), 'r');
xlabel('time [s]');
ylabel('F [N]');
legend('fy desired', 'fy');

subplot(3, 1, 3);
hold on;
grid on;
yline(y_axis_1(1,3), 'b');
plot(x_axis, y_axis_2(3,:)', 'r');
xlabel('time [s]');
ylabel('F [N]');
legend('fz desired', 'fz');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');

% orientation
figure();
subplot(3, 1, 1);
hold on;
grid on;
plot(x_axis, y_axis_3(1,:), 'b');
plot(x_axis, y_axis_4(1,:), 'r');
xlabel('time [s]');
ylabel('x [m]');
legend('x ref', 'x');

subplot(3, 1, 2);
hold on;
grid on;
plot(x_axis, y_axis_3(2,:), 'b');
plot(x_axis, y_axis_4(2,:), 'r');
xlabel('time [s]');
ylabel('y [m]');
legend('y ref', 'y');

subplot(3, 1, 3);
hold on;
grid on;
plot(x_axis, y_axis_3(3,:), 'b');
plot(x_axis, y_axis_4(3,:), 'r');
xlabel('time [s]');
ylabel('z [m]');
legend('z ref', 'z');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
x_axis = out.wrench.Time;
y_axis_1 = out.wrench.data;
y_axis_2 = out.wrench_joint.data;

y_axis_1 = squeeze(y_axis_1);
% y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);
% y_axis_2 = y_axis_2';

%% plot 12
% position
figure();
subplot(2, 1, 1);
hold on;
grid on;
plot(x_axis, y_axis_1(1,:), 'r');
plot(x_axis, y_axis_1(2,:), 'g');
plot(x_axis, y_axis_1(3,:), 'b');
plot(x_axis, y_axis_1(4,:), 'b');
plot(x_axis, y_axis_1(5,:), 'b');
plot(x_axis, y_axis_1(6,:), 'b');
xlabel('time [s]');
ylabel('end-effector wrench [N]');
legend('x axis force', 'y axis force', 'z axis force');

subplot(2, 1, 2);
hold on;
grid on;
plot(x_axis, y_axis_2(1,:), 'r');
plot(x_axis, y_axis_2(2,:), 'g');
plot(x_axis, y_axis_2(3,:), 'b');
xlabel('time [s]');
ylabel('joint torques [Nm]');
legend('joint 1 torque', 'joint 2 torque', 'joint 3 torque');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
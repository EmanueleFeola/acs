%% plot simulink output
x_axis = out.y.Time;
y_axis_1 = out.y.data; % out.theta.data;
y_axis_2 = out.ddq.data; % out.theta.data;

y_axis_1 = squeeze(y_axis_1);
y_axis_1 = y_axis_1';
y_axis_2 = squeeze(y_axis_2);

figure();
subplot(1,3,1);
hold on;
grid on;
plot(x_axis, y_axis_1(:,1));
plot(x_axis, y_axis_2(:,1));
xlabel('time [s]');
legend('y 1', 'ddq 1');

subplot(1,3,2);
hold on;
grid on;
plot(x_axis, y_axis_1(:,2));
plot(x_axis, y_axis_2(:,2));
xlabel('time [s]');
legend('y 2', 'ddq 2');

subplot(1,3,3);
hold on;
grid on;
plot(x_axis, y_axis_1(:,3));
plot(x_axis, y_axis_2(:,3));
xlabel('time [s]');
legend('y 3', 'ddq 3');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
% close
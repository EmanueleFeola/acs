%% plot simulink output
x_axis = out.tau.Time;
y_axis_1 = out.tau.data; % out.theta.data;

y_axis_1 = squeeze(y_axis_1);
y_axis_1 = y_axis_1';


figure();
subplot(1,3,1);
hold on;
grid on;
% plot(x_axis, y_axis_1(:,1)*ones(size(out.theta_hat.data, 1)));
plot(x_axis, y_axis_1(:,1));
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau 1');

subplot(1,3,2);
hold on;
grid on;
% plot(x_axis, y_axis_1(:,2)*ones(size(out.theta_hat.data, 1)));
plot(x_axis, y_axis_1(:,2));
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau 2');

subplot(1,3,3);
hold on;
grid on;
plot(x_axis, y_axis_1(:,3));
xlabel('time [s]');
ylabel('tau [Nm]');
legend('tau 3');

set(gcf, 'Position', get(0, 'Screensize'));
export_fig(strcat('C:\Users\emanuele\Desktop\acs_report_images\', datestr(now,'dd_mm_yyyy_HH_MM_SS_FFF')), '-eps');
% close
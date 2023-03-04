robot = importrobot('../RRR_zxx_modified_lengths.urdf');
robot.DataFormat = 'row';

traj_data = out.q.Data;
time = out.q.Time;

for i=1:size(traj_data, 1)
    time_i = time(i);
    q_conf = [traj_data(i, 1), traj_data(i, 2), traj_data(i, 3)];
    show(robot, q_conf);
    pause(0.001);

    disp(i);
    disp(time_i);

    fprintf("q2 %.2f\n", traj_data(i, 2));
end

disp("end")

%%
return;
show(robot, q_conf);
hold on;
plot3(x_home(2), x_home(1), 1, 'x');


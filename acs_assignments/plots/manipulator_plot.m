% manipulator plot
robot = importrobot('../../RRR_zxx_modified_lengths_visual.urdf');
% robot = importrobot('../RRR_zxx.urdf');

robot.DataFormat = 'row';

figure();
config = [0, 0, 0];
subplot(2, 2, 1);
show(robot, config);
xlim([-0.5 0.5]);
ylim([-0.5 7]);
zlim([0 1.5]);
title(['joint variables (degree):' strjoin(string(rad2deg(config)), ',')]);

subplot(2, 2, 3);
config = [0, pi/2, 0];
show(robot, config);
xlim([-0.5 0.5]);
ylim([-0.5 1.5]);
zlim([0 7]);
title(['joint variables (degree):' strjoin(string(rad2deg(config)), ',')]);

subplot(2, 2, 2);
config = [0, 0, pi/2];
show(robot, config);
xlim([-0.5 0.5]);
ylim([-0.5 3.5]);
zlim([0 5]);
title(['joint variables (degree):' strjoin(string(rad2deg(config)), ',')]);

subplot(2, 2, 4);
config = [0, pi/2, pi/2];
show(robot, config);
xlim([-0.5 0.5]);
ylim([-3 2]);
zlim([-0.5 3.5]);
title(['joint variables (degree):' strjoin(string(rad2deg(config)), ',')]);

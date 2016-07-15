
load poses.mat

x_vo_0 = pose_0(:, 4);
y_vo_0 = pose_0(:, 8);
z_vo_0 = pose_0(:, 12);

x_vo_3 = pose_3(:, 4);
y_vo_3 = pose_3(:, 8);
z_vo_3 = pose_3(:, 12);

x_vo_6 = pose_6(:, 4);
y_vo_6 = pose_6(:, 8);
z_vo_6 = pose_6(:, 12);

x_vo_9 = pose_9(:, 4);
y_vo_9 = pose_9(:, 8);
z_vo_9 = pose_9(:, 12);

x_vo_12 = pose_12(:, 4);
y_vo_12 = pose_12(:, 8);
z_vo_12 = pose_12(:, 12);

figure(1), hold on,
l_0 = plot3(x_vo_0, y_vo_0, z_vo_0, 'r');
l_3 = plot3(x_vo_3, y_vo_3, z_vo_3, 'b');
l_6 = plot3(x_vo_6, y_vo_6, z_vo_6, 'g');
l_9 = plot3(x_vo_9, y_vo_9, z_vo_9, 'k');
l_12 = plot3(x_vo_12, y_vo_12, z_vo_12, 'm');
xlabel('X');
ylabel('Y');
zlabel('Z');
axis equal
legend([l_0, l_3, l_6, l_9, l_12], 'VO 0', 'VO 1', 'VO 2', 'VO 3', 'VO 4');
xlabel('X'); ylabel('Y'); zlabel('Z');

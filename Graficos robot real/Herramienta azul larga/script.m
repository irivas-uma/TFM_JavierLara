bag = rosbag('bagfile_2_001.bag');

ef_msgs = select(bag, 'Topic', 'iiwa_surgery/output/ef_pose');
tcp_msgs = select(bag, 'Topic', 'iiwa_surgery/output/tcp_pose');

length = min(ef_msgs.NumMessages, tcp_msgs.NumMessages);

ef_msg = readMessages(ef_msgs, 1:length, 'DataFormat', 'struct');
tcp_msg = readMessages(tcp_msgs, 1:length, 'DataFormat', 'struct');

xEF = cellfun(@(m) double(m.Pose.Position.X), ef_msg);
yEF = cellfun(@(m) double(m.Pose.Position.Y), ef_msg);
zEF = cellfun(@(m) double(m.Pose.Position.Z), ef_msg);

xTCP = cellfun(@(m) double(m.Pose.Position.X), tcp_msg);
yTCP = cellfun(@(m) double(m.Pose.Position.Y), tcp_msg);
zTCP = cellfun(@(m) double(m.Pose.Position.Z), tcp_msg);

% Fija los límites de los ejes
figure;

for i = 1:500:length
    plot3(xEF, yEF, zEF, '*');
    hold on;
    plot3(xTCP, yTCP, zTCP, 'm*');
    plot3([xEF(i) xTCP(i)], [yEF(i) yTCP(i)], [zEF(i) zTCP(i)], 'g-');
end

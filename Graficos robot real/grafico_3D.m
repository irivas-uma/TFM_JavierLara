bag = rosbag('bagfile_001_2.bag');

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
    plot3(xEF(i), yEF(i), zEF(i), 'ko');
    hold on;
    plot3(xTCP(i), yTCP(i), zTCP(i), 'ro');
    hold on;
    plot3([xEF(i) xTCP(i)], [yEF(i) yTCP(i)], [zEF(i) zTCP(i)], 'b-');
end

% Etiquetas y título del gráfico
xlabel('Eje X');
ylabel('Eje Y');
zlabel('Eje Z');
%title('Gráfico de Puntos 3D');

% Leyenda personalizada
legend('Posiciones EF', 'Posiciones TCP', 'Posiciones de la herramienta', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Mantener la relación de aspecto
%axis equal;

% Finalizar la configuración del gráfico
hold off;


clear all;
bag = rosbag('bagfile_001_1.bag');

ef_msgs = select(bag, 'Topic', 'iiwa_surgery/output/ef_pose');
tcp_msgs = select(bag, 'Topic', 'iiwa_surgery/output/tcp_pose');

length = min(ef_msgs.NumMessages, tcp_msgs.NumMessages);

ef_msg = readMessages(ef_msgs, 1:length, 'DataFormat', 'struct');
tcp_msg = readMessages(tcp_msgs, 1:length, 'DataFormat', 'struct');

yEF = cellfun(@(m) double(m.Pose.Position.Y), ef_msg);
zEF = cellfun(@(m) double(m.Pose.Position.Z), ef_msg);

yTCP = cellfun(@(m) double(m.Pose.Position.Y), tcp_msg);
zTCP = cellfun(@(m) double(m.Pose.Position.Z), tcp_msg);

% Fija los límites de los ejes
figure;

for i = 1:50:length
    plot(yEF(i), zEF(i), 'ko'); % Plotea el punto en el plano YZ
    hold on;
    plot(yTCP(i), zTCP(i), 'ro'); % Plotea el punto en el plano YZ
    hold on;
    plot([yEF(i) yTCP(i)], [zEF(i) zTCP(i)], 'b-'); % Plotea la línea en el plano YZ
end

% Etiquetas y título del gráfico
xlabel('Eje Y');
ylabel('Eje Z');
%title('Gráfico de Puntos 2D (Plano YZ)');

% Leyenda personalizada
legend('Posiciones EF', 'Posiciones TCP', 'Posiciones de la herramienta', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Finalizar la configuración del gráfico
hold off;

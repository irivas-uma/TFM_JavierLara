clear all;
bag = rosbag('bagfile_001_1.bag');

ef_msgs = select(bag, 'Topic', 'iiwa_surgery/output/ef_pose');
tcp_msgs = select(bag, 'Topic', 'iiwa_surgery/output/tcp_pose');

length = min(ef_msgs.NumMessages, tcp_msgs.NumMessages);

ef_msg = readMessages(ef_msgs, 1:length, 'DataFormat', 'struct');
tcp_msg = readMessages(tcp_msgs, 1:length, 'DataFormat', 'struct');

xEF = cellfun(@(m) double(m.Pose.Position.X), ef_msg);
zEF = cellfun(@(m) double(m.Pose.Position.Z), ef_msg);

xTCP = cellfun(@(m) double(m.Pose.Position.X), tcp_msg);
zTCP = cellfun(@(m) double(m.Pose.Position.Z), tcp_msg);

% Fija los límites de los ejes
figure;

for i = 1:50:length
    plot(xEF(i), zEF(i), 'ko'); % Plotea el punto en el plano XZ
    hold on;
    plot(xTCP(i), zTCP(i), 'ro'); % Plotea el punto en el plano XZ
    hold on;
    plot([xEF(i) xTCP(i)], [zEF(i) zTCP(i)], 'b-'); % Plotea la línea en el plano XZ
end

% Etiquetas y título del gráfico
xlabel('Eje X');
ylabel('Eje Z');
%title('Gráfico de Puntos 2D (Plano XZ)');

% Leyenda personalizada
legend('Posiciones EF', 'Posiciones TCP', 'Posiciones de la herramienta', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Finalizar la configuración del gráfico
hold off;

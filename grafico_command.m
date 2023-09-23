%Limpiar el espacio de trabajo 
clear;
clc;

% Cargar los datos desde el archivo BAG
bag = rosbag('bagfile_command7.bag');

% Obtener todos los mensajes de posición cartesiana (EF)
ef_msgs = select(bag, 'MessageType', 'geometry_msgs/PoseStamped');

% Parámetro para configurar la longitud de la herramienta
tool_length = 0.1275; % Ajusta la longitud según tus necesidades

% Inicializar matrices para almacenar los puntos EF y TCP
ef_points = [];
tcp_points = [];

% Recorrer los mensajes de posición cartesiana
for i = 1:ef_msgs.NumMessages
    ef_msg = readMessages(ef_msgs, i);

    % Obtener la posición y orientación del EF
    ef_position = [ef_msg{1}.Pose.Position.X, ef_msg{1}.Pose.Position.Y, ef_msg{1}.Pose.Position.Z];
    ef_orientation = [ef_msg{1}.Pose.Orientation.W, ef_msg{1}.Pose.Orientation.X, ef_msg{1}.Pose.Orientation.Y, ef_msg{1}.Pose.Orientation.Z];

    % Crear una matriz de rotación a partir del cuaternio recibido
    Rm = quat2rotm(ef_orientation);

    % Calcular el vector de dirección relativo al EF
    direction = [0, 0, tool_length];
    direction_transformed = Rm * direction';

    % Calcular la posición del TCP
    tcp_position = ef_position + direction_transformed';

    % Almacenar los puntos EF y TCP en las matrices
    ef_points = [ef_points; ef_position];
    tcp_points = [tcp_points; tcp_position];
end

% Crear una figura
figure;

% Dibujar todos los puntos EF en azul
scatter3(ef_points(:, 1), ef_points(:, 2), ef_points(:, 3), 100, 'bo', 'filled');
hold on;

% Dibujar todos los puntos TCP en azul sin rellenar
scatter3(tcp_points(:, 1), tcp_points(:, 2), tcp_points(:, 3), 100, 'bo');

% Unir cada EF con su correspondiente TCP con líneas azules
for i = 1:size(ef_points, 1)
    plot3([ef_points(i, 1), tcp_points(i, 1)], [ef_points(i, 2), tcp_points(i, 2)], [ef_points(i, 3), tcp_points(i, 3)], 'b-');
end

% Etiquetas y título del gráfico
xlabel('Eje X');
ylabel('Eje Y');
zlabel('Eje Z');
title('Gráfico de Puntos 3D EF y TCP');

% Leyenda personalizada
legend('EF', 'TCP', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Mantener la relación de aspecto
axis equal;

% Finalizar la configuración del gráfico
hold off;
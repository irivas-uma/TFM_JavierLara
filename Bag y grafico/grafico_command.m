% Cargar los datos desde el archivo BAG
bag = rosbag('bagfile_command.bag');

% Obtener todos los mensajes de posición cartesiana (EF)
ef_msgs = select(bag, 'MessageType', 'geometry_msgs/PoseStamped');

% Parámetro para configurar la longitud de la herramienta
tool_length = 0.1; % Ajusta la longitud según tus necesidades

% Inicializar matrices para almacenar los puntos EF y TCP
ef_points = [];
tcp_points = [];

% Recorrer los mensajes de posición cartesiana
for i = 1:length(ef_msgs.MessageList)
    ef_msg = ef_msgs.MessageList{i};
    
    % Obtener la posición y orientación del EF
    ef_position = [ef_msg.Pose.Position.X, ef_msg.Pose.Position.Y, ef_msg.Pose.Position.Z];
    ef_orientation = [ef_msg.Pose.Orientation.W, ef_msg.Pose.Orientation.X, ef_msg.Pose.Orientation.Y, ef_msg.Pose.Orientation.Z];
    
    % Reordenar el cuaternio en el vector q (w, x, y, z)
    q = ef_orientation;
    
    % Crear una matriz de rotación a partir del cuaternio recibido 
    Rm = quat2rotm(q);

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

% Dibujar todos los EF y TCP
scatter3(ef_points(:, 1), ef_points(:, 2), ef_points(:, 3), 100, 'bo', 'filled'); % EF en azul relleno
hold on;
scatter3(tcp_points(:, 1), tcp_points(:, 2), tcp_points(:, 3), 100, 'bo'); % TCP en azul sin relleno

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
legend('EF (Herramienta)', 'TCP (Herramienta)', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Mantener la relación de aspecto
axis equal;

% Finalizar la configuración del gráfico
hold off;

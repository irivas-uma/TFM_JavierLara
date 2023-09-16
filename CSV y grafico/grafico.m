% Cargar los datos desde el archivo CSV
data = csvread('datos_robot.csv');

% Dividir los datos en coordenadas x, y, y z
x = data(:, 1);
y = data(:, 2);
z = data(:, 3);

% Crear una figura
figure;

% Dibujar el primer punto (Punto de fulcro) en rojo y relleno
scatter3(x(1), y(1), z(1), 100, 'ro', 'filled');
hold on;

% Dibujar el segundo punto (EF inicial) en verde y relleno
scatter3(x(2), y(2), z(2), 100, 'go', 'filled');
hold on;

% Dibujar el tercer punto (TCP inicial) en verde y sin relleno
scatter3(x(3), y(3), z(3), 100, 'go');
hold on;

% Unir el EF inicial y el TCP inicial con una línea verde
plot3(x(2:3), y(2:3), z(2:3), 'g-');
hold on;

% Inicializar colores y símbolos para los EF y TCP tras incrementos
colors = ['b'; 'b']; % Azul para EF tras incremento, sin relleno
symbols = ['o'; ''];  % 'o' para EF tras incremento, vacío para TCP tras incremento

% Recorrer los datos a partir del cuarto punto
for i = 4:2:length(x)
    % Dibujar el punto actual (EF tras incremento) en azul y relleno
    scatter3(x(i), y(i), z(i), 100, 'bo', 'filled');
    hold on;
    
    % Dibujar el punto siguiente (TCP tras incremento) en azul y sin relleno
    scatter3(x(i+1), y(i+1), z(i+1), 100, 'bo');
    hold on;
    
    % Unir el par de puntos actual con una línea azul
    plot3(x(i:i+1), y(i:i+1), z(i:i+1), 'b-');
end

% Etiquetas y título del gráfico
xlabel('Eje X');
ylabel('Eje Y');
zlabel('Eje Z');
title('Gráfico de Puntos 3D');

% Leyenda personalizada
legend('Punto de fulcro', 'EF inicial', 'TCP inicial', 'Primera posición de la herramienta', 'EF tras incrementos', 'TCP tras incrementos',  'Posiciones de la herramienta tras incrementos', 'Location', 'Best');

% Mostrar la cuadrícula
grid on;

% Mantener la relación de aspecto
axis equal;

% Finalizar la configuración del gráfico
hold off;


% Clean variables
clc, clear all, close all;
load('Control_Kin_Arm_4DOF.mat')

%% TRAYECTORIA DESEADA GRIPPER


%% ANIMACION
figure(3)
l1 =0.0676;
l2 =0.06883;            
l3 =0.06883;          
l4 =0.15916;


axis equal
DimensionesManipulador_i(0,l1,l2,l3,l4,1);
h1=Manipulador3D(0,0,0,q(1,1),q(2,1),q(3,1),q(4,1));
h2=plot3(0,0,0,'*b'); hold on
h3=plot3(0,0,0,'*b'); hold on
view(3)
axis equal 
pause=10;

%% ANIMACION FOR
for i=1:pause:length(t)
  drawnow;
  delete(h1);
  delete(h2);
  h1=Manipulador3D(0,0,0,q(1,i),q(2,i),q(3,i),q(4,i));hold on
  h2=plot3(h(1,1:i),h(2,1:i),h(3,1:i),'*r'); hold on  
  h3 = plot3(h_d(1,1:i),h_d(2,1:i),h_d(3,1:i),'*b'); hold on
  
%   pause(5)
end


% Crear las figuras
figure(1);

% Gráfico de posiciones en el eje X
subplot(3, 1, 1);
plot(t, h(1, 1:end-1), 'b', 'LineWidth', 1.5); hold on;
plot(t, h_d(1, :), 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Posición X');
legend('Posición leída (x)', 'Posición deseada (x_d)');
title('Posiciones del Espacio Operativo en X');
grid on;

% Gráfico de posiciones en el eje Y
subplot(3, 1, 2);
plot(t, h(2, 1:end-1), 'b', 'LineWidth', 1.5); hold on;
plot(t, h_d(2, :), 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Posición Y');
legend('Posición leída (x)', 'Posición deseada (x_d)');
title('Posiciones del Espacio Operativo en Y');
grid on;

% Gráfico de posiciones en el eje Z
subplot(3, 1, 3);
plot(t, h(3, 1:end-1), 'b', 'LineWidth', 1.5); hold on;
plot(t, h_d(3, :), 'r--', 'LineWidth', 1.5);
xlabel('Tiempo (s)');
ylabel('Posición Z');
legend('Posición leída (x)', 'Posición deseada (x_d)');
title('Posiciones del Espacio Operativo en Z');
grid on;

% Gráfico de acciones de control
figure(2);
for i = 1:4
    subplot(4, 1, i);
    plot(t, u(i, 1:end-1), 'LineWidth', 1.5);
    xlabel('Tiempo (s)');
    ylabel(['u', num2str(i)]);
    title(['Acción de Control u', num2str(i)]);
    grid on;
end


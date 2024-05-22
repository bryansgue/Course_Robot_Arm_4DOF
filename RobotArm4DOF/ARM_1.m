clc
clear all
close all

% Cerrar el nodo de ROS
rosshutdown;
rosinit;


control_actions(:,1) = [0;0;0]
%% CONFIGURACION PUBLICADOR
[state_topic, state_msg] = rospublisher('/states','geometry_msgs/Pose');

state_msg.Position.X = 2;


% Publicar el mensaje en el tópico '/states'
send(state_topic, state_msg);


%% CONFIGURACION SUSCRIPTOR
% Crear el suscriptor para el mensaje de Odometría
 control_sub = rossubscriber('/states');


%% Programa principal

f = 30; %Hz
ts = 1/f; % Tiempo de muestreo (segundos)
tiempo_simulacion = 60*5; % Tiempo total de simulación (segundos)

% Calcular el número de iteraciones necesario
num_iteraciones = tiempo_simulacion / ts;

% Bucle for para publicar múltiples mensajes
for i = 1:num_iteraciones
    tic
    
    x = round(randn(1, 4), 2);
    x_p = round(randn(1, 4), 2);

    state_msg.Position.X = x(1);
    state_msg.Position.Y = x(2);
    state_msg.Position.Z = x(3);
    
    
    send(state_topic, state_msg);
 
    
    % Leer el mensaje de Odometría
    try
        msg_control = receive(control_sub, 0.1); % Espera hasta 0.1 segundos por un mensaje
        control_actions(:,i+1) = [msg_control.Position.X; msg_control.Position.Y; msg_control.Position.Z];  
    catch
        control_actions(:,i+1) = control_actions(:,i);
    end
    
    
    disp(control_actions(:,i)')

    while toc<ts
    end
    
   
end

% Cerrar el nodo de ROS
rosshutdown;

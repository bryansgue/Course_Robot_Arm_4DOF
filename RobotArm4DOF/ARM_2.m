clc
clear all
close all

% Cerrar el nodo de ROS
rosshutdown;
rosinit;


control_actions(:,1) = [0;0;0]
%% CONFIGURACION PUBLICADOR
[state_topic, state_msg] = rospublisher('/states','nav_msgs/Odometry');
frame_id = 'map';
state_msg.Header.FrameId = frame_id;

state_msg.Pose.Pose.Position.X = 2;


% Publicar el mensaje en el tópico '/states'
send(state_topic, state_msg);


%% CONFIGURACION SUSCRIPTOR
% Crear el suscriptor para el mensaje de Odometría
 control_sub = rossubscriber('/states');



%% Programa principal

f = 30; %Hz
ts = 1/f; % Tiempo de muestreo (segundos)
tiempo_simulacion = 60; % Tiempo total de simulación (segundos)

% Calcular el número de iteraciones necesario
num_iteraciones = tiempo_simulacion / ts;


%% Trayectoria deseada
% Generar el vector de tiempo t
t = linspace(0, tiempo_simulacion, num_iteraciones);

hxd =  3 * sin(5*0.08*t)  + 0.15;
hyd = 3 * sin(5*0.04*t);
hzd = 3  * sin(5*0.08*t)  + 0.1;
 


% Bucle for para publicar múltiples mensajes
for i = 1:num_iteraciones
    tic
    
    x = round(randn(1, 4), 2);
    x_p = round(randn(1, 4), 2);

    frame_id = 'map';
    state_msg.Header.FrameId = frame_id;
    state_msg.Pose.Pose.Position.X = hxd(i);
    state_msg.Pose.Pose.Position.Y = hyd(i);
    state_msg.Pose.Pose.Position.Z = hzd(i);
    
    
    send(state_topic, state_msg);
 
    
    % Leer el mensaje de Odometría
    try
        msg_control = receive(control_sub, 0.1); % Espera hasta 0.1 segundos por un mensaje
        control_actions(:,i+1) = [msg_control.Pose.Pose.Position.X; msg_control.Pose.Pose.Position.Y; msg_control.Pose.Pose.Position.Z];  
    catch
        control_actions(:,i+1) = control_actions(:,i);
    end
    
    
    disp(control_actions(:,i)')

    while toc<ts
    end
    
   
end

% Cerrar el nodo de ROS
rosshutdown;


%**********************************************************************************************************
%*******************************CONTROL DE POSICION SCARA***************************************************
%**********************************************************************************************************
clc; clear all; close all; warning off;
tfin = 30;
f = 30;
ts   = 1/f;
t    = 0:ts:tfin;


% POSICIONES DESEADAS DE LOS ESLABONES PARA AHORRAR ENERGIA
q1d = 30*pi/180;               
q2d = -40*pi/180;               
q3d = -25*pi/180;               
q4d = 20*pi/180;

%% DISTANCIAS DE LOS ESLABONES

l1 =0.07;
l2 =0.071;            
l3 =0.071;          
l4 =0.15;


   
%% TRAYECTORIA DESEADA GRIPPER
value  = 9;
hxd =  0.025 * sin(value*0.08*t)  + 0.15;
hyd = 0.1 * sin(value*0.04*t);
hzd = 0.05 * sin(value*0.08*t)  + 0.125;

hd = [hxd; hyd; hzd];
% %% VELOCIDAD DESEADA
hxdp=  0.025 * value* 0.08 * cos(value*0.08*t);
hydp= 0.1 * value * 0.04 * cos(value*0.04*t);
hzdp= 0.05 * value* 0.08 * cos(value*0.08*t);


%% Lectura de posiciones del brazo for ROS

rosshutdown
setenv('ROS_MASTER_URI','http://localhost:11311')
% setenv('ROS_IP','192.168.88.104')
rosinit

[velControl_topic, velControl_msg] = rospublisher('/control','sensor_msgs/Joy');
axe = [0.0 0 0 0];
% Establecer los valores de axe en el mensaje
velControl_msg.Axes = axe;
% Publicar el mensaje en el tópico '/joy'
send(velControl_topic, velControl_msg);


 states_sub = rossubscriber('/states');
%[pos_arm,ver_arm,current_arm] = dynamixeldata(armSub);

    % Leer el mensaje de Odometría
    try
        msg_states = receive(states_sub, 0.1); % Espera hasta 0.1 segundos por un mensaje
        q(:,1) = [msg_states.Axes(1); msg_states.Axes(2); msg_states.Axes(3); msg_states.Axes(4)];  
    catch
        q(:,1) = [0;0;0;0];
    end
    

    
%% CINEMATICA DIRECTA 
h = CDArm4DOF(l1,l2,l3,l4,q(:,1));
hx=h(1);
hy=h(2);
hz=h(3);

%% CONTROL

for k=1:length(t)
    tic
    
    %% VECTOR DE ERRORES
    he(:,k)= hd(:,k)-h(:,k);   
    
    %% VECTOR DE VELOCIDADES DESEADAS
    hdp(:,k)=[hxdp(k) hydp(k) hzdp(k)]';
    
    %% Controlador Jacobiano
    qpref = Controler(l2,l3,l4,q(:,k),he(:,k),hdp(:,k));
       
    % Leer el mensaje de Odometría
    try
        msg_states = receive(states_sub, 0.1); % Espera hasta 0.1 segundos por un mensaje
        q(:,k+1) = [msg_states.Axes(1); msg_states.Axes(2); msg_states.Axes(3); msg_states.Axes(4)];  
    catch
        q(:,k+1) = [0;0;0;0];
    end
    
    
    %% CINEMATICA DIRECTA
    
    
    h(:,k+1) = CDArm4DOF(l1,l2,l3,l4,q(:,k+1));
    
    qpref = Controler_min_norm(l2,l3,l4,q(:,k),he(:,k),hdp(:,k));
    
    axe = [qpref(1) qpref(2) qpref(3) qpref(4)];
    % Establecer los valores de axe en el mensaje
    velControl_msg.Axes = axe;
    % Publicar el mensaje en el tópico '/joy'
    send(velControl_topic, velControl_msg);
    
     while toc<ts   
     end
        
    toc
    
    %%
end


velControl_msg.Axes = [0.0 0 0 0];

send(velControl_topic, velControl_msg);
toc


%% ANIMACION
figure(1)
axis equal
 DimensionesManipulador_i(0,l1,l2,l3,l4,1);
h1=Manipulador3D(0,0,0,q(1,1),q(2,1),q(3,1),q(4,1));
h2=plot3(hx(1),hy(1),hz(1),'*r'); hold on
h3=plot3(hxd,hyd,hzd,'*b');
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
  
%   pause(5)
end

%% GRAFICAS DE ERROR
figure(2)

plot(t,he(1,:),'g');hold on;grid on
plot(t,he(2,:),'r');hold on
plot(t,he(3,:),'b')
legend('Ex','Ey','Ez');
xlabel('Tiempo [s]');
ylabel('Error [m]');
title('ERRORES DE CONTROL');

%% VELOCIDADES DE ESLABONES
% Gráfica de las posiciones deseadas y reales del manipulador SCARA
figure(3);

subplot(3,1,1);
plot(t, hxd, 'b', t, h(1,1:end-1), 'r--');
xlabel('Tiempo [s]');
ylabel('hx [m]');
legend('Posición deseada', 'Posición real');
title('Posición en el eje x');

subplot(3,1,2);
plot(t, hyd, 'b', t, h(2,1:end-1), 'r--');
xlabel('Tiempo [s]');
ylabel('hy [m]');
legend('Posición deseada', 'Posición real');
title('Posición en el eje y');

subplot(3,1,3);
plot(t, hzd, 'b', t, h(3,1:end-1), 'r--');
xlabel('Tiempo [s]');
ylabel('hz [m]');
legend('Posición deseada', 'Posición real');
title('Posición en el eje z');
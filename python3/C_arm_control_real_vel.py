
import rospy
from sensor_msgs.msg import Joy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TwistStamped
import numpy as np
from scipy.io import savemat
import os


# Global variables Odometry Drone

q1 = 1.0
q2 = 0
q3 = 0.0
q4 = 0.0
q1_p = 0
q2_p = 0
q3_p = 0.0
q4_p = 0.0


xd_p = 0
yd_p = 0
zd_p = 0.0

#Caracteristicas del Brazo
l = [0.0676, 0.06883, 0.06883, 0.15916]


def CDArm4DOF(l, q):
    q1, q2, q3, q4 = q

    hx = np.cos(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hy = np.sin(q1)*(l[2]*np.cos(q2 + q3) + l[1]*np.cos(q2) + l[3]*np.cos(q2 + q3 + q4))
    hz = l[0] + l[2]*np.sin(q2 + q3) + l[1]*np.sin(q2) + l[3]*np.sin(q2 + q3 + q4)

    cinematica4DOF = np.array([hx, hy, hz])
    return cinematica4DOF

def jacobiana_Brazo4DOF(L, q):
    l1, l2, l3, l4 = L
    q1, q2, q3, q4 = q

    J = np.array([
        [-np.sin(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), 
         -np.cos(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), 
         -np.cos(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), 
         -l4*np.sin(q2 + q3 + q4)*np.cos(q1)],
        
        [np.cos(q1)*(l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4)), 
         -np.sin(q1)*(l3*np.sin(q2 + q3) + l2*np.sin(q2) + l4*np.sin(q2 + q3 + q4)), 
         -np.sin(q1)*(l3*np.sin(q2 + q3) + l4*np.sin(q2 + q3 + q4)), 
         -l4*np.sin(q2 + q3 + q4)*np.sin(q1)],
        
        [0, 
         l3*np.cos(q2 + q3) + l2*np.cos(q2) + l4*np.cos(q2 + q3 + q4), 
         l3*np.cos(q2 + q3) + l4*np.cos(q2 + q3 + q4), 
         l4*np.cos(q2 + q3 + q4)]
    ])
    
    return J

import numpy as np

def Controler_vel(L, q, hdp, val):

    q1, q2, q3, q4 = q

    # Jacobiano
    J = jacobiana_Brazo4DOF(L, q)

    # Matriz de ganancia
    K = val*np.eye(3)

    # Posiciones deseadas de los eslabones
    q1d = 0 * np.pi / 180
    q2d = 30 * np.pi / 180
    q3d = +15 * np.pi / 180
    q4d = 0* np.pi / 180

    # Vector n
    hq1 = q1d - q1
    hq2 = q2d - q2
    hq3 = q3d - q3
    hq4 = q4d - q4
    n = np.array([hq1, hq2, hq3, hq4])

    # Matriz ganancia D
    D = np.diag([1,5,5,5])

    # Tarea secundaria para el robot
    I = np.eye(4)
    

    TAREA_S = np.dot((I - np.dot(np.linalg.pinv(J), J)), np.dot(D, n))

    # Matriz de ganancia
    K = val*np.eye(3)

    # Controlador
    Vref = np.linalg.pinv(J) @ (hdp) 
    
    return Vref


def states_call_back(state_msg):
    global q1, q2, q3, q4, q1_p, q2_p, q3_p, q4_p
    # Leer velocidades lineales deseadas del mensaje
    q1 = state_msg.position[0]
    q2 = state_msg.position[1]
    q3 = state_msg.position[2]
    q4 = state_msg.position[3]
    q1_p = state_msg.velocity[0]
    q2_p = state_msg.velocity[1]
    q3_p = state_msg.velocity[2]
    q4_p = state_msg.velocity[3]

def rc_call_back(state_msg):
    global xd_p, yd_p, zd_p
    # Leer velocidades lineales deseadas del mensaje
    xd_p = state_msg.axes[4]/20
    yd_p = state_msg.axes[3]/20
    zd_p = state_msg.axes[1]/20


def get_vel_deseado():
    global xd_p, yd_p, zd_p
    x = [xd_p, yd_p, zd_p]
    return x


def get_pose_arm():
    global q1, q2, q3, q4
    x = [q1, q2, q3, q4]
    return x

def get_vel_arm():
    global q1_p, q2_p, q3_p, q4_p
    x_p = [q1_p, q2_p, q3_p, q4_p]
    return x_p


def send_velocity_control(u):


    # PUBLISHER
    control_msg = Joy()
    control_pub = rospy.Publisher("/control", Joy, queue_size=10)

    control_msg.header.frame_id = "base_link"
    control_msg.header.stamp = rospy.Time.now()

    control_msg.axes = u

    # Publish control values
    control_pub.publish(control_msg)

def main():
    # Initial Values System
    # Simulation Time
    t_final = 90
    # Sample time
    frec= 30
    t_s = 1/frec
       
    # Time simulation
    t = np.arange(0, t_final, t_s)

    # Vector Initial conditions
    x = np.zeros((4, t.shape[0]), dtype = np.double)
    x_p = np.zeros((4, t.shape[0]), dtype = np.double)
    h = np.zeros((3, t.shape[0]+1), dtype = np.double)
    u = np.zeros((4, t.shape[0]+1), dtype = np.double)

    # Read Real data
    x[:, 0] = get_pose_arm()
    x_p[:, 0] = get_vel_arm()

    h[:,0] = CDArm4DOF(l, x[:, 0])

    # Simulation System
    ros_rate = frec  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    #TAREA DESEADA
    value = 9/3

    # Definir las funciones originales como expresiones lambda
    xd = lambda t: 0.025 * np.sin(value * 0.08 * t) + 0.15
    yd = lambda t: 0.1 * np.sin(value * 0.04 * t)
    zd = lambda t: 0.05 * np.sin(value * 0.08 * t) + 0.125

    # Definir las derivadas de las funciones originales como expresiones lambda
    xdp = lambda t: 0.025 * value * 0.08 * np.cos(value * 0.08 * t)
    ydp = lambda t: 0.1 * value * 0.04 * np.cos(value * 0.04 * t)
    zdp = lambda t: 0.05 * value * 0.08 * np.cos(value * 0.08 * t)
    
    hxd = xd(t)
    hyd = yd(t)
    hzd = zd(t)
    hxdp = xdp(t)
    hydp = ydp(t)
    hzdp = zdp(t)

    # Reference Signal of the system
    ref = np.zeros((3, t.shape[0]), dtype = np.double)
    ref_p = np.zeros((3, t.shape[0]), dtype = np.double)
    ref[0,:] = hxd 
    ref[1,:] = hyd
    ref[2,:] = hzd  
    ref_p[0,:] = hxdp
    ref_p[1,:] = hydp
    ref_p[2,:] = hzdp 
 
    # Errors of the system
    Error = np.zeros((3, t.shape[0]), dtype = np.double)

    # Ganancia del controlador
    K = 0.5

    a = True
    umbral = 0.01
    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):


        # Read Real data
        x[:, k] = get_pose_arm()
        #x_p[:, k] = get_vel_arm()

        #h[:,k] = CDArm4DOF(l, x[:, k])

        #Controlador
        #Error[:,k] = ref[:, k] - h[:, k]

        angulo = x[0,k] + np.pi
        R = np.array([
            [np.cos(angulo), np.sin(angulo), 0],
            [-np.sin(angulo), np.cos(angulo), 0],
            [0, 0, 1]
            ])

        # Obtener la velocidad deseada en el marco inercial
        vel_d = get_vel_deseado()

        # Transformar la velocidad deseada al marco de referencia de la base usando la transpuesta de R
        ref_p[:, k] = np.linalg.inv(R)  @ vel_d
        
        u[:,k] = Controler_vel(l, x[:, k], ref_p[:, k], K)
    
        send_velocity_control(u[:,k])
        # Loop_rate.sleep()
        rate.sleep() 

        print(vel_d)
    
    send_velocity_control([0, 0, 0, 0])


    # Ruta que deseas verificar
    pwd = "/home/bryansgue/Doctoral_Research/Cursos/Course_Robot_Arm_4DOF/Matlab"

    # Verificar si la ruta no existe
    if not os.path.exists(pwd) or not os.path.isdir(pwd):
        print(f"La ruta {pwd} no existe. Estableciendo la ruta local como pwd.")
        pwd = os.getcwd()  # Establece la ruta local como pwd
  
    name_file = "Control_Kin_Arm_4DOF.mat"
    
    save = True
    if save==True:
        savemat(os.path.join(pwd, name_file), {
                'h': h,
                'h_d': ref,
                't': t,
                'u': u,
                'x_e': Error,
                'q': x})

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Controlador",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/dynamixel_workbench/joint_states", JointState, states_call_back)


        rc_subscriber = rospy.Subscriber("/joy", Joy, rc_call_back)
        
                  
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0])
        pass
    else:
        print("Complete Execution")
        pass

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import numpy as np

# Global variables Odometry Drone

q1 = 1.0
q2 = 0
q3 = 0.0
q4 = 0.0
q1_p = 0
q2_p = 0
q3_p = 0.0
q4_p = 0.0

#Caracteristicas del Brazo
l = [0.07, 0.071, 0.071, 0.15]


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

def Controler(L, q, he, hdp):

    q1, q2, q3, q4 = q

    # Jacobiano
    J = jacobiana_Brazo4DOF(L, q)

    # Matriz de ganancia
    K = np.eye(3)

    # Posiciones deseadas de los eslabones
    q1d = 0 * np.pi / 180
    q2d = 60 * np.pi / 180
    q3d = -15 * np.pi / 180
    q4d = -15 * np.pi / 180

    # Vector n
    hq1 = q1d - q1
    hq2 = q2d - q2
    hq3 = q3d - q3
    hq4 = q4d - q4
    n = np.array([hq1, hq2, hq3, hq4])

    # Matriz ganancia D
    D = 0.1 * np.eye(4)

    # Tarea secundaria para el robot
    I = np.eye(4)
    

    TAREA_S = np.dot((I - np.dot(np.linalg.pinv(J), J)), np.dot(D, n))

    # Controlador
    Vref = np.linalg.pinv(J) @ (hdp + K @ np.tanh(0.5 * he)) + TAREA_S
    
    return Vref


def states_call_back(state_msg):
    global q1, q2, q3, q4, q1_p, q2_p, q3_p, q4_p
    # Leer velocidades lineales deseadas del mensaje
    q1 = state_msg.axes[0]
    q2 = state_msg.axes[1]
    q3 = state_msg.axes[2]
    q4 = state_msg.axes[3]
    q1_p = state_msg.axes[4]
    q2_p = state_msg.axes[5]
    q3_p = state_msg.axes[6]
    q4_p = state_msg.axes[7]

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
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate


    #TAREA DESEADA
    value = 9

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

    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):


        Error[:,k] = ref[:, k] - h[:, k]

        # Read Real data
        x[:, k] = get_pose_arm()
        x_p[:, k] = get_vel_arm()

        h[:,k+1] = CDArm4DOF(l, x[:, k])

        #Controlador
        u[:,k] = Controler(l, x[:, k], Error[:,k], ref_p[:, k])
    
        send_velocity_control(u[:,k])
        # Loop_rate.sleep()
        rate.sleep() 

        print(Error[:,k])
    
    send_velocity_control([0, 0, 0, 0])

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Controlador",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/states", Joy, states_call_back)
        
                  
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0])
        pass
    else:
        print("Complete Execution")
        pass
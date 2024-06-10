
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import numpy as np
import matplotlib.pyplot as plt

# Global variables Odometry Drone

u1 = 0
u2 = 0
u3 = 0.0
u4 = 0.0


def control_call_back(state_msg):
    global u1,u2,u3,u4
    # Leer velocidades lineales deseadas del mensaje
    u1 = state_msg.axes[0]
    u2 = state_msg.axes[1]
    u3 = state_msg.axes[2]
    u4 = state_msg.axes[3]

def get_control_action():
    global u1,u2,u3,u4
    u = [u1, u2, u3, u4]
    return u

def send_arm_states(q, control_pub,  control_msg ):

    control_msg.header.frame_id = "Arm States"
    control_msg.header.stamp = rospy.Time.now()

    
    control_msg.axes = q

    # Publish control values
    control_pub.publish(control_msg)



# Definir la función de dinámica del sistema
def f_sys(x, u):
    # Implementa la función que describe la dinámica del sistema
    A = np.array([[-0.7706, -0.0051, 0.2265, 0.1806],
                  [0.0163, -0.3537, -0.2810, 0.0157],
                  [-0.2056, -0.0701, -1.2446, 0.0927],
                  [-0.3169, 0.0471, -0.0381, -1.6991]])

    B = np.array([[0.9636, -0.0825, -0.1494, -0.0196],
                  [0.0113, 1.0200, -0.0269, -0.0893],
                  [0.0338, -0.0108, 1.8970, -0.0488],
                  [0.1425, 0.0266, -0.1410, 2.0500]])
    
    x_p = A @ x + B @ u
    return x_p  # Por ejemplo, para un sistema lineal

def runge_kutta_4(f_sys, q, u, dt):

    k1 = f_sys(q, u)
    k2 = f_sys(q + 0.5 * dt * k1, u)
    k3 = f_sys(q + 0.5 * dt * k2, u)
    k4 = f_sys(q + dt * k3, u)

    q_next = q + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
    
    return q_next


def plot_states(q):
    # Plot states
    plt.clf()  # Limpiar la figura anterior
    plt.plot(q[0], label='q1')
    plt.plot(q[1], label='q2')
    plt.plot(q[2], label='q3')
    plt.plot(q[3], label='q4')
    plt.xlabel('Time steps')
    plt.ylabel('State value')
    plt.title('System States over Time')
    plt.legend()
    plt.draw()
    plt.pause(0.001)  # Espera un breve período para permitir la actualización de la ventana


def main(control_pub, control_msg):

    # Simulation Time
    t_final = 60
    # Sample time
    frec= 30
    t_s = 1/frec
       
    # Time simulation
    t = np.arange(0, t_final, t_s)

    # Vector Initial conditions
    q = np.zeros((4, t.shape[0]+1), dtype = np.double)
    q_p = np.zeros((4, t.shape[0]), dtype = np.double)
    u = np.zeros((4, t.shape[0]), dtype = np.double)

    # Read Real data
    u[:, 0] = get_control_action()

    # Initial Condition
    q[:, 0] = [0.4, 0.3, 0.2 , 0.1]
    
    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate


    # Crear la figura para la gráfica
    plt.figure()

    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):

        # Read Real data
        u[:, k] = get_control_action()
        #u[:, k] = calcular_control(q[:, k], q_d)
        #u[:, k] = [1, 0, 0.0, 0.0]

        # Model ARM [ q_p = f(q,u) ]
        q_p[:, k] = f_sys(q[:, k], u[:, k])

        # Evolucion de los estados del sistema
        q[:, k+1] = q[:, k] + q_p[:, k]*t_s 
        q[:, k+1] = runge_kutta_4(f_sys, q[:, k], u[:, k], t_s )    

        send_arm_states(q[:, k+1] ,control_pub, control_msg )

        # Plot states in real time
        plot_states(q[:, :k+1])
        
        # Loop_rate.sleep()
        rate.sleep() 


        #print(q[:, k+1])
        print(f"q: {q[:, k+1]}")


if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Robot",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/control", Joy, control_call_back)

        # PUBLISHER
        control_msg = Joy()
        control_pub = rospy.Publisher("/states", Joy, queue_size=10)
        
                  
        main(control_pub, control_msg)

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        pass
    
    else:
        print("Complete Execution")
        pass

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import numpy as np
import math
import matplotlib.pyplot as plt

# Global variables Odometry Drone

q1 = 0
q2 = 0
q3 = 0.0
q4 = 0.0


q1_d = 0
q2_d = 0
q3_d = 0.0
q4_d = 0.0


def states_call_back(state_msg):
    global q1, q2, q3, q4
    q1 = state_msg.axes[0]
    q2 = state_msg.axes[1]
    q3 = state_msg.axes[2]
    q4 = state_msg.axes[3]


def rc_call_back(state_msg):
    global q1_d, q2_d, q3_d, q4_d
    q1_d = state_msg.axes[0]
    q2_d = state_msg.axes[1]
    q3_d = state_msg.axes[2]
    q4_d = state_msg.axes[3]



def get_pose_arm():
    global q1, q2, q3, q4
    x = [q1, q2, q3, q4]
    return x


def send_velocity_control(u, control_pub, control_msg):

   

    control_msg.header.frame_id = "base_link"
    control_msg.header.stamp = rospy.Time.now()

    control_msg.axes = u

    # Publish control values
    control_pub.publish(control_msg)




# Función para calcular la señal de control
def calcular_control(x, x_d):
    # Ganancia proporcional
    K_p = 10  # Ajusta este valor según sea necesario
    return K_p * (x_d - x)

def plot_states(q, qd=None):
    plt.clf()
    
    # Solo tomar las últimas 100 muestras
    q = q[:, -100:]
    if qd is not None:
        qd = qd[:, -100:]
    
    plt.plot(q[0], label='q1')
    plt.plot(q[1], label='q2')
    plt.plot(q[2], label='q3')
    plt.plot(q[3], label='q4')
    
    if qd is not None:
        plt.plot(qd[0], '--', label='qd1')
        plt.plot(qd[1], '--', label='qd2')
        plt.plot(qd[2], '--', label='qd3')
        plt.plot(qd[3], '--', label='qd4')
    
    plt.xlabel('Time steps')
    plt.ylabel('State value')
    plt.title('System States over Time')
    
    plt.legend()
    plt.draw()
    plt.pause(0.001)

def main(control_pub, control_msg ):
    # Initial Values System
    # Simulation Time
    t_final = 60
    # Sample time
    frec= 30
    t_s = 1/frec
       
    # Time simulation
    t = np.arange(0, t_final, t_s)

    # Vector Initial conditions
    q = np.zeros((4, t.shape[0]), dtype = np.double)
    q_d = np.zeros((4, t.shape[0]), dtype = np.double)
    u = np.zeros((4, t.shape[0]), dtype = np.double)

    # Read Real data
    q[:, 0] = get_pose_arm()

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    # Estado deseado
    q1_d = 1
    q2_d = 1.5
    q3_d = -2.5
    q4_d = math.radians(15)
    
    
    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):

        # Tarea deseada
        q_d[:,k] = 1*np.array([q1_d, q2_d, q3_d , q4_d])

        # Read Data
        q[:, k] = get_pose_arm()

        # Controller u = f(q) 
        #u[:, k] =  [1,-1,1,-2]
        u[:, k] = calcular_control(q[:, k], q_d[:, k])
        
        #Envia las velocidades por ROS
        send_velocity_control(u[:, k], control_pub, control_msg )


        # Plot states in real time
        plot_states(q[:, :k+1], q_d[:, :k+1])

        # Loop_rate.sleep()
        rate.sleep() 

        print(f"u:{u[:, k]}")


if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Controlador",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/states", Joy, states_call_back)

         # SUCRIBER
        #rc_subscriber = rospy.Subscriber("/joy", Joy, rc_call_back)

        
        
         # PUBLISHER
        control_msg = Joy()
        control_pub = rospy.Publisher("/control", Joy, queue_size=10)
                  
        main(control_pub, control_msg )

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("\nError System")
        send_velocity_control([0, 0, 0, 0], control_pub, control_msg )
        pass
    else:
        print("Complete Execution")
        send_velocity_control([0, 0, 0, 0], control_pub, control_msg )
        pass
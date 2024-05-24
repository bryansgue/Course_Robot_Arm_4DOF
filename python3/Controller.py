
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import TwistStamped
import numpy as np

# Global variables Odometry Drone

q1 = 0
q2 = 0
q3 = 0.0
q4 = 0.0



def states_call_back(state_msg):
    global q1, q2, q3, q4
    q1 = state_msg.axes[0]
    q2 = state_msg.axes[1]
    q3 = state_msg.axes[2]
    q4 = state_msg.axes[3]

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
    u = np.zeros((4, t.shape[0]), dtype = np.double)

    # Read Real data
    q[:, 0] = get_pose_arm()

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate

    # Estado deseado
    q_d = 1*np.array([0.5, 1, 1.5, 2])

    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):

        # Read Data
        q[:, k] = get_pose_arm()

        # Controller u = f(q) 
        #u[:, k] =  [1,0,0,0]
        u[:, k] = calcular_control(q[:, k], q_d)
        
        #Envia las velocidades por ROS
        send_velocity_control(u[:, k], control_pub, control_msg )

        # Loop_rate.sleep()
        rate.sleep() 

        print(f"u:{u[:, k]}")


if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("Controlador",disable_signals=True, anonymous=True)

        # SUCRIBER
        velocity_subscriber = rospy.Subscriber("/states", Joy, states_call_back)
        
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
        pass
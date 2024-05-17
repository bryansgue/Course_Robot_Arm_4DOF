
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
    t_final = 60
    # Sample time
    frec= 30
    t_s = 1/frec
       
    # Time simulation
    t = np.arange(0, t_final, t_s)

    # Vector Initial conditions
    x = np.zeros((4, t.shape[0]), dtype = np.double)
    x_p = np.zeros((4, t.shape[0]), dtype = np.double)

    # Read Real data
    x[:, 0] = get_pose_arm()
    x_p[:, 0] = get_vel_arm()

    # Simulation System
    ros_rate = 30  # Tasa de ROS en Hz
    rate = rospy.Rate(ros_rate)  # Crear un objeto de la clase rospy.Rate


    #INICIALIZA LECTURA DE ODOMETRIA
    for k in range(0, t.shape[0]):

        # Read Real data
        x[:, k] = get_pose_arm()
        x_p[:, k] = get_pose_arm()

        send_velocity_control([k, k, k, k])
        # Loop_rate.sleep()
        rate.sleep() 

        print(x[:, k], x_p[:, k] )


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
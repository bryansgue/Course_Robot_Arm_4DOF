#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def joy_publisher():
    # Inicializa el nodo con el nombre 'joy_publisher'
    rospy.init_node('joy_publisher', anonymous=True)
    
    # Crea un publicador en el tópico '/joy' que usa el tipo de mensaje Joy
    pub = rospy.Publisher('/joy', Joy, queue_size=10)
    
    # Define la tasa de publicación en Hz
    rate = rospy.Rate(10)  # 10 Hz
    
    while not rospy.is_shutdown():
        # Crea un mensaje Joy
        joy_msg = Joy()
        
        # Llena los campos del mensaje con 4 elementos arbitrarios
        joy_msg.axes = [-0.0, 0, 0, 0]
        joy_msg.buttons = [0, 0, 0, 0]
        
        # Publica el mensaje
        pub.publish(joy_msg)
        
        # Duerme para mantener la tasa de publicación
        rate.sleep()

if __name__ == '__main__':
    try:
        joy_publisher()
    except rospy.ROSInterruptException:
        joy_msg = Joy()
        pub = rospy.Publisher('/joy', Joy, queue_size=10)
        joy_msg.axes = [0.0, 0, 0, 0]
        pub.publish(joy_msg)
        pass

#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy

def joy_publisher():
    # Inicializa el nodo con el nombre 'joy_publisher'
    rospy.init_node('joy_publisher', anonymous=True)
    
    # Crea un publicador en el tópico '/joy' que usa el tipo de mensaje Joy
    pub = rospy.Publisher('/control', Joy, queue_size=10)
    
    # Define la tasa de publicación en Hz
    rate = rospy.Rate(10)  # 10 Hz
    
    def shutdown_hook():
        # Cuando se interrumpe con Ctrl+C, publica valores de ejes "0000"
        joy_msg = Joy()
        joy_msg.axes = [0.01, 0.00, 0.00, 0.00]
        pub.publish(joy_msg)
        rospy.loginfo("Valores de ejes cuando se interrumpe con Ctrl+C: %s", joy_msg.axes)
    
    rospy.on_shutdown(shutdown_hook)
    
    try:
        while not rospy.is_shutdown():
            # Crea un mensaje Joy
            joy_msg = Joy()
            
            # Llena los campos del mensaje con 4 elementos arbitrarios
            joy_msg.axes = [0.1, 0.00, 0.00, 0.00]
            joy_msg.buttons = [0, 0, 0, 0]
            
            # Publica el mensaje
            pub.publish(joy_msg)
            
            # Duerme para mantener la tasa de publicación
            rate.sleep()
    
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    joy_publisher()

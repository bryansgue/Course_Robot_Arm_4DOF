#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from pynput import keyboard

# Initialize the Joy message
joy_msg = Joy()
joy_msg.axes = [0.0, 0.0, 0.0, 0.0, 0]  
joy_msg.buttons = [0] * 12

# Define the key press event handler
def on_press(key):
    try:
        if key == keyboard.Key.up:
            joy_msg.axes[4] = 1.0
            print("Flecha arriba presionada")
        elif key == keyboard.Key.down:
            joy_msg.axes[4] = -1.0
            print("Flecha abajo presionada")
        elif key == keyboard.Key.left:
            joy_msg.axes[3] = 1.0
            print("Flecha izquierda presionada")
        elif key == keyboard.Key.right:
            joy_msg.axes[3] = -1.0
            print("Flecha derecha presionada")
        elif key.char == '8':
            joy_msg.axes[1] = 1.0
            print("Tecla 'up' presionada")
        elif key.char == '2':
            joy_msg.axes[1] = -1.0
            print("Tecla 'down' presionada")
    except AttributeError:
        pass

# Define the key release event handler
def on_release(key):
    try:
        if key in [keyboard.Key.up, keyboard.Key.down]:
            joy_msg.axes[4] = 0.0
            print("Flecha arriba/abajo liberada")
        elif key in [keyboard.Key.left, keyboard.Key.right]:
            joy_msg.axes[3] = 0.0
            print("Flecha izquierda/derecha liberada")
        elif key.char == '8' or key.char == '2':
            joy_msg.axes[1] = 0.0
            print("Tecla 'up'/'down' liberada")
    except AttributeError:
        pass

def main():
    rospy.init_node('joy_publisher')
    pub = rospy.Publisher('/joy', Joy, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    print("Nodo joy_publisher iniciado. Presione las flechas y las teclas 'w' y 's' para controlar.")

    listener = keyboard.Listener(on_press=on_press, on_release=on_release)
    listener.start()

    while not rospy.is_shutdown():
        pub.publish(joy_msg)
        rate.sleep()

    listener.stop()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

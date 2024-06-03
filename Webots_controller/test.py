#!/usr/bin/env python3  

from controller import Supervisor
from controller import Robot
from controller import Motor

#from controller import Device
import math
import rospy
import time

def main():
    # Obtener el nodo del robot
    robot = Supervisor()

    # Obtener una referencia a un dispositivo en tu robot 
    robot_node = robot.getFromDef("ROBOT") 
    print("FPS: {:.2f} Hz".format(1))
    print("OK")

if __name__ == '__main__':
    try:
        # Node Initialization
        rospy.init_node("vision_system",disable_signals=True, anonymous=True)

        # Simulation 
        main()

    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        pass
    else:
        print("Complete Execution")
        pass

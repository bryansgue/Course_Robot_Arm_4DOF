# Course_Robot_Arm_4DOF

## Demostración del Brazo Robot de 4 grados de libertad

![Brazo Robot de 4 grados de libertad](Gif/brazo_arm_4DOF.gif)

## Digital Twin

Demostración del Digital Twin:

![Digital Twin](Gif/DigitalTwin_2.gif)

## Creación de un Paquete ROS C++

Si deseas crear un paquete ROS C++ para trabajar con el brazo robótico, puedes seguir estos pasos:

1. Crea un nuevo paquete ROS llamado `brazo_robotico` utilizando el comando `catkin_create_pkg`:
    ```bash
    catkin_create_pkg brazo_robotico roscpp std_msgs
    ```

2. Navega al directorio de tu paquete recién creado:
    ```bash
    cd ~/catkin_ws/src/brazo_robotico
    ```

3. Crea un archivo llamado `nodo_robot.cpp` en el directorio `src` del paquete:
    ```bash
    touch src/nodo_robot.cpp
    ```

4. Regresa al directorio principal de tu espacio de trabajo de catkin y compila el paquete recién creado:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

Con estos pasos, habrás creado un paquete ROS C++ llamado `brazo_robotico` que estará listo para ser utilizado en tu proyecto de brazo robótico.

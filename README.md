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

4. Agrega el siguiente contenido al archivo `src/nodo_robot.cpp`:
    ```cpp
    #include "ros/ros.h"
    #include "std_msgs/String.h"

    int main(int argc, char **argv) {
        ros::init(argc, argv, "nodo_brazo");
        ros::NodeHandle nh;
        
        ros::Publisher pub = nh.advertise<std_msgs::String>("chatter", 1000);

        ros::Rate loop_rate(10);

        while (ros::ok()) {
            std_msgs::String msg;
            msg.data = "Hola desde brazo_robotico";

            pub.publish(msg);
            ros::spinOnce();
            loop_rate.sleep();
        }

        return 0;
    }
    ```

5. Regresa al directorio principal de tu espacio de trabajo de catkin y modifica el archivo `CMakeLists.txt` para agregar el nuevo nodo. Agrega la siguiente línea debajo de `add_executable(nodo_robot src/nodo_robot.cpp)`:
    ```cmake
    add_executable(nodo_brazo src/nodo_brazo.cpp)
    ```

6. Luego, ejecuta `catkin_make` para compilar tu paquete:
    ```bash
    cd ~/catkin_ws
    catkin_make
    ```

Con estos pasos, habrás creado un paquete ROS C++ llamado `brazo_robotico` que incluye dos nodos: `nodo_robot` y `nodo_brazo`.

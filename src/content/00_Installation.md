# Tópicos 00: Instalación y Configuración de ROS
## Contenidos:
- ROS Workspace
- ROS Package
- ROS Nodes

## Objetivos
- Realizar la instalación de ROS.
- Realizar la configuración de ROS.
- Realizar ejemplos empleando los nodos del paquete _turtlesim_.

---
## Next
- [ROS Nodes](01_Basics.md)

## ROS Installation
Los pasos se encuentran el _Readme.md_ del repositorio.

## ROS Workspace
Empezaremos creando un _catkin workspace_.
```
mkdir -p catkin_ws/src
cd catkin_ws
catkin_make
```

Ahora debemos la configuración al .bashrc en el raíz:
```
source /home/your_user/catkin_ws/devel/setup.bash
```

Al abrir una nueva terminal ejecutamos lo siguiente para ver si la configuración realizada es correcta:
```
echo $ROS_PACKAGE_PATH
```

## ROS Package
First change to the source space directory in _catkin workspace_.
```
cd ~/catkin_ws/src
```
Now, we are going to create our first ROS Package:
```
# catkin_create_pkg <package_name> [depend1] [depend2] [depend3]
catkin_create_pkg beginner_tutorials std_msgs rospy roscpp
```

## Files Subsystem
!["ROS Workspace Structure"](images/ROS_Ws_Structure.png)

---
## Setup
Antes de dar paso a esto, ejecutaremos en una terminal por comando los siguientes:
```bash
roscore # Comando para iniciar el ROS Computation Graph
rosrun turtlesim turtlesim_node # Comando para iniciar el nodo turtlesim
```
## ROS Nodes
Básicamente un nodo es un programa escrito en C++ o Python que hemos creado en nuestro ROS Package. Se recomienda que cada nodo tenga una única responsabilidad, por ejemplo, en el caso de un robot, un nodo puede ser el encargado de la navegación, otro de la visión, movimiento de motores, lectura de datos de sensores, etc.  
El nodo puede actuar como un _publisher_, _subscriber_, _service server_ o _service client_ (e inclusive una combinación de estos).  
Los nodos se comunican entre sí a través de _topics_ y _services_.  
__De forma técnica__: Son los procesos que dan forma al ROS Computation Graph. Ahora, ¿Qué es **ROS Computation Graph**?  
### ROS Computation Graph
Es una *red peer-to-peer* de procesos de ROS que procesan data juntos. Para esta sesión estará compuesto de: _nodes, Master, topics & messages._

### Commands
```bash
roscore # Comando para inciar el ROS master node
rosrun <nombre_paquete> <nombre_nodo> # Comando de ROS para ejecutar un nodo
rosnode list # Comando para listar los nodos en ROS Computation Graph
rosnode info /<node_name> # Comando para obtener la información del Nodo
```
### Ejemplo
Creamos un programa llamado *turtlesim_lemniscate.cpp*, veremos más adelante que al crear el programa tendremos la siguiente instrucción:
```C++
ros::init(argc, argv, "turtle_lemniscate");
```
De donde *turtle_lemniscate* viene a ser el nombre del nodo que se crea al correr el ejecutable creado por *turtlesim_lemniscate.cpp*.  
Podemos observar esto en nuestra terminal empleando:
```bash
rosnode list
```
Obteniendo la siguiente salida:
```
/rosout
/turtle_lemniscate
```
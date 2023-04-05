# Tópicos 01: Conceptos Básicos de ROS

## Contenidos:
- ROS Topics
- ROS Messages
- ROS Publisher
- ROS Subscriber

## Objetivos
- Realizar ejemplos empleando los nodos del paquete _turtlesim_.
- Lograr desarrollar códigos para envío y recepción de mensajes.

---
## Previous
- [ROS Installation](00_Installation.md)

## Next
- [ROS Nodes](02_Nodes.md)

---
## Setup
Debemos ejecutar los siguientes comandos en distintas terminales:
```bash
roscore # Comando para iniciar el ROS Computation Graph
rosrun turtlesim turtlesim_node # Comando para iniciar el nodo turtlesim
rosrun turtlesim turtle_teleop_key # Comando para iniciar el nodo turtle_teleop_key
```

## ROS Topics  
- Un topic es un canal de comunicación entre nodos. Un nodo puede ser el _publisher_ o un _subscriber_ de un topic.  
- Literalmente es como un tema de conversación entre dos personas y cada persona sería un nodo de ROS.

### Useful Commands

```bash
rostopic list # Comando para listar los topicos en ROS Computation Graph
rostopic info <topic_name> # Comando para obtener información de un topic
rostopic echo <topic_name> # Comando para ver la información que se está publicando en un topic
rostopic pub <topic_name> <msg_type> <msg_data> # Comando para publicar un mensaje en un topic
```
### Other Commands
```bash
rostopic bw <topic_name> # Comando para ver el ancho de banda de un topic
rostopic hz <topic_name> # Comando para ver la frecuencia de un topic
rostopic type <topic_name> # Comando para ver el tipo de mensaje de un topic
```
## ROS Messages
- Un nodo envía data a través de un topic. Esta data es un mensaje. Un mensaje es una estructura de datos que define el tipo de data que se envía a través de un topic.  
- En ROS, los mensajes son definidos en un archivo .msg.  
- En la conversación seria como el tipo de mensaje que se envía, por ejemplo, un mensaje de texto, un mensaje de voz, un mensaje de video, etc.  

En nuestro ejemplo podemos observar los topics que se están publicando en el _ROS Computation Graph_:
```bash
rostopic list
```
Prestemos atención a los siguientes:
```bash
/turtle1/cmd_vel
/turtle1/color_sensor
/turtle1/pose
```
Y ahora analicemos el tipo de mensaje que se está publicando en cada uno de ellos:
```bash
rostopic info /turtle1/cmd_vel
rostopic info /turtle1/color_sensor
rostopic info /turtle1/pose
```
Obteniendo lo siguiente:
```bash
Type: geometry_msgs/Twist
```
```bash
Type: turtlesim/Color
```
```bash
Type: turtlesim/Pose
```

## De forma gráfica
Debemos instalar *rqt_graph* para visualizar la comunicación entre nodos.  
```bash
sudo apt-get install ros-noetic-rqt
sudo apt-get install ros-noetic-rqt-common-plugins
```
Y en la terminal ejecutamos el nodo:
```bash
rosrun rqt_graph rqt_graph
```
Obteniendo lo siguiente:
![Ejemplo 01](/src/content/images/Rqt_Graph_Example_01.png)

Observando claramente que el nodo */teleop_turtle* es el _publisher_ y el nodo */turtlesim* es el _subscriber_ del topic */turtle1/cmd_vel*. 

**Hay varias cosas que no se han mencionado pero que pueden ser encontradas en la documentación oficial [aquí](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics)**


---

Nos toca aprender sobre lo que hemos venido mencionando hace ya un buen rato: los _publishers_ y los _subscribers_.
## ROS Publisher
Parte de código
## ROS Subscriber
Parte de código
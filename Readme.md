# A little about ROS
- _Autor_: Cristopher Rufasto
## Requisitos
- Tarjeta Gráfica NVidia de preferencia (Para la ejecución de Gazebo).
- Ubuntu 20.04 (Al menos hasta el 2025, después le toca a otro actualizar todo esto).
- Una buena cantidad de espacio entre _root_ y _home_. (En mi caso tengo 60Gb en cada uno).
- Conocimientos en Git, C++ y Python. (Mínimo, indispensable)
- Conocimientos en ROS. (No es tan necesario, pero si no sabes nada, te vas a perder un poco en este tutorial)
- Inglés intermedio o un buen traductor :v ([Deepl](https://www.deepl.com/es/translator))

## Contents:
00. [Installation and Configuration](src/content/00_Installation.md)
01. [ROS Basics](src/content/01_Basics.md)
02. [ROS Motion](src/content/02_Motion.md)
03. [ROS Serial](src/content/03_Serial.md)

## Instalando ROS Noetic
Debido a problemas haremos uso de dos guías:
1. [Configuración del repositorio: Pasos del 1 al 3.](https://linuxopsys.com/topics/install-ros-noetic-on-ubuntu)
2. [Instalación de todos los paquetes: Pasos del 1.4 al 1.6.1.](http://wiki.ros.org/noetic/Installation/Ubuntu)

Se recomienda encarecidamente leer bien lo que nos explican en ambas guías para no cometer errores en la instalación.  
Para verificar la instalación, ejecutamos en una terminal:
```
gazebo
```
El cual debería corrernos fácilmente a 60 fps.

## Adicional
Para programar ROS en Python deberíamos instalar el siguiente paquete:
```
sudo apt install python-is-python3
```
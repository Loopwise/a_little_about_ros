# Tópicos 03: ROS & Arduino - rosserial package
## Instalando Arduino 1.8 en Ubuntu 20.04
```bash
sudo snap install arduino
```

## Instalando rosserial
```bash
sudo apt-get install ros-${ROS_DISTRO}-rosserial-arduino
sudo apt-get install ros-${ROS_DISTRO}-rosserial
```

## Instalando ros_lib into Arduino Environment
Al instalarse por snap la librería de Arduino se encuentra en la ruta */home/user_name/snap/arduino/current/Arduino*:

```bash
cd <sketchbook>/libraries
rm -rf ros_lib
rosrun rosserial_arduino make_libraries.py .
```

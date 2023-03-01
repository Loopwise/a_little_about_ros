#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Global Variables
const double a = 2; // Tamaño del lazo
const double b = 1; // Factor de escala
const int freq = 10; // Frecuencia de Publicación (Hz)
const double dt = 1.0/freq; // Paso del tiempo (s)

inline double X(float t){
	return a * b * cos(t) / (1 + pow(sin(t), 2));
}

inline double Y(float t){
	return a * b * cos(t) * sin(t) / (1 + pow(sin(t), 2));
}

inline double d_dt(double f(float), float t, float dt){
	return (f(t + dt) - f(t) )/ dt;
}

int main(int argc, char **argv){
    // Inicialización del nodo
    ros::init(argc, argv, "turtle_lemniscate");
    ros::NodeHandle node; // Creamos el objeto nodo (necesario para crear publishers, subscribers, etc.)

    // Creamos el objeto publisher de velocidad en base a un nodo (necesario para publicar mensajes)
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // Frecuencia de publicación de mensajes (10 Hz)
    ros::Rate loop_rate(freq);

    // Variables para la lemniscata
    double t = 0; // Variable de tiempo

    while (ros::ok()){
        // Cálculo de la velocidad necesaria para seguir la lemniscata
        // Derivada de la posición con respecto al tiempo
        double vx = d_dt(X, t, dt);
        double vy = d_dt(Y, t, dt);

        // Creación del mensaje de velocidad
        geometry_msgs::Twist msg;
        msg.linear.x = vx;
        msg.linear.y = vy;
        msg.angular.z = 0;

        // Publicación del mensaje
        pub.publish(msg);

        // Actualización de la variable de tiempo
        t += dt;

        // Espera hasta el siguiente ciclo
        loop_rate.sleep();
    }

    return 0;
}
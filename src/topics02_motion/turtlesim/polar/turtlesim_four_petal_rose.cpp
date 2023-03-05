#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Global Variables
const int freq = 10; // Frecuencia de Publicación (Hz)
const double dt = 1.0/freq; // Paso del tiempo (s)
const double a = 3; // Tamaño del lazo
const double w = 2*M_PI*dt; // Frecuencia angular


// Function Definition
inline double r(float t){
    return a * sin(2*w*t);
}

inline double X(float t){
    return r(t) * cos(w*t);
}

inline double Y(float t){
    return r(t) * sin(w*t);
}

inline double d_dt(double f(float), float t, float dt){
    return (f(t + dt) - f(t - dt) )/ (2*dt);
}

int main(int argc, char **argv){
    // Inicialización del nodo
    ros::init(argc, argv, "turtle_four_petal_rose");
    ros::NodeHandle node; // Creamos el objeto nodo (necesario para crear publishers, subscribers, etc.)

    // Creamos el objeto publisher de velocidad en base a un nodo (necesario para publicar mensajes)
    ros::Publisher pub = node.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 1000);

    // Frecuencia de publicación de mensajes (10 Hz)
    ros::Rate loop_rate(freq);

    // Variables para la lemniscata
    double t = 0; // Variable de tiempo

    while (ros::ok()){
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
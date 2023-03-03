#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

// Global Variables
const int freq = 500; // Frecuencia de Publicación (Hz)
const double dt = 1.0/freq; // Paso del tiempo (s)
const double a = 3; // Tamaño del lazo
const double w = 2*M_PI/10; // Velocidad angular del w*t

// Function Definitions
inline double r(float t){
    return a * sin(2*w*t); // 2*theta = w*t
}

inline double X(float t){
    return r(t) * cos(w*t); // cos(theta) = cos(w*t)
}

inline double Y(float t){
    return r(t) * sin(w*t); // sin(theta) = sin(w*t)
}

// First numerical derivative
inline double d_dt(double f(float), float t, float dt){
	return (f(t + dt) - f(t - dt) )/ (2*dt);
}

// Second numerical derivative
inline double d2_dt2(double f(float), float t, float dt){
    return (f(t + dt) - 2*f(t) + f(t - dt) )/ pow(dt, 2);
}

// Ratio of the curvature
inline double R(double t){
    double num = pow(d_dt(X, t, dt), 2) + pow(d_dt(Y, t, dt), 2);
    double den = d_dt(X, t, dt)*d2_dt2(Y, t, dt) - d_dt(Y, t, dt)*d2_dt2(X, t, dt);

    return pow(num, 1.5)/den;
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

        double radio = R(t);

        double v = sqrt(pow(vx, 2) + pow(vy, 2));

        double v_theta = v/radio;

        // Creación del mensaje de velocidad
        geometry_msgs::Twist msg;
        msg.linear.x = v;
        msg.angular.z = v_theta;

        // Publicación del mensaje
        pub.publish(msg);

        // Actualización de la variable de tiempo
        t += dt;

        // Espera hasta el siguiente ciclo
        loop_rate.sleep();
    }

    return 0;
}
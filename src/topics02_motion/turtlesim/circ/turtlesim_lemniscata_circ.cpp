#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cmath>

/*
 * Debido al método que se emplea para calcular la velocidad angular y lineal
 * debemos aumentar la frecuencia de publicación de mensajes para que el robot
 * siga la lemniscata correctamente. Y minimizar el error de la derivada.
*/

// Global Variables
const double a = 2.0; // Tamaño del lazo
const double b = 1.0; // Factor de escala
const int freq = 100; // Frecuencia de Publicación (Hz)
const double dt = 1.0/freq; // Paso del tiempo (s)

inline double X(float t){
	return a * b * cos(t) / (1 + pow(sin(t), 2));
}

inline double Y(float t){
	return a * b * cos(t) * sin(t) / (1 + pow(sin(t), 2));
}

// Primera derivada numérica
inline double d_dt(double f(float), float t, float dt){
	return (f(t + dt) - f(t - dt) )/ (2*dt);
}

// Segunda derivada numérica
inline double d2_dt2(double f(float), float t, float dt){
    return (f(t + dt) - 2*f(t) + f(t - dt) )/ pow(dt, 2);
}

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
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

using namespace std;

const int freq = 10; // 10 Hz
const float tol = 0.1; // Tolerancia
const float P_v = 1.5; // Proportional Control Linear Velocity
const float P_w = 4; // Proportional Control Angular Velocity

void move_turtle(ros::Publisher publisher, float goal_x, float goal_y){
    ros::Rate rate(freq);
    while (ros::ok()){
        // Suscribirse a la posición actual de la tortuga
        turtlesim::PoseConstPtr pose = ros::topic::waitForMessage<turtlesim::Pose>("/turtle1/pose");

        // Calcular la distancia a la posición objetivo
        float distance = sqrt(pow(goal_x - (pose -> x), 2) + pow(goal_y - (pose -> y), 2));

        if (distance < tol)
            break;

        // Calcular el ángulo hacia el objetivo
        float angle = atan2(goal_y - (pose -> y), goal_x - (pose -> x));

        // Publicar un mensaje Twist para mover la tortuga hacia el objetivo
        geometry_msgs::Twist msg;
        msg.linear.x = P_v* distance;
        msg.angular.z = P_w * (angle - pose->theta);
        publisher.publish(msg);

        rate.sleep();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "Turtle_goal_to_goal");
    ros::NodeHandle node_handle;
    ros::Publisher vel_pub = node_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    float X = 3, Y = 3;
    move_turtle(vel_pub, X, Y); // Mover la tortuga hacia la posición (5, 5)

    return 0;
}

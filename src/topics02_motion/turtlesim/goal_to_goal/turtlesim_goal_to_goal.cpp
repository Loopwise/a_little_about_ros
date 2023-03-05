#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <cmath>

void move_turtle(ros::Publisher publisher, float goal_x, float goal_y)
{
    ros::Rate rate(10); // 10 Hz
    while (ros::ok())
    {
        // Suscribirse a la posición actual de la tortuga
        turtlesim::PoseConstPtr pose = ros::topic::waitForMessage<turtlesim::Pose>("/turtle1/pose");

        // Calcular la distancia a la posición objetivo
        float distance = std::sqrt(std::pow(goal_x - pose->x, 2) + std::pow(goal_y - pose->y, 2));

        // Si la distancia es menor que 0.1, entonces la tortuga llegó a su destino
        if (distance < 0.1){
            break;
        }

        // Calcular el ángulo hacia el objetivo
        float angle = std::atan2(goal_y - pose->y, goal_x - pose->x);

        // Publicar un mensaje Twist para mover la tortuga hacia el objetivo
        geometry_msgs::Twist msg;
        msg.linear.x = 0.5 * distance;
        msg.angular.z = 4 * (angle - pose->theta);
        publisher.publish(msg);

        rate.sleep();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_turtle");
    ros::NodeHandle node_handle;
    ros::Publisher publisher = node_handle.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10);

    move_turtle(publisher, 5, 5); // Mover la tortuga hacia la posición (5, 5)

    return 0;
}

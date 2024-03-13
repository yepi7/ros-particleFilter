#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <vector>
#include <math.h>

#define PI 3.14159265358979323846

// Estructura para almacenar la posición actual del robot
struct RobotState {
    float x, y, yaw;
} robot_state;

// Estructura para definir un punto
struct Point {
    float x, y;
};

// Vector para almacenar los puntos objetivos
std::vector<Point> points = {{2, 2}, {4, 0}, {0, 0}};

// Índice para el punto objetivo actual
size_t current_point = 0;

// Callback para la odometría
void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    robot_state.x = msg->pose.pose.position.x;
    robot_state.y = msg->pose.pose.position.y;
    float siny_cosp = 2 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z);
    float cosy_cosp = 1 - 2 * (msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
    robot_state.yaw = atan2(siny_cosp, cosy_cosp);
}

bool isGoalReached(const Point& goal, float threshold = 0.1) {
    float dx = goal.x - robot_state.x;
    float dy = goal.y - robot_state.y;
    float distance = sqrt(dx * dx + dy * dy);
    return distance < threshold;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "move_robot");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("odom", 1000, odometryCallback);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Rate rate(10.0);

    while (ros::ok() && current_point < points.size()) {
        ros::spinOnce();
        
        if (isGoalReached(points[current_point])) {
            ++current_point; // Mover al siguiente punto
            if (current_point >= points.size()) break; // Si no hay más puntos, salir del bucle
            continue;
        }

        float goal_angle = atan2(points[current_point].y - robot_state.y, points[current_point].x - robot_state.x);
        float angle_diff = goal_angle - robot_state.yaw;

        // Normalización del ángulo a [-PI, PI]
        while (angle_diff > PI) angle_diff -= 2 * PI;
        while (angle_diff < -PI) angle_diff += 2 * PI;

        geometry_msgs::Twist cmd_vel;
        if (fabs(angle_diff) > 0.1) { // Si la diferencia de ángulo es significativa
            cmd_vel.linear.x = 0.0;
            cmd_vel.angular.z = 0.2 * (angle_diff / fabs(angle_diff)); // Ajustar velocidad angular
        } else {
            cmd_vel.linear.x = 0.5; // Ajustar velocidad lineal hacia adelante
            cmd_vel.angular.z = 0.0;
        }

        vel_pub.publish(cmd_vel);

        rate.sleep();
    }

    // Detener el robot al finalizar
    geometry_msgs::Twist stop_msg;
    stop_msg.linear.x = 0.0;
    stop_msg.angular.z = 0.0;
    vel_pub.publish(stop_msg);

    return 0;
}

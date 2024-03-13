#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

#define PI 3.14159265

float point_x;
float point_y;
float point_z;
float orien_w;

void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
	std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << ", w:"<< msg->pose.pose.orientation.w << "}" << std::endl;
	point_x = msg->pose.pose.position.x;
	point_y = msg->pose.pose.position.y;
	point_z = msg->pose.pose.position.z;
	orien_w = msg->pose.pose.orientation.w;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"firstProj");	
	ros::NodeHandle nh;	
	
	//float point_x;
	
	
	// Queremos conectarnos a odometria
	ros::Subscriber odom_sub = nh.subscribe("/robot0/odom",1000,odometryCallback); 
	ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1000);

	geometry_msgs::Point points;
	points.x = 2;
	points.y =2;

	float dx, dy, angle;

	//dest_points = [2,3,4,5,6,7,8,9,10,11];

	// para el angulo se hace la arcotangente del cateto adyacente entre el cateto opuesto.
	//std::atan2(points.y,points.x); // Sirve para calcular con el cuadrante automatico el 
	// cuidado con el error del angulo

	//dist = sqrt(pow((points.y-msg->pose.pose.position.y),2) + pow((points.x-msg->pose.pose.position.x),2));

	ros::Rate loop(10); // Ejecuta a hercios
	while(ros::ok()){ // Espera a que el master este listo para comunicarse
		geometry_msgs::Twist speed;
		//geometry_msgs::Pose position;
		
		dy = points.y-point_y;
		dx = points.x-point_x;
		std::cout << "dx: "<< dx <<" orientation: " << orien_w << " angulo: " << angle << "\n";
		// dist = sqrt(pow((dy),2) + pow((dx),2));
		//speed.linear.x=0.1;
		speed.linear.x = 0.1;
		speed.angular.z = 0.1;

		angle = atan2(dy,dx); //*180/PI; // atan2 devuelve radiantes. Convertimos de radiantes a grados.

		speed_pub.publish(speed);
		ros::spinOnce();
		loop.sleep();
	}

	
	return 0;

}

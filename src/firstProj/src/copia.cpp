#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <math.h>

#include "tinyxml2.h"
#include <tf2/utils.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Importacion de libreria para calculo geometrico
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <cmath>

#define PI M_PI

static float point_x;
static float point_y;
static float point_z;
static float orien_w;

namespace bg = boost::geometry;

// Definicion de estructuras	
struct Punto {
	float x;
	float y;
};
struct Obstaculo{
	float xmin;
	float xmax;
	float ymin;
	float ymax;
};

// Declaracion de funciones
double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion);
void quaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg);
void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
void laserParticula(int x0, int y0, int laserRadius);
void perteneceObstaculo(Punto punto, Obstaculo obstaculo, int angulo, int laserRadius);
void distanciaObstaculo(Punto punto, Obstaculo obstaculo, int angulo);

// Funcion main
int main(int argc, char** argv){

	// Definicion de variables
	typedef bg::model::point<float, 2, bg::cs::cartesian> point;
    typedef bg::model::box<point> box;
    typedef bg::model::linestring<point> linestring;
	// Definicion del obstaculo
	box rectangle(point(25, 30), point(55, 50)); // Esquina inf-izq, esquina sup-der
	// Punto central del rectangulo
    point rect_center(40, 40);

	// Declaracion de parametros
	uint8_t laserRadius = 56;

	// Inicializacion
	ros::init(argc,argv,"firstProj");
	ros::NodeHandle nh;	

	// Suscripciones	
	ros::Subscriber sub = nh.subscribe("/robot0/odom", 1000, odometryCallback);
	//ros::Subscriber sub = nh.subscribe("/robot0/odom", 1000, quaternionCallback);
	ros::Publisher speed_pub = nh.advertise<geometry_msgs::Twist>("/robot0/cmd_vel",1000);
	// Importar XML File
	tinyxml2::XMLDocument doc;
	doc.LoadFile("/home/alumno/robotica_movil_ws/src/firstProj/src/obstacles.xml");

	// Creacion de las particulas. Posteriormente anadir aleatoriedad.
	Punto arrayPuntos[10][3]  = {
		{230,275},
		{238,70},
		{359,60},
		{196,43},
		{137,246},
		{78,131},
		{168,43},
		{48,190},
		{300,202},
		{124,239}
	};

	linestring sonar;
    for (int angle = 0; angle < 360; ++angle) {
        float angle_rad = angle * (M_PI / 180.0f);
        float x = rect_center.get<0>() + 56 * std::cos(angle_rad); // El radio del sonar es laserRadius, 56
        float y = rect_center.get<1>() + 56 * std::sin(angle_rad);
        sonar.push_back(point(x, y));
    }

   // Calcular las intersecciones entre el sonar y el rectángulo
    std::vector<bg::model::linestring<point>> intersections;
    bg::intersection(sonar, rectangle, intersections);
    std::out << "HOLA ESTO ES UNA PRUEBA" << std::endl;
    // Imprimir las intersecciones
    for (const auto& linestring : intersections) {
        for (const auto& point : linestring) {
            std::cout << "Intersección: " << bg::get<0>(point) << ", " << bg::get<1>(point) << std::endl;
        }
    }

	// Prueba, borrar
	// laserParticula(0,0,laserRadius);

	// Ejecucion
	ros::Rate loop(10); // Ejecuta a hercios
	while(ros::ok()){ // Espera a que el master este listo para comunicarse
		


	// 	geometry_msgs::Twist speed;
	// 	speed.linear.x = 0.1;
	// 	speed.angular.z = 0.1;

	// 	// Empieza a moverse
	// 	speed_pub.publish(speed);
	// 	ros::spinOnce();
	// 	loop.sleep();

	// }
	return 0;
}

// Definicion de funciones
double getYawFromQuaternion(const geometry_msgs::Quaternion& quaternion){
    tf2::Quaternion tf_quaternion;
    tf2::fromMsg(quaternion, tf_quaternion);

    // Convertir quaternion a un objeto de tipo tf2::Matrix3x3
    tf2::Matrix3x3 tf_matrix(tf_quaternion);

    // Obtener los ángulos de Euler
    double roll, pitch, yaw;
    tf_matrix.getRPY(roll, pitch, yaw);

    // Devolver el ángulo de apuntamiento (yaw)
    return yaw;
}
void quaternionCallback(const geometry_msgs::Quaternion::ConstPtr& msg)
{
    // Calcular el ángulo de apuntamiento (yaw) a partir del quaternion recibido
    double yaw = getYawFromQuaternion(*msg);

    // Imprimir el ángulo de apuntamiento (yaw)
    ROS_INFO("Ángulo de apuntamiento (yaw): %f", yaw);
	std::cout << "Angulo de apuntamiento (yaw):" << yaw << "\n" << std::endl;
}
void odometryCallback(const nav_msgs::OdometryConstPtr& msg){
	//std::cout << "Position: {x:" << msg->pose.pose.position.x << ", y:" << msg->pose.pose.position.y << ", w:"<< msg->pose.pose.orientation.w << "}" << std::endl;
	point_x = msg->pose.pose.position.x;
	point_y = msg->pose.pose.position.y;
	point_z = msg->pose.pose.position.z;
	orien_w = msg->pose.pose.orientation.w;

	geometry_msgs::Quaternion orientation = msg->pose.pose.orientation;
	float yaw = getYawFromQuaternion(orientation);
    ROS_INFO("Angulo de apuntamiento (yaw): %f", yaw);
}

// Calcula los puntos del laser a distancia maxima en 360 grados.
// Posiblemente anadir un array de obstaculos como argumento
void laserParticula(int x0, int y0, int laserRadius){

	// Definir el obstaculo fuera de la funcion. BORRAR.
	Obstaculo obstaculo = {25,55,30,50}; // width: 30, height: 20

	Punto puntos[360];
	float distancias[360];
	for (int i=0; i<360; i++){
		puntos[i].x = x0 + laserRadius*cos(i*M_PI/180);
		puntos[i].y = y0 + laserRadius*sin(i*M_PI/180);
		std::cout << "" << std::endl;
		std::cout << "Para el angulo: "<<i<<" grados:" << std::endl;
		std::cout << "El punto de la circunf es: ("<<puntos[i].x <<","<<puntos[i].y<<")" << std::endl;
		//std::cout << "--------------------------------------" << std::endl;

		perteneceObstaculo(puntos[i], obstaculo, i, laserRadius);

	}
}
void perteneceObstaculo(Punto punto, Obstaculo obstaculo, int angulo, int laserRadius){ // Quizás modificar y que devuelva bool.
	//std::vector<float> distRobotObstaculo; // Array de distancias al obstaculo
	float distRobotObstaculo[360];
	float dx, dy;

	if ((punto.x <= obstaculo.xmax)&&(punto.x >= obstaculo.xmin)){
		if ((punto.y <= obstaculo.ymax) && (punto.y >= obstaculo.ymin)){
			std::cout << "--------------------------------------" << std::endl;
			std::cout << "El punto "<<"("<<punto.x<<","<<punto.y<<") pertenece al obstaculo"<<std::endl;
			std::cout << "--------------------------------------" << std::endl;
			
			dx = punto.x - obstaculo.xmin;
			dy = obstaculo.ymax - punto.y;
			
			std::cout << "Los DIFERENCIALES son: dx:"<<dx<<",dy:"<<dy<<std::endl;

			if (dx > laserRadius*cos(89*M_PI/180)){ // Que el error sea suficiente
				//distRobotObstaculo.push_back(obstaculo.ymin/sqrt(1-pow(cos(angulo*M_PI/180),2)));
				distRobotObstaculo[angulo] = obstaculo.ymin/sqrt(1-pow(cos(angulo*M_PI/180),2));
				std::cout << "LA DISTANCIA AL OBSTACULO ES (para dx="<< dx << ")" <<std::endl;
				std::cout << distRobotObstaculo[angulo] << std::endl;
			}
			// if (dy > laserRadius*sin(M_PI/180)){
			// 	//distRobotObstaculo.push_back(obstaculo.xmin/sqrt(1-pow(sin(angulo*M_PI/180),2)));
			// 	distRobotObstaculo[angulo] = obstaculo.xmin/sqrt(1-pow(sin(angulo*M_PI/180),2));
			// 	std::cout << "LA DISTANCIA AL OBSTACULO ES(para dy="<< dy << ")" <<std::endl;
			// 	std::cout << distRobotObstaculo[angulo] << std::endl;
			// }
		}
		// No hace nada
	}
	// No hace nada
}
// Solo valido para primer cuadrante
void distanciaObstaculo(Punto punto, Obstaculo obstaculo, int angulo){
	std::vector<float> distRobotObstaculo; // Array de distancias al obstaculo
	if (punto.x - obstaculo.xmin > 0){
		distRobotObstaculo.push_back(sqrt(pow(obstaculo.ymin,2)/(1-pow(cos(angulo*M_PI/180),2))));
		std::cout << "LA DISTANCIA AL OBSTACULO ES:" <<std::endl;
		std::cout << distRobotObstaculo[angulo] << std::endl;
	}
	if (obstaculo.ymax - punto.y > 0){
		distRobotObstaculo.push_back(sqrt(pow(obstaculo.xmin,2)/(1-pow(sin(angulo*M_PI/180),2))));
		std::cout << "LA DISTANCIA AL OBSTACULO ES:" <<std::endl;
		std::cout << distRobotObstaculo[angulo] << std::endl;
	}
}

// bool funcion robotDentroDeObstaculo()
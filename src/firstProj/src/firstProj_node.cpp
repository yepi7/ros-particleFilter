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

namespace bg = boost::geometry;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::linestring<point> linestring;

void calculateDistance(point sonar, std::vector<box> obstaculos);

// Funcion main
int main(int argc, char** argv){
	point sonarPositions[3] = {
		point(0.0,0.0),
		point(0.5,0.0),
		point(1.0,0.0)
	};
	// Generamos tres obstaculos
    std::vector<box> obstaculos = {
        box(point(-5, -5), point(-3, 0)),
        box(point(2, 1), point(4, 3)),
        box(point(5, -2), point(7, 1))
    };
	// Lanzar la siguiente funcion con tres puntos diferentes:
	for (int i=0; i<3; i++){
		calculateDistance(sonarPositions[i], obstaculos);
		std::cout << "--------------------------" << std::endl;
	}
	
    
	return 0;
}

void calculateDistance(point sonar, std::vector<box> obstaculos){
	for (int angle = 0; angle < 360; angle += 1) {
        float rad = angle * M_PI / 180.0;
        point direccion(std::cos(rad) * 100, std::sin(rad) * 100);

        linestring rayo;
        rayo.push_back(sonar);
        rayo.push_back(direccion);

        std::vector<linestring> output;
        for (const auto& obstaculo : obstaculos) {
            bg::intersection(rayo, obstaculo, output);
        }

        if (!output.empty()) {
            for (const auto& segmento : output) {
                if (!segmento.empty()) {
                    // Calcula la distancia del sonar al primer punto de intersección encontrado
                    float distancia = bg::distance(sonar, segmento.front());
                    std::cout << "Ángulo " << angle << " grados: Obstáculo detectado a " << distancia << " unidades." << std::endl;
                    break; // Supone que solo nos interesa el primer obstáculo detectado
                }
            }
        } else {
            std::cout << "Ángulo " << angle << " grados: Sin obstáculo." << std::endl;
        }
    }
}
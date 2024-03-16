#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"

#include "tinyxml2.h"
#include <tf2/utils.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Importacion de libreria para calculo geometrico
#include <boost/geometry/geometry.hpp>
#include <boost/geometry/geometries/point.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/linestring.hpp>

#include <cmath>
#include <math.h>
#include <random>
namespace bg = boost::geometry;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::linestring<point> linestring;

// Prototipos de funciones
void calculateDistance(point sonar, std::vector<box> obstaculos, int laserRadius);
void particlesGeneration (int numberOfParticles, point arrayOfParticles[]);

// Funcion main
int main(int argc, char** argv){

    // ============================
    // Declaracion dee las variables
    // ============================

    // Definicion de parametros
    const int laserRadius = 100;
    const int numberOfParticles = 10;
    point arrayOfParticles [numberOfParticles];
    float distancesOfPartciles [numberOfParticles][360]; // Guarda las distancias de las particulas en cada iteracion
    
    // Simula el movimiento del robot
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
    
    // ============================
    // Comienzo del programa
    // ============================
    

    // Genera
    particlesGeneration(numberOfParticles, arrayOfParticles);

	// Lanzar la siguiente funcion con tres puntos diferentes:
	for (int i=0; i<3; i++){
		calculateDistance(sonarPositions[i], obstaculos, laserRadius);
		std::cout << "----------------------------------------------------" << std::endl;
        std::cout << "----------------------------------------------------" << std::endl;
	}
	
    // Posteriormente ira aqui un bucle while donde se repita el proceso constantemente.
    
	return 0;
}
// No es necesario poner obstaculos como parametro, ya que el mapa es conocido. Puede ir en el main.
void calculateDistance(point sonar, std::vector<box> obstaculos, int laserRadius){
	for (int angle = 0; angle < 360; angle += 1) {
        float rad = angle * M_PI / 180.0;
        point direccion(std::cos(rad) * laserRadius, std::sin(rad) * laserRadius);

        linestring rayo;
        rayo.push_back(sonar);
        rayo.push_back(direccion);
        // Buscamos si existen intersecciones
        std::vector<linestring> intersecciones;
        for (const auto& obstaculo : obstaculos) {
            bg::intersection(rayo, obstaculo, intersecciones);
        }
        // En caso de haber intersecciones, se calcularan las distancias a cada una.
        if (!intersecciones.empty()) {
            // Itera por todos los puntos donde hay intersecciones. Despues calculara la distancia
            for (const auto& interseccion : intersecciones) {
                if (!interseccion.empty()) {
                    // Calcula la distancia del sonar al primer punto de intersección encontrado.
                    //Las distancias se guardan en la variable "distancia"
                    float distancia = bg::distance(sonar, interseccion.front());
                    std::cout << "Ángulo " << angle << " grados: Obstáculo detectado a " << distancia << " unidades." << std::endl;
                    break; // Supone que solo nos interesa el primer obstáculo detectado
                }
            }
        } else {
            std::cout << "Ángulo " << angle << " grados: Sin obstáculo." << std::endl;
        }
    }
}

void particlesGeneration (int numberOfParticles, point arrayOfParticles[]){
    // Motor de generación de números aleatorios
    std::mt19937 motor(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));

    for (int i=0; i<numberOfParticles; i++){
    // Distribución uniforme entre 1 y 100 (inclusive)
    std::uniform_int_distribution<> width_distribution(1, 380);
    std::uniform_int_distribution<> height_distribution(1, 280);

    // Guarda en el array los puntos aleatorios
    arrayOfParticles[i]  = point(width_distribution(motor),height_distribution(motor));
    }
}
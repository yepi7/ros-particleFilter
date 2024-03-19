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

#include <SFML/Graphics.hpp>

namespace bg = boost::geometry;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::linestring<point> linestring;

// Prototipos de funciones
void calculateDistance(point sonar, std::vector<box> obstaculos, int laserRadius);
void particlesGeneration (int numberOfParticles, Particle arrayOfParticles[]);

// Definicion de una Particula. No se agrega la distancia porque NO forma parte de la particula.
// Es un valor que se obtiene tras su creacion.
struct Particle {
    point position;
    float direction;
};

// ========================================================
// Funcion main
// ========================================================

int main(int argc, char** argv){

    // ========================================================
    // Declaracion de las variables
    // ========================================================

    // Definicion de parametros
    const int laserRadius = 300;
    const int numberOfParticles = 1;
    point sonarPositions[3];
    Particle arrayOfParticles [numberOfParticles]; // Array con Particulas aleatorias
    // point arrayOfParticles [numberOfParticles] = {point(0.0,0.0)};
    float distancesOfPartciles [numberOfParticles][360]; // Guarda las distancias de las particulas en cada iteracion
    
    // Simula el movimiento del robot
	sonarPositions[0] = point(0.0,0.0);
    sonarPositions[0] = point(0.5,0.0);
    sonarPositions[0] = point(1.0,0.0);

	// Definicion de los obstaculos. De momento son 3.
    std::vector<box> obstaculos = {
        // box(point(-5, -5), point(-3, 0)),
        // box(point(2, 1), point(4, 3)),
        // box(point(5, -2), point(7, 1))
        box(point(-10, 20), point(10, 40)), // (0, 30)
        box(point(40, -15), point(60, 15)), // (50, 0)
        box(point(-115, -10), point(-85, 10)) // (-100, 0)
    };
    
    // ========================================================
    // Comienzo del programa
    // ========================================================
    

    // Genera las particulas y las guarda en arrayOfPartciles
    particlesGeneration(numberOfParticles, arrayOfParticles);

	// Calculamos las distancias de las particulas a los obstaculos
	for (int i=0; i<numberOfParticles; i++){
		std::cout << "-----------------------Comienzo particula-----------------------------" << std::endl;
        std::cout << "POSICION:"<< bg::get<0>(arrayOfParticles[i].position)<<"," <<bg::get<1>(arrayOfParticles[i].position) << std::endl;
		
        calculateDistance(arrayOfParticles[i].position, obstaculos, laserRadius);
        
        std::cout << "-----------------------Fin Particula-----------------------------" << std::endl;
	}

    // ========================================================
    // Graficacion de los punntos
    // ========================================================
    sf::RenderWindow window(sf::VideoMode(400,300), "Puntos con SFML");

    // Define algunos puntos como círculos pequeños
    sf::CircleShape particles[numberOfParticles];
    sf::CircleShape axisCenter;
    axisCenter.setRadius(2);
    axisCenter.setFillColor(sf::Color::White);
    axisCenter.setPosition(200,150);
    for(int i=0; i<numberOfParticles; i++){
        particles[i].setRadius(3);
        particles[i].setFillColor(sf::Color::Red);
        // Aumentamos el offset con 10 ya que es el error en la medida para el grosor del borde del mapa
        particles[i].setPosition(bg::get<0>(arrayOfParticles[i])+210, bg::get<1>(arrayOfParticles[i])+160);
    }

    // Definicion de los obstaculos
    const int numberOfObstacles = 3;
    sf::RectangleShape obstacles[numberOfObstacles];
    // Configuración de ejemplo para obstáculos
    obstacles[0].setSize(sf::Vector2f(20, 20)); // Tamaño del obstáculo
    obstacles[0].setPosition(200, 180); // Posición
    obstacles[0].setOutlineColor(sf::Color::Red); // Color del borde
    obstacles[0].setOutlineThickness(3); // Grosor del borde
    obstacles[0].setFillColor(sf::Color::Transparent); // Color interior

    obstacles[1].setSize(sf::Vector2f(20, 30));
    obstacles[1].setPosition(250,150);
    obstacles[1].setOutlineColor(sf::Color::Green); // Color del borde
    obstacles[1].setOutlineThickness(3); // Grosor del borde
    obstacles[1].setFillColor(sf::Color::Transparent); // Color interior

    obstacles[2].setSize(sf::Vector2f(30, 20));
    obstacles[2].setPosition(100, 150);
    obstacles[2].setOutlineColor(sf::Color::Blue); // Color del borde
    obstacles[2].setOutlineThickness(3); // Grosor del borde
    obstacles[2].setFillColor(sf::Color::Transparent); // Color interior

    // Bucle principal de la ventana
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed)
                window.close();
        }

        window.clear(); // Limpia la ventana

        // Dibuja los puntos
        window.draw(axisCenter);
        for(int i=0; i<numberOfParticles; i++){
            window.draw(particles[i]);
        }
        
        for(int i=0; i<numberOfObstacles; i++){
            window.draw(obstacles[i]);
        }

        window.display(); // Muestra el contenido de la ventana
    }

	return 0;
}

// ========================================================
// Definicion de funciones externas
// ========================================================

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

void particlesGeneration (int numberOfParticles, Particle arrayOfParticles[]){
    // Motor de generación de números aleatorios
    std::mt19937 motor(static_cast<unsigned int>(std::chrono::steady_clock::now().time_since_epoch().count()));

    for (int i=0; i<numberOfParticles; i++){
    // Distribución uniforme entre 1 y 100 (inclusive)
    std::uniform_int_distribution<> width_distribution(-190, 190);
    std::uniform_int_distribution<> height_distribution(-140, 140);
    std::uniform_real_distribution<> direction_distribution(-1,1);
    // Guarda en el array los puntos aleatorios.
    arrayOfParticles[i].position  = point(width_distribution(motor),height_distribution(motor));
    arrayOfParticles[i].direction = direction_distribution(motor)*M_PI;
    }
}
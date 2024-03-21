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

/* 
Mapa de 400 (width) x 300 (height). Tiene paredes, por lo que asumiremos que ocupan 10 pixeles de grosor.
*/

namespace bg = boost::geometry;
typedef bg::model::point<float, 2, bg::cs::cartesian> point;
typedef bg::model::box<point> box;
typedef bg::model::linestring<point> linestring;

// Definicion de una Particula. No se agrega la distancia porque NO forma parte de la particula.
// Es un valor que se obtiene tras su creacion.
struct Particle {
    point position;
    float direction;
};

// Prototipos de funciones
void calculateDistance(point sonar, std::vector<linestring> obstaculos, int laserRadius, float arrayOfDistances[]);
void particlesGeneration (int numberOfParticles, Particle arrayOfParticles[]);
void creacionObstaculos(std::vector<linestring>& obstaculos);
float distanciaEuclidea (float *array1, float *array2, int longitud);

// ========================================================
// Funcion main
// ========================================================

int main(int argc, char** argv){

    // ========================================================
    // Declaracion de las variables
    // ========================================================

    // Definicion de parametros
    const int laserRadius = 250;
    const int numberOfParticles = 100;
    std::vector<linestring> obstaculos;
    point sonarPositions[6];
    Particle arrayOfParticles [numberOfParticles]; // Array con Particulas aleatorias
    float distanceComparison [numberOfParticles]; // Array con distancia euclidea

    // Creacion del mapa
    creacionObstaculos(obstaculos);

    // Matriz de particulas y distancias
    float distancesOfParticles [numberOfParticles][360]; // Guarda las distancias de las particulas en cada iteracion
    // Matriz de distancias reales del robot
    float realDistancesOfRobot [6][360]; // Se crean 6 posiciones. Estan hardcodeadas. Modificar despues.
    
    // Simula el movimiento del robot. Posiciones reales
	sonarPositions[0] = point(0.0,0.0);
    sonarPositions[1] = point(0.5,0.0);
    sonarPositions[2] = point(1.0,0.0);
    sonarPositions[3] = point(1.5,0.0);
    sonarPositions[4] = point(2,0.0);
    sonarPositions[5] = point(2.5,0.0);


    // Obtenemos las distancias desde las posiciones reales. Seran las distancias reales (medidas con sensor).
    for (int i=0;i<6;i++){
        calculateDistance(sonarPositions[i],obstaculos,laserRadius,realDistancesOfRobot[i]);
        //// Para visualizar la matriz de distancias 
        // std::cout << "Fila: " << i <<":"<< std::endl;
        // for (int j=0;j<360;j++){
        //     std::cout << realDistancesOfRobot[i][j] << " ";
        // }
        // std::cout << std::endl;
    }
    
    // Genera las particulas y las guarda en arrayOfPartciles
    particlesGeneration(numberOfParticles, arrayOfParticles);

    // Calculamos las distancias de las particulas a los obstaculos
    for (int i=0; i<numberOfParticles; i++){
        calculateDistance(arrayOfParticles[i].position,obstaculos,laserRadius,distancesOfParticles[i]);
        //// Para visualizar la matriz de distancias 
        // std::cout << "Particula: " << i <<"=("<< bg::get<0>(arrayOfParticles[i].position)<<","<<bg::get<1>(arrayOfParticles[i].position)<<")"<<std::endl;
        // for (int j=0;j<360;j++){
        //     std::cout << distancesOfParticles[i][j] << " ";
        // }
        // std::cout << std::endl;
    }
    
    // Calculamos la diferencia entre cada distancia en funcion del angulo.
    // En este caso se comparara con la primera particula "realDistancesOfRobot[0]"
    for (int i=0;i<numberOfParticles;i++){
        distanceComparison[i] = distanciaEuclidea(realDistancesOfRobot[0],distancesOfParticles[i],360);
        // Para visualizar la matriz de distancias 
        std::cout << "Particula: " << i <<"=("<< bg::get<0>(arrayOfParticles[i].position)<<","<<bg::get<1>(arrayOfParticles[i].position)<<")"<<std::endl;
        std::cout << distanceComparison[i] << std::endl;
    }
    
    // Para asignar un PESO, dividir entre la distancia maxima posible, es decir, entre sqrt(200^2 + 150^2).
    

    // ========================================================
    // Comienzo del programa
    // ========================================================
    


    /* arrayOfParticles[0].position = point(0,0);
    
	for (int i=0; i<numberOfParticles; i++){
		std::cout << "-----------------------Comienzo particula-----------------------------" << std::endl;
        std::cout << "POSICION:"<< bg::get<0>(arrayOfParticles[i].position)<<"," <<bg::get<1>(arrayOfParticles[i].position) << std::endl;
		
        calculateDistance(arrayOfParticles[i].position, obstaculos, laserRadius);
        
        std::cout << "-----------------------Fin Particula-----------------------------" << std::endl;
	} */

    // ========================================================
    // Graficacion de los punntos
    // ========================================================
    
    /* sf::RenderWindow window(sf::VideoMode(400,300), "Puntos con SFML");

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
        // Muestra el contenido de la ventana
        window.display(); 
    } */

	return 0;
}

// ========================================================
// Definicion de funciones externas
// ========================================================

/* Revisar, no obtieene distancias correctas con objetos tipo box, pero si con linestring. Las paredes estan
    ya puestas como linestring */

// No es necesario poner obstaculos como parametro, ya que el mapa es conocido. Puede ir en el main.
// La funcion calcula la distancia en funcion del radio del laser.
void calculateDistance(point sonar, std::vector<linestring> obstaculos, int laserRadius, float arrayOfDistances[]){
	for (int angle = 0; angle < 360; angle += 1) {
        float rad = angle * M_PI / 180.0;
        point direccion(std::cos(rad) * laserRadius, std::sin(rad) * laserRadius);

        linestring rayo;
        rayo.push_back(sonar);
        rayo.push_back(direccion);

        // Buscamos si existen intersecciones
        std::vector<linestring> intersecciones;
        // Busca intersecciones en los 360 grados. Si hay obstaculo, habra intersecccion.
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
                    arrayOfDistances[angle] = distancia; 
                    // std::cout << "Ángulo " << angle << " grados: Obstáculo detectado a " << distancia << " unidades." << std::endl;
                    break; // Supone que solo nos interesa el primer obstáculo detectado
                }
            }
        } else {
            // std::cout << "Ángulo " << angle << " grados: Sin obstáculo." << std::endl;
            arrayOfDistances[angle] = std::numeric_limits<float>::infinity(); // Se asigna un numero muy grande ya que se buscara el minimo.
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

void creacionObstaculos(std::vector<linestring>& obstaculos){
    // Definición de las paredes del mapa como linestrings
    linestring pared_izquierda;
    pared_izquierda.push_back(point(-200, -150)); // Vértice inferior izquierdo
    pared_izquierda.push_back(point(-200, 150)); // Vértice superior izquierdo

    linestring pared_derecha;
    pared_derecha.push_back(point(200, -150)); // Vértice inferior derecho
    pared_derecha.push_back(point(200, 150)); // Vértice superior derecho

    linestring pared_superior;
    pared_superior.push_back(point(-200, 150)); // Vértice superior izquierdo
    pared_superior.push_back(point(200, 150)); // Vértice superior derecho

    linestring pared_inferior;
    pared_inferior.push_back(point(-200, -150)); // Vértice inferior izquierdo
    pared_inferior.push_back(point(200, -150)); // Vértice inferior derecho

    // Vector que contiene las paredes del mapa
    obstaculos = {pared_izquierda, pared_derecha, pared_superior, pared_inferior};
}

float distanciaEuclidea (float *array1, float *array2, int longitud){
    float distancia = 0.0;
    for (int i=0; i<longitud; i++){
        distancia += pow(array2[i] - array1[i], 2);
    }
    return sqrt(distancia);
}
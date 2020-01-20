/* rrtAlg.cpp
Performs a rapid random tree search over a 2D domain with blockades in a ros environment
Data typing based on: https://www.geeksforgeeks.org/shortest-path-for-directed-acyclic-graphs/
By: Sam Migirditch migirditch@gmail.com
Last edit: 01/14/2020
*/

#include <cmath>
#include <iostream>
#include <random>
#include <list>

// Definitions
const unsigned int RANDOM_SEED = 41407283;
const double WIDTH = 100;
const double HEIGHT = 50;
const int VERTICIES = 500;

// Global functions
double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

// Global classes and structs to emulate ROS objects

struct point // points in grid for start and stop points
{
    // Default public members
    double x,y, distance;

    // Constructor
    point(int initPointIndx, double initX, double initY)
    {
        int pointIndx = initPointIndx;
        x = initX;
        y = initY;
        double distance = MAXFLOAT;
        std::list<int> *children;
    }

    // Public member function
    void randomPosition(double width, double height);
    void refreshDistance(point *target);
    void placeValidPoint();

};

class graph
{
    int size; // number of verticies
    std::list<point> *nodes;
    // private fxn
    void refreshSize(){this -> size = nodes.size() }

public:
    graph(int initSize);
    int getSize() {this -> refreshSize(); return(graph.size);}
    void addPoint(point *pointptr);
};

struct obst // Obsticles in plane
{

};

int main()
{
    // Start randomization engine
    srand(RANDOM_SEED);
    // Preallocate adj. matrix

    // Place start and end points in opposite corners
    point startPoint(0, -WIDTH,-HEIGHT);
    point endPoint(VERTICIES+1, WIDTH,HEIGHT);
    startPoint.refreshDistance(&endPoint);


    return(0);
}

// Support functions

void point::randomPosition(double width, double height)
{
    this -> x = fRand( -WIDTH, WIDTH);
    this -> y = fRand( -HEIGHT, HEIGHT);
}

void point::refreshDistance(point *target)
{
    double localX = this -> x;
    double localY = this -> y;
    double targetX = target -> x;
    double targetY = target -> y;

    this -> distance = sqrt( pow( targetX-localX , 2) + pow(targetY-localY , 2) );
}

void point::placeValidPoint();
{
    int tries = 100;

    for( int i=0;i<100;i++)
    {

    }
    
}
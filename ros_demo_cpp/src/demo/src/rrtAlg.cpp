/* rrtAlg.cpp
Performs a rapid random tree search over a 2D domain with blockades in a ros environment
By: Sam Migirditch migirditch@gmail.com
Last edit: 01/14/2020
*/

#include <cmath>
#include <iostream>
#include <random>

// Definitions
const unsigned int RANDOM_SEED = 41407283;
const double WIDTH = 100;
const double HEIGHT = 50;
const int VERTICIES = 500;


// Global classes and structs to emulate ROS objects

struct point // points in grid for start and stop points
{
    // Default public members
    double x,y;

    // Constructor
    point(double initX, double initY)
    {
        x = initX;
        y = initY;
    }

    // Public member function
    void randomPosition(double width, double height);
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
    point startPoint(-WIDTH,-HEIGHT);
    point endPoint(WIDTH,HEIGHT);

    return(0);
}

// Support functions

void point::randomPosition(double width, double height)
{
    this -> x = rand();
    this -> y = rand();
}


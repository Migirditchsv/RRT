/* rrt.hpp
    functions for building a rapidly expanding random tree and finding near optimal paths.
    by: Sam Migirditch
    migirditch@gmail.com
*/

//Header guards
#ifndef RAPIDRANDOMTREE
#define RAPIDRANDOMTREE

// Standard Lib Include
#include <iostream> // output
#include <random> //prng for random point placement
// Ros include
#include "ros/ros.h"
#include "std_msgs/String.h"
// system include
#include <visualization_msgs/Marker.h>
#include <sstream>

// Constants
#define RANDOMSEED 1993
#define WIDTH 10.0

// Use namespaces
using namespace std;

// Global objects
std::mt19937 gen(RANDOMSEED); 

// Global functions
double fRandom(double fMin, double fMax)
{
    std::uniform_real_distribution<> dist(fMin, fMax);
    double fRand = dist(gen);// use the random engine to pull somehting out of dist
    return(fRand);
}

int testFunction(int i)
{
    return(i);
}

// Structs
class publishVisMarker
{
    private:
        ros::Publisher pub;
        ros::NodeHandle n;
    public:

        publish()
        {
            pub = n.advertise<visualization_msgs::Marker> ("visualization_marker", 10);
        }
};

class subscribeVisMarker
{
    private:
        ros::Subscriber sub;
        ros::NodeHandle n;
    public:

        subscribe()
        {
            sub = n.subscribe<visualization_msgs::Marker> ("visualization_marker", 10, &subscribeVisMarker::callback, this);
        }

        void callback(const visualization_msgs::Marker& obst)
        {
            cout<<" rrt::Class subscribeVisMarker:  callback: "<< obst << endl;
        }
};

struct point
{
    public:

    // Public Vars
    double x,y; // position
    int index, parentIndex; // index for incluison in vector of points & index of parent. 
    point *parent; // pointer to unique parent

    // Public Functions
    void randomEdge(visualization_msgs::Marker obst);
    int nearestNeighbor(vector<point> tree); // return index of nearest neighbor in tree

    private:

    // Private Vars
 

    // Private fxns
    void validPoint(visualization_msgs::Marker obst);
    double pointDistance(point target);


};

struct rrtSearch
{

vector<point> tree; // where it all happens.
vector<int> shortestPath;
subscribeVisMarker obstSubscription; 


};

// Point Functions

void point::randomEdge(visualization_msgs::Marker obst)
{   
    bool condition = true;
    while( condition == true )
    {
        this->x = fRandom(-WIDTH,WIDTH);
        this->y = fRandom(-WIDTH,WIDTH);
        condition = false;
    }
    return;
}

int nearestNeighbor(vector<point> tree)
{
    return(0);
}

void validPoint(visualization_msgs::Marker obst)
{
    return;
}

double pointDistance(point target)
{
    return(0);
}


#endif


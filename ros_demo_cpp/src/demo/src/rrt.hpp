/* rrt.hpp
    functions for building a rapidly expanding random tree and finding near optimal paths.
    by: Sam Migirditch
    migirditch@gmail.com
*/

//Header guards
#ifndef RAPIDRANDOMTREE
#define RAPIDRANDOMTREE

// Include
#include <iostream> // output
#include <random> //prng for random point placement// Ros include
#include "ros/ros.h"
#include "std_msgs/String.h"
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
void visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    int length;
    cout << "visMsgSybscriber received: " << msg->ns <<endl;

    const std::string ns = msg->ns;

   if( ns == "Goal Points")
   {    
       length = msg->points.size();
        // write to start and end pts
        cout<<"rrt::vizMarkerCallback| is a goal"
        <<"\n\tmsg->length:"<<length<<endl;

        // push to start and end points pointed to by constructor from rrtSolve. 


   }
    else if( ns == "obstacles")
    {
        // write to obst vec
        cout<<"rrt::vizMarkerCallback| is a obst"<<endl;
    }
    else if( ns == "Boundary")
    {
        // write to obst vec
        cout<<"rrt::vizMarkerCallback| is a Boundary"<<endl;
    }
    else if( ns == "vertices_and_lines")
    {
        // ignore
        cout<<"rrt::vizMarkerCallback| is a vertices_and_lines"<<endl;
    }
    else if( ns == "rob")
    {
        // maybe use for truthing
        cout<<"rrt::vizMarkerCallback| is a rob"<<endl;
    }
    else
    {
        cerr<<"rrt::visMarkerCallback ERROR: msg->ns of unknown category:"<<ns<<endl;
        exit(0);
    }
    return;
}

struct visMsgSubscriber
{
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<visualization_msgs::Marker>("visualization_marker", 100,
                                      visMarkerCallback);
    visualization_msgs::Marker startPoint;
    visualization_msgs::Marker endPoint;
    
    private:

};

struct point
{
    public:

    // Public Vars
    double x,y; // position
    int index, parentIndex; // index for incluison in vector of points & index of parent. 
    point *parent; // pointer to unique parent
    visualization_msgs::Marker obst;

    // Public Functions
    void randomEdge();
    int nearestNeighbor(vector<point> tree); // return index of nearest neighbor in tree

    private:

    
 

    // Private fxns
    void validPoint(visualization_msgs::Marker obst);
    double pointDistance(point target);


};

struct rrtSearch
{

public:
    vector<point> tree; // where it all happens.
    vector<int> shortestPath;
    visMsgSubscriber visSub;


    
private:
    
};

// Point Functions

void point::randomEdge()
{   
    bool tryCondition = true;
    while( tryCondition == true )
    {
        this->x = fRandom(-WIDTH,WIDTH);
        this->y = fRandom(-WIDTH,WIDTH);
    }
}

#endif


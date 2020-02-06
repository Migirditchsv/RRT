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
#include <random> //prng for random point placement

// Use namespaces
using namespace std;

// GLobal Objects

// Global functions
int testFunction(int i)
{
    return(i);
}

// Structs
struct point
{
    public:

    // Public Vars
    double x,y; // position
    int index, parentIndex; // index for incluison in vector of points & index of parent. 
    point *parent; // pointer to unique parent

    // Public Functions
    void randomPlacement(visualization_msgs::Marker obst);
    int nearestNeighbor(vector<point> tree); // return index of nearest neighbor in tree

    private:

    // Private Vars

    // Private fxns
    void validPoint(visualization_msgs::Marker obst);
    double pointDistance(point target);



}

#endif
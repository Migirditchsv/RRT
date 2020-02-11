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
#include <random> //prng for random point placement// Ros include
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>

// Constants
#define RANDOMSEED 1993
#define WIDTH 10.0
#define PLACEMENT_TRIES 100 // how many times to try placing a random pt outside of an obsticle before assuming something is wrong

// Use namespaces
using namespace std;

// Global objects
std::mt19937 gen(RANDOMSEED); 

// forward declarations
struct rrtSearch;
struct point;
struct visMsgSubscriber;

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

class testclass
{
public:
    int testInt = 3;
private:
    int privateTestInt = 4;
};

// Structs
struct point
{
    public:

    // Public Vars
    double x,y; // position
    int index, parentIndex; // index for incluison in vector of points & index of parent. 
    point *parent; // pointer to unique parent

    // Public Functions

    //constructor
    point(int _x, int _y)
    {
        this -> x = _x;
        this -> y = _y;
        this -> index = -1;
        this -> parentIndex = -1;
        this -> parent = NULL;
    }
    private:
    // Private fxns
    double pointDistance(point target);
};

struct rrtSearch
{

public:
    // public vars

    // public fxns
    void setTreeSize(int _treeSize){treeSize = _treeSize;}
    void setObst(const visualization_msgs::Marker::ConstPtr& newObst);
    void setGoalPoints(const visualization_msgs::Marker::ConstPtr& newPoint); // = not defined for marker to point
    void addRandomEdge();

    rrtSearch(int _treeSize)
    {
        cout<<" rrt.hpp| rrtSearch(int treeSize): constructor called"<<endl;
        setTreeSize(_treeSize);
        tree.resize(2);
        cout<<" rrt.hpp| rrtSearch(int treeSize): treeSize set"<<endl;

        cout<<" rrt.hpp| rrtSearch(int treeSize): constructor complete"<<endl;
    }

    
private:
    visualization_msgs::Marker obst;
    int treeSize;
    vector<point> tree; // where it all happens.
    vector<int> shortestPath;
    // sub fxns & objects
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<visualization_msgs::Marker>("visualization_marker", 100, &rrtSearch::visMarkerCallback,this);
    visualization_msgs::Marker startPoint;
    visualization_msgs::Marker endPoint;
    void visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void validRandomPoint(int pointIndx);
    void addPointToTree();
};

// Point Functions


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


// pub sub fxns

void rrtSearch::visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    int length;
    cout << "visMarkerCallback received: " << msg->ns <<endl;

    const std::string ns = msg->ns;

   if( ns == "Goal Points")
   {    
       length = msg->points.size();
        // write to start and end pts
        cout<<"rrt::vizMarkerCallback| is a goal"
        <<"\n\tmsg->length:"<<length<<endl;

        // push to start and end points pointed to by constructor from rrtSolve. 
        this->setGoalPoints(msg);

   }
    else if( ns == "obstacles")
    {
        // write to obst vec
        cout<<"rrt::vizMarkerCallback| is a obst"<<endl;
        this->setObst(msg);
    }
    else if( ns == "Boundary")
    {
        // write to boundary vec
        //cout<<"rrt::vizMarkerCallback| is a Boundary"<<endl;
    }
    else if( ns == "vertices_and_lines")
    {
        // ignore
        //cout<<"rrt::vizMarkerCallback| is a vertices_and_lines"<<endl;
    }
    else if( ns == "rob")
    {
        // maybe use for truthing
        //cout<<"rrt::vizMarkerCallback| is a rob"<<endl;
    }
    else
    {
        cerr<<"rrt::visMarkerCallback ERROR: msg->ns of unknown category:"<<ns<<endl;
        exit(0);
    }
    return;
}

// rrtSearch member fxns

void rrtSearch::setObst(const visualization_msgs::Marker::ConstPtr& newObst)
{
    obst = *newObst;// remember to dereffrence the message pointer into the visMsg it points to
    //cout<<"rrtSearch::setObst: obst\n: "<<obst<<"\n"<<endl; // yup it's importing correctly. 
    return;
}


void rrtSearch::setGoalPoints(const visualization_msgs::Marker::ConstPtr& newPoint) // = not defined for marker to point
{
tree[0].x = newPoint->points[0].x;
tree[0].y = newPoint->points[0].y;
tree[0].parentIndex = 0; //start is it's own parent
tree[0].parent = &tree[0]; 
tree[1].x = newPoint->points[1].x;
tree[1].y = newPoint->points[1].y;
return;
}

void rrtSearch::addRandomEdge()
{   
    // find a valid random point
    this -> validRandomPoint(); //*** resume here
}

void rrtSearch::validRandomPoint(int pointIndx)
{
    double tempX, tempY;
    int obstSize = sizeof(obst.points);
    cout<<"rrtSearch::validRandomPoint: obstSize: "<<obstSize<<endl;

    for(int i = 0; i<PLACEMENT_TRIES; i++)
    {
        // Guess a point
        tempX = fRandom(-WIDTH,WIDTH);
        tempY = fRandom(-WIDTH,WIDTH);

        for( int j = 0; j < obstSize; j++)
        {
            int objectType = obst.type;

        }

}

void addTreePoint()
{
    int currentTreeSize = sizeof(this.tree);
    // makesure we have points to add.
    if(currentTreeSize >= treeSize)
    {
        cerr<<"rrtSearch::addTreePoint: OUT OF POINTS TO ADD. SEARCH FAILED."<<endl;
        exit(0);
    }
    // add point to tree
    point newPoint;
    this.tree.push_back()
    // initialize points in tree to be invisable, have -1 as a parent index and a null pointer as a parent pointer
    tree[i].x = -WIDTH;
    tree[i].y = -WIDTH;
    tree[i].index = i;
    tree[i].parentIndex = -1;
    // use a null pointer to cause a program crash (address NULL=0 is reserved for the OS) if uninited point is accessed. 
    tree[i].parent = NULL;
    cout<<" rrt.hpp| rrtSearch(int treeSize): tree["<<i<<"] inited"<<endl;

}

#endif

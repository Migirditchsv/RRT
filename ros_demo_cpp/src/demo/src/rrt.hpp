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
    point()
    {
        this -> x = -WIDTH; //init to bottom corner
        this -> y = -WIDTH;
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
    void setTreeSize(uint _treeSize);
    void addRandomEdge();

    rrtSearch(int _treeSize)
    {
        cout<<" rrt.hpp| rrtSearch(int treeSize): constructor called"<<endl;
        setTreeSize(_treeSize);
        
    }

    
private:
    // private PODs
    int treeSize;
    // private objects
    vector<visualization_msgs::Marker> obst;
    vector<point> tree; // where it all happens.
    vector<int> shortestPath;
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<visualization_msgs::Marker>("visualization_marker", 100, &rrtSearch::visMarkerCallback,this);
    visualization_msgs::Marker startPoint;
    visualization_msgs::Marker endPoint;
    //private fxns
    void setObst(const visualization_msgs::Marker::ConstPtr& newObst);
    void setGoalPoints(const visualization_msgs::Marker::ConstPtr& newPoint); // = not defined for marker to point
    void visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void addTreePoint();
    void validRandomPoint(int pointIndex);
    bool validEdge(int pointIndex); // returns 0 if edge fails
    int  nearestNeighbor(int pointIndex); // nearest in tree
 
};

// Point Functions
double point::pointDistance(point target)
{
    double dx = pow( (this -> x - target.x), 2);
    double dy = pow( (this -> y - target.y), 2);
    double dist = sqrt( dx + dy );
    return(dist);
}

// rrtSearch member fxns

void rrtSearch::setTreeSize(uint newTreeSize)
{
    tree.resize(newTreeSize + 2);
}

void rrtSearch::setObst(const visualization_msgs::Marker::ConstPtr& newObst)
{
    int id, checkId, obstSize;
    bool newEntry;
    visualization_msgs::Marker holder = *newObst;

    //cout<<"rrtSearch::setObst: obstMsg:\n\n\n\n"<<holder.id<<endl;
    id = holder.id;
    // check if novel
    newEntry = true;
    //cout<<"rrtSearch::setObst: SEG0"<<endl;
    for(int i = 0; i<obst.size(); i++)
    {
        checkId = obst[i].id;
        if(checkId==id){newEntry=false;break;}
    }
    if(newEntry)
    {
        //cout<<"rrtSearch::setObst: NEW OBSTICLE ID#: "<<id<<endl;
        obst.push_back(holder);}
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

int rrtSearch::nearestNeighbor(int pointIndex)
{   
    
    return(0);
}

void rrtSearch::addTreePoint()
{
    int position = sizeof(this -> tree);
    // makesure we have points to add.
    if(position >= treeSize)
    {
        cerr<<"rrtSearch::addTreePoint: OUT OF POINTS TO ADD. SEARCH FAILED."<<endl;
        exit(0);
    }
    // add point to tree
    point newPoint;
    this -> tree.push_back(newPoint);
    // initialize points in tree to be invisable, have -1 as a parent index and a null pointer as a parent pointer
    tree[position].x = -WIDTH;
    tree[position].y = -WIDTH;
    tree[position].index = position;
    tree[position].parentIndex = -1;
    // use a null pointer to cause a program crash (address NULL=0 is reserved for the OS) if uninited point is accessed. 
    tree[position].parent = NULL;
    cout<<" rrt.hpp| rrtSearch(int treeSize): tree["<<position<<"] inited"<<endl;
    return;
}

void rrtSearch::addRandomEdge()
{   
    int newPointIndex;

    // attempt create the point to place
    this -> addTreePoint();
    // get index of newPoint
    newPointIndex = sizeof(tree) - 1;
    // find a valid random point
    this -> validRandomPoint(newPointIndex);
    return;
}

void rrtSearch::validRandomPoint(int pointIndex)
{
    double tempX, tempY, scaleX, scaleY, obstX, obstY, w;
    int obstSize = sizeof(obst);
    cout<<"rrtSearch::validRandomPoint: obstSize: "<<obstSize<<endl;

    for(int i = 0; i<PLACEMENT_TRIES; i++)
    {
        // Guess a point
        tempX = fRandom(-WIDTH,WIDTH);
        tempY = fRandom(-WIDTH,WIDTH);

        for( int j = 0; j < obstSize; j++)
        {/*
            //get obj generic info
            int objectType = obst.type;
            double obstX = obst.pose.position.x;
            double obstY = obst[i].pose.position.y;

            switch(objectType)
            {
                case 1: // Cube

                scaleX = obst[i].scale.x / 2.0;
                scaleY = obst[i].scale.y / 2.0;

                // rotation check
                if( obst[i].pose.orientation.w!=1.0)
                {
                scaleX = 2.5;
                scaleY = 0.3;
                }

            }


        */}
    }
    return;
}

bool rrtSearch::validEdge(int pointIndex)
{
    return( false );// holder
}


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
        cout<<"rrt::vizMarkerCallback| is a obst:\n"<<*msg<<endl;
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


#endif

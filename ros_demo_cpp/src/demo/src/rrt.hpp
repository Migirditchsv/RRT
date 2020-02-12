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
#define RANDOMSEED 19937
#define WIDTH 10.0
#define PLACEMENT_TRIES 100 // how many times to try placing a random pt outside of an obsticle before assuming something is wrong
#define EPSILON 0.001 // stepsize for clipping

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
    double pointDistance(point target);

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
};

struct rrtSearch
{

public:
    // Public Objects
    ros::NodeHandle rrt_node;
    // public vars
    bool complete;
    // public fxns
    void setTreeSize(uint _treeSize){ treeSize = _treeSize; }
    int getTreeSize(){return( tree.size() );}
    void addEdge();

    rrtSearch(int _treeSize)
    {
        cout<<" rrt.hpp| rrtSearch(int treeSize): constructor called"<<endl;
        setTreeSize(_treeSize);

        complete = false; //start incomplete        
    }

    ~rrtSearch()
    {
        cout<<"\n\n\n\n WARNING RRTSEARCH STRUCT DESTROYED \n\n\n\n"<<endl;
    }

    
private:
    // private PODs
    int treeSize;
    // private objects
    vector<visualization_msgs::Marker> obst;
    vector<point> tree; // where it all happens.
    vector<int> shortestPath;
    ros::Subscriber sub = rrt_node.subscribe<visualization_msgs::Marker>("visualization_marker", 100, &rrtSearch::visMarkerCallback,this);
    visualization_msgs::Marker startPoint;
    visualization_msgs::Marker endPoint;
    //private fxns
    void setObst(const visualization_msgs::Marker::ConstPtr& newObst);
    void setGoalPoints(const visualization_msgs::Marker::ConstPtr& newPoint); // = not defined for marker to point
    void visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg);
    void addTreePoint();
    bool validPoint(int pointIndex);
    void validRandomPoint(int pointIndex);
    bool validEdge(int nearPointIndex, int newPointIndex); // returns 0 if edge fails
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
{ //weird that poitns are published in pointlists but obstacles are not published in object lists. 
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
    int nearestIndex = -1; //segfault if no nearest
    double dist;
    double minDist = MAXFLOAT;

    for( int i = 0; i < tree.size(); i++)
    {
        // don't compare to self
        if( i ==  pointIndex ){continue;}
        dist = tree[pointIndex].pointDistance(tree[i]);
        if( dist <= minDist )
        {
            minDist = dist;
            nearestIndex = i;
        }
    }
    
    return(nearestIndex);
}

void rrtSearch::addTreePoint()
{
    int position = tree.size();
    // makesure we have points to add.
    if(position >= treeSize)
    {
        cerr<<"rrtSearch::addTreePoint: OUT OF POINTS TO ADD. SEARCH FAILED. TREE SIZE:"<<position<<endl;
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

void rrtSearch::addEdge()
{   
    int newPointIndex, nearPointIndex;
    int tries = 0;
    bool success = false;
cout<<"RRT SEG -1"<<endl;
    while( !success && tries < 5)
    {
        // get index of newPoint
        newPointIndex = sizeof(tree);
        // attempt create the point to place, exits if out of points
        this -> addTreePoint();
        // find a valid random point
        cout<<"RRT SEG 0"<<endl;
        this -> validRandomPoint(newPointIndex);
        // find nearest point to random point
        nearPointIndex = this -> nearestNeighbor(newPointIndex);
        cout<<"RRT SEG 1"<<endl;
        success = validEdge(nearPointIndex, newPointIndex);
        cout<<"RRT SEG 2"<<endl;
        //cout<<"\n\n rrt.hpp| SEG FLAG 0 \n"<<endl;
        tries++;
    }
    if(!success)
    {
        cerr<<"rrtSearch::addEdge() ERROR: UNABLE TO FIND INTER-POINT PATHS AFTER 5 ATTEMPTS. ABORTING"<<endl;
        exit(0);
    }
    // assign parent
    // assign indicies 
    // create and push visMsg to rviz
    // check for endstate
    cout<<"RRT SEG 3"<<endl;
    return;
}

bool rrtSearch::validPoint(int pointIndex) //return true if legal positoin false if else
{
    double dist, scaleX, scaleY, obstX, obstY;
    bool top, bottom, left, right;
    // shorthand pointIndex's coords
    double x = tree[pointIndex].x;
    double y = tree[pointIndex].y;

    // Loop through known objects
    for( int j = 0; j < obst.size() ; j++)
    {
        //get obj generic info
        int objectType = obst[j].type;
        double obstX = obst[j].pose.position.x;
        double obstY = obst[j].pose.position.y;


        switch(objectType)
        {
            case 1: // Cube

            scaleX = obst[j].scale.x / 2.0;
            scaleY = obst[j].scale.y / 2.0;

            // rotation check
            if( obst[j].pose.orientation.w != 1.0 )
            {
                scaleX = 2.5;
                scaleY = 0.3;
            }

            right = x < obstX + scaleX;
            left = x > obstX - scaleX;
            top = y < obstY + scaleY;
            bottom = y > obstY - scaleY;


            if( left and right and top and bottom )
            {   
                cout<<"rrtSearch::validPoint COLLISION CUBE # "<<j<<endl;
                return(false);
            }
            break; // break case 1 Cube

            case 3: // cyl

            scaleX = obst[j].scale.x / 2.0;
            dist = sqrt( pow(x - obstX, 2 ) + pow(y - obstY,2 ) );
            if( dist <= scaleX )
            {
                cout<<"rrtSearch::validPoint COLLISION CYL # "<<j<<endl;
                return(false);
            }
            break;

            default:
                cerr<<"rrtSearch::validRandomPoint: unkown object type: "<< objectType <<". EXITING."<<endl;
                exit(0);
            break; // default

        } // end switch
    } // end object for
    //cout<<"rrtSearch::validPoint POINT CLEAR OF ALL "<<obst.size()<<" OBSTACLES"<<endl;
    return(true);
}

void rrtSearch::validRandomPoint(int pointIndex)
{
    bool valid;
    double tempX, tempY;
    //cout<<"rrtSearch::validRandomPoint: obstSize: "<<obst.size()<<endl;

    for(int i = 0; i<PLACEMENT_TRIES; i++)
    {
        nextPoint:
        // Guess a point
        tree[pointIndex].x = fRandom(-WIDTH,WIDTH);
        tree[pointIndex].y = fRandom(-WIDTH,WIDTH);

        valid = this -> validPoint(pointIndex);
        //cout<<"rrtSearch::validRandomPoint: x y: "<<tree[pointIndex].x<<","<<tree[pointIndex].y<<" valid = "<< valid <<endl;

        if(valid)
        {
            return;
        }

    } // end placement for

    // Search has failed somehting is probably wrong
    cerr<<"rrtSearch::validRandomPoint: VALID POINT SEARCH HAS FAILED. EXITING."<<endl;
    exit(0);
}

bool rrtSearch::validEdge(int nearIndex, int newIndex) // moves newIndex to a legal edge position along near-new ray, deletes new if no such ray exists
{
    bool edgeValidity;
    int steps;
    double scale, dx, dy, startX, startY;

    dx = tree[newIndex].x - tree[nearIndex].x;
    dy = tree[newIndex].y - tree[nearIndex].y;

    startX = tree[nearIndex].x;
    startY = tree[nearIndex].y;


    steps = tree[nearIndex].pointDistance( tree[newIndex] ) / EPSILON;
    steps = min(steps, 2); // zero guard

    for(int i = 1; i<=steps; i++)// i=1 don't want initial overlap
    {
        scale = ( (double) i / (double) steps );

        // scale out to new point
        tree[newIndex].x = ( scale * dx ) + startX;
        tree[newIndex].y = ( scale * dy ) + startY;

        // check point validity
        edgeValidity = this -> validPoint(newIndex);

        // if edge fails go to previous point, if first point, delete newIndex and return false
        if( !edgeValidity )
        {
            if( i == 1 )
            {
                cout<<"\n\n\nrrtSearch::validEdge NO LEGAL EDGES EXIST. DISCARDING POINT "<<newIndex<<endl;
                tree.erase( tree.begin() + newIndex ); //this is not causeing the double free, it occurs when // out
                return(false);
            }
            else
            {
                scale = ( (double) (i-1) / (double) steps );    
                // scale out to new point
                tree[newIndex].x = ( scale * dx ) + startX;
                tree[newIndex].y = ( scale * dy ) + startY;
                return(true);
            }
        }

    } // full ray is valid
    
    return( true );// holder
}


void rrtSearch::visMarkerCallback(const visualization_msgs::Marker::ConstPtr& msg)
{
    //cout << "visMarkerCallback received: " << msg->ns <<endl;

    const std::string ns = msg->ns;

   if( ns == "Goal Points")
   {    
        this->setGoalPoints(msg);
   }
    else if( ns == "obstacles")
    {
        this->setObst(msg);
    }
    else if( ns == "Boundary")
    {
        // ignore
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

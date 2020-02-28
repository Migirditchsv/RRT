/* rrt.cpp
    functions for building a rapidly expanding random tree and finding near optimal paths.
    by: Sam Migirditch
    migirditch@gmail.com
*/

// Standard Lib Include
#include <iostream> // output
#include <random> //prng for random point placement// Ros include
// Ros include
#include "ros/ros.h"
#include "std_msgs/String.h"

// system include
#include <visualization_msgs/Marker.h>
#include <sstream>
// custom include
#include "rrt.hpp"


#define RRT_MAX_NODES 200 



// Use namespaces
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt"); // we're still in the ros_demo package
    ros::NodeHandle rrt_node;

    rrtSearch rrt( (int)RRT_MAX_NODES );
    ros::Subscriber rrtSub = rrt_node.subscribe<visualization_msgs::Marker>("visualization_marker", 15, &rrtSearch::visMarkerCallback, &rrt);
    ros::Publisher  rrtPub = rrt_node.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    //hot loop
    while( ros::ok() )
    {   
        if(rrt.getTreeSize() == 0 ){continue;} // skip if no info recieved from demo yet
        if(!rrt.complete)
        {
            rrt.addEdge();
            rrtPub.publish( rrt.getLineList() );
        }
        //elseif(rrt.complete){rrt.pubNextStep();} // publish next sollution step to rviz

        ros::spinOnce();
        ros::spinOnce();
    }



}
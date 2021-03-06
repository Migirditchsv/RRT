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

// Use namespaces
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rrt"); // we're still in the ros_demo package

    //each second, ros "spins" and draws 20 frames
    ros::NodeHandle rrt_node;
    // once a nodehandle exists, you can assign it a loop rate
    ros::Rate loop_rate(4);

    rrtSearch rrt(4000);
    cout<<"rrt.cpp: main(): rrt Created" <<endl;
    ros::Subscriber rrtSub = rrt_node.subscribe<visualization_msgs::Marker>("visualization_marker", 15, &rrtSearch::visMarkerCallback, &rrt);
    ros::Publisher  rrtPub = rrt_node.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    //ros::Subscriber rrtdebugsub = rrt_node.subscribe<visualization_msgs::Marker>("visualization_marker", 15, &rrtSearch::visMarkerCallback, &rrt);
    cout<<"rrt.cpp: main(): pub/sub created"<<endl;
    //hot loop
    cout<<"rrt.cpp: main(): ros::ok()"<<ros::ok()<<endl;
    static int rrtLoopCounter = 0;
    while( ros::ok() )
    {   
        cout<<"rrt.cpp: main() loop #: "<<rrtLoopCounter<<endl;
        //if(rrt.getTreeSize() == 0 ){continue;} // skip if no info recieved from demo yet
        if(!rrt.complete)
        {
            cout<<"rrt.cpp: main(): adding edge"<<endl;
            rrt.addEdge();
            visualization_msgs::Marker lineList;
            lineList = rrt.getLineList();
            cout<<"rrt.cpp: main(): publishing linelist" << endl;
            rrtPub.publish( lineList );

            // test

            visualization_msgs::Marker testObj;
            testObj.type = visualization_msgs::Marker:: CUBE;
            testObj.header.frame_id = "map";
            testObj.header.stamp = ros::Time::now();
            testObj.ns = "obstacles";
            testObj.id = 999;
            testObj.action = visualization_msgs::Marker::ADD;
            testObj.scale.x = 3;
            testObj.scale.y = 1;
            testObj.scale.z = 5;
            testObj.pose.position.z = 0.0;
            testObj.pose.position.x = testObj.pose.position.y = 0.0;
            testObj.pose.orientation.x = testObj.pose.orientation.y = testObj.pose.orientation.z = 0;
            testObj.pose.orientation.w = 1;
            testObj.color.r = 1.0f;
            testObj.color.a = 1.0;
            rrtPub.publish(testObj);
        }
        //elseif(rrt.complete){rrt.pubNextStep();} // publish next sollution step to rviz
        ros::spinOnce();
        //loop_rate.sleep();
        rrtLoopCounter++;
    }



}
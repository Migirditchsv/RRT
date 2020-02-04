/*
For class E502 Introduction to Cyber Physical Systems
Indiana University
Lantao Liu
Last update: 9/10/2019
*/

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <visualization_msgs/Marker.h>
#include <sstream>
#include <random>   // rng for rrt
#include <iterator> //maybe unneeded?
#include<bits/stdc++.h> // graph deffinition 

using namespace std;

int fuckup[] = {0,0,0,0,0,0,0,0};

//Global Controls

const int MAX_PTS = 20000; //# of rrt edges
const double EPSILON = 0.01; // clipping check distance
const double WIDTH = 5.0; //Width of arena
const double GRAPH_LINE_WIDTH = 0.02; // width of lines in RRT graph
const double RANGE = 1.0; // range within which new verticies will auto attempt to connect to stopPoint
const int SEED = 59073451;

// Rand control
std::random_device                  rand_dev;
std::mt19937                        generator(rand_dev());

struct rrtInfo
{
  std::vector<geometry_msgs::Point> pointList;
  visualization_msgs::Marker edgeList;


  // constructor
  /*rrtInfo(std::vector<geometry_msgs::Point> pL, visualization_msgs::Marker eL)
  {
    pointList = pL;
    edgeList = eL;
  }*/
};

rrtInfo rrtResult;

double fRand(double fMin, double fMax)
{
    double f;
    std::uniform_real_distribution<> dist(fMin,fMax);
    f = dist(generator);
    //std::cout<<"fRand| f: "<<f<<"\n"<<fflush;
    return f;
}

bool collisionTest(geometry_msgs::Point *p, visualization_msgs::Marker obj[])
{
  double scaleX, scaleY, objX, objY, distX, distY, dist, hold;
  bool  right, left, top, bottom, collision = 0;
  int type;

  int len = sizeof(obj);
  //std::cout<<"\ncollisionTest| obj array length: "<< len << std::endl;


  double x = p->x;
  double y = p->y;
  double sizeP = 0.2;

  //std::cout<<"\ncollisionTest| Read in Point: p.x, p.y: "<< x << "," << y << "\n"<<fflush;

  for(int i=0; i<len-1; i++)
  {
    type = obj[i].type;
    objX = obj[i].pose.position.x;
    objY = obj[i].pose.position.y;
  //std::cout<<"\ncollisionTest| Read in type objX, obY: "<< objX << "," << objY << "\n"<<fflush;

    // Check for out of bounds width = 10 / 2 = 5
    if ( objX >= WIDTH or objY >= WIDTH ) { return (1); }

    switch(type)
    {
      case 1 : // Cube
      
      scaleX = obj[i].scale.x / 2.0;
      scaleY = obj[i].scale.y / 2.0;
      if( obj[i].pose.orientation.w!=1.0)
      {
        //hold = scaleX;
        scaleX = 2.5;
        scaleY = 0.3;
      }
      //std::cout << "collisionTest| Cube object #: "<<i<<" Volume: "<< volume<< std::endl;

      right = x < objX + scaleX;
      left = x > objX - scaleX;
      top = y < objY + scaleY;
      bottom = y > objY -scaleY;


      //std::cout << "collisionTest| Cube bound Check| object #: "<<i<<" cube center: ("<<objX<<","<<objY<<") top: "<<top<<" bottom: "<<bottom<<" left: "<<left<<" right: "<<right<<"\n"<< fflush;
      if( left and right and top and bottom )
      {
        fuckup[i] += 1;// count which object is fucking up most.
        collision = 1;
        //std::cout << "\n\n\ncollisionTest| Cube collision| object #: "<<i<<" cube center: ("<<objX<<","<<objY<<
        //"), cube scale (x,y): "<< scaleX<<","<<scaleY<<") x: "<<x<<", y: "<< y <<"\n\n\n"<< fflush;
        /*if(x>-1.25)
        {
          std::cout << "\n\n\ncollisionTest| !!THE ZONE!!| object #: "<<i<<" cube center: ("<<objX<<","<<objY<<") cube.scale.(x,y): ("
          <<scaleX<<"."<<scaleY<<") x: "<<x<<", y: "<< y <<"\n\n\n"<<fflush;
        }*/
      }

      break;

      case 3 : // Cylinder
      //std::cout<<"\ncollisionTest| CYL\n";
      // Get scale and position
      scaleX = obj[i].scale.x / 2.0;
      //std::cout << "collisionTest| 0x= "<<x<<"\n"<<fflush;
      distX = pow( abs(x-objX) - sizeP , 2);
      distY = pow( abs(y-objY) - sizeP , 2);
      dist = sqrt(distX+distY);
      //std::cout << "collisionTest| 1x= "<<x<<"\n"<<fflush;
      //std::cout << "collisionTest| object: "<<i<<" distX: "<<distX<<" distY: "<<distY<<") dist:"<<dist<<"\n"
      //<<"     x objX y objY: "<< x <<","<< objX <<","<< y <<","<< objY << "\n" << fflush;


      if( dist<= scaleX)
      {
        collision = 1;
        //std::cout << "collisionTest| CYL collision| object #: "<<i<<" cly center: ("<<objX<<","<<objY<<") radius:"<<scaleX<<"\n"<< fflush;
      }

      break;

      //case 4 : // line_strip
      //std::cout<<"collisionTest| LINE!!!!";
      //break;

      default: std::cerr<<"collisionTest: WARNING: type "<<type<<" unrecognized. Aborting"<<std::endl;
      exit(0);
    }
  }
  //std::cout << "collisionTest| collision: "<< collision << std::endl;
  return(collision);
}


double pointDistance(geometry_msgs::Point a, geometry_msgs::Point b)
{
  double xDiff = pow( a.x - b.x, 2 );
  double yDiff = pow( a.y - b.y, 2 );
  double dist = sqrt( xDiff + yDiff );

  return(dist);
}

void randomValidPoint( geometry_msgs::Point *a, visualization_msgs::Marker obst[])
{
  int tries = 100;
  bool collision;
  a->z = 0.0; //everything occurs in plane.

  for( int i = 0; i <= tries; i++)
  {
    a->x = fRand( -WIDTH, WIDTH);
    a->y = fRand( -WIDTH, WIDTH);
    //std::cout<<"randomValidPoint| Point test at x: "<<a->x<<" y: "<<a->y<<"\n"<<fflush;
    collision = collisionTest(a, obst);
    if(collision==0)
    {
       //std::cout<<"randomValidPoint| Valid Point found x: "<<a->x<<" y: "<<a->y<<"\n";
       return;
    }
    else{}//std::cout << "randomValidPoint| fail# "<< i << std::endl;}
  }
  //std::cerr<<"randomValidPointPosition| Failed to find valid point after "<<tries<<" tries. ABORTING.\n";
  exit(0);
}

int nearestPointIndx(geometry_msgs::Point a, std::vector<geometry_msgs::Point> pointList)
{
  int indx, len = pointList.size();
  //std::cout<<"nearestPointIndx| pointList.size(): "<<len<<"\n"<<fflush;
  std::vector<double> dist(len);
  double test;
  //std::cout<<"nearestPointIndx|a.x,y: "<<a.x<<","<<a.y<<"\n"<<fflush;

  for(int i = 0; i < len; i++)
  {
    dist[i] = pointDistance( a, pointList[i] );
    
  }
  indx = std::distance(dist.begin(), std::min_element(dist.begin(),dist.end() ) );
  //std::cout<<"nearestPointIndx| indx): "<<indx<<"\n"<<fflush;
  return(indx);
}

geometry_msgs::Point clippingCheck(geometry_msgs::Point a, geometry_msgs::Point b,visualization_msgs::Marker obj[] )
{
  geometry_msgs::Point traj; // trajectory from a to b
  geometry_msgs::Point origin, check; // for math, growing vector
  double length, scale; // length of traj, scales of x y
  bool collision;
  int steps;

  origin.x = 0.0;
  origin.y = 0.0;
  origin.z = 0.0;
  check.x = a.x;
  check.y = a.y;
  check.z = 0.0;

  traj.x = b.x - a.x;
  traj.y = b.y - a.y;
  traj.z = 0.0; //in plane
  length =pointDistance(origin, traj);
  //std::cout<<"clippingCheck| length: "<< length<<"\n"<<fflush;

   
  steps = length / EPSILON;

  for(int i=0; i<steps; i++)
  {
    scale = ((double)i / (double)steps);
    // check intersection
    collision = collisionTest(&check, obj);
    if( collision ==1 )
    {
      std::cout << "clippingCheck| edge intersects. Returning intermediatept at ("<<check.x<<","<<check.y<<")\n" <<fflush;
      return(check);
    }
    // grow the check vector
    check.x =  scale * traj.x + a.x;
    check.y =  scale * traj.y + a.y;
    //std::cout << "clippingCheck| step: "<<i<<" check at ("<<check.x<<","<<check.y<<")\n" <<fflush;
  }
  //std::cout << "clippingCheck| VALID EDGE. Returning pt at ("<<check.x<<","<<check.y<<")\n" <<fflush;
  return(check);

}

void rrtBuild(geometry_msgs::Point startPoint, geometry_msgs::Point stopPoint, visualization_msgs::Marker obst[], ros::Publisher draw_pub, double epsilon)// maxpts to use in search, epsilon smallest clipping distance
{

  std::cout<<"+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n\n\n+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n"<<fflush;
  geometry_msgs::Point qRandom, qNear, qNew, stopCheck; // points for feeling out and growing graph
  std::vector<geometry_msgs::Point> pointList;// pointList for rapidly finding Nearby
  visualization_msgs::Marker edgeList; // store graph as edgelist, represent data type as list of lines: 1a,1b,2a,2b,...
  int indx;
  edgeList.type = visualization_msgs::Marker::LINE_LIST;
  edgeList.scale.x = GRAPH_LINE_WIDTH; // line width is set by x dimension only
  edgeList.color.g = 1.0; // Graph is green
  edgeList.color.a = 1.0; // This is probably making it opaque

  //edgeList.points.push_back(startPoint); // add startpoint;
  pointList.push_back(startPoint);


  for(int i=0; i< MAX_PTS; i++)
  {
    qRandom.x = 1.0;
    qRandom.y = 1.0;
    qRandom.z = 0.0;
    randomValidPoint(&qRandom, obst); // pick a point not in or clipping an obsticle
    indx = nearestPointIndx(qRandom, pointList);
    qNear = pointList[indx];
    qNew = clippingCheck(qNear, qRandom, obst); // walks along qRandom - qNear vector and clips it if path intersects object
    
    
    // add line pair to line_list and point to point list
    edgeList.points.push_back( qNear );
    edgeList.points.push_back( qNew );
    //std::cout<<"rrtBuild| qNear(" << qNear.x <<","<<qNear.y<<") paired with qNew("<<qNew.x<<","<<qNew.y<<")\n"<<fflush;
    pointList.push_back( qNew );
    //std::cout<<"rrtBuild| qNew placed at: ("<<qNew.x<<","<<qNew.y<<")\n"<<fflush;

    // If qNew is in range of stopPoint, attempt to connect
    if( pointDistance(qNew,stopPoint) <= RANGE )
    {
      stopCheck = clippingCheck(qNew,stopPoint,obst);
      if( pointDistance(qNew, stopCheck) <= EPSILON) //stopCheck.x == qNew.x and stopCheck.y == qNew.y )
      {
        edgeList.points.push_back(qNew);
        edgeList.points.push_back(stopPoint);
        std::cout<<"rrtBuild| stopPoint connection made\n TERMINATING TREE GROWTH\n\n"<<fflush;
        goto stop;
      }
    }
  }
  stop: //from goto stop; connection made to stoppoint.

  // add stop point as last member of pointList
  pointList.push_back(stopPoint);
    //std::cout<<"rrtBuild| pointLIst: "<<edgeList<<"\n"<<fflush;
    edgeList.header.frame_id = "map"; //NOTE: this should be "paired" to the frame_id entry in Rviz
    edgeList.header.stamp = ros::Time::now();

    // Set the namespace and id
    edgeList.ns = "graph";
    edgeList.id = 99;

    // Set the marker action
    edgeList.action = visualization_msgs::Marker::ADD;


    edgeList.lifetime = ros::Duration();

  //std::cout << "rrtBuild| edgelist: "<<edgeList<<"\n"<< fflush;
  draw_pub.publish(edgeList);

  //planning_pub.publish( rrtResult );
  rrtResult.pointList = pointList;
  rrtResult.edgeList = edgeList;

  for( int i =0; i<8; i++)
  {
  std::cout<<"rrtBuild| fuckup["<<i<<"]: "<<fuckup[i]<<std::endl;
  }

}


void dijkstra()
{
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_demo");

  //create a ros handle (pointer) so that you can call and use it
  ros::NodeHandle n;

  //in <>, it specified the type of the message to be published
  //in (), first param: topic name; second param: size of queued messages, at least 1
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("some_chatter", 10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

  //each second, ros "spins" and draws 20 frames
  ros::Rate loop_rate(20);

  int frame_count = 0;
  float f = 0.0;

  int width = 10;

  while (ros::ok())
  {
    //first create a string typed (std_msgs) message
    std_msgs::String msg;

    std::stringstream ss;
    ss << "Frame index: " << frame_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str()); //printing on screen

    //publisher publishes messages, the msg type must be consistent with definition advertise<>();
    chatter_pub.publish(msg);



  /******************** From here, we are defining and drawing obstacles and environment in the workspace **************************/

    visualization_msgs::Marker obstBoundary;
    obstBoundary.type = visualization_msgs::Marker::LINE_STRIP; //Set obstBoundary as a line strip
    // Set the frame id and timestamp
    obstBoundary.header.frame_id = "map"; //NOTE: this should be "paired" to the frame_id entry in Rviz
    obstBoundary.header.stamp = ros::Time::now();

    // Set the namespace and id
    obstBoundary.ns = "Boundary";
    obstBoundary.id = 0;

    // Set the marker action
    obstBoundary.action = visualization_msgs::Marker::ADD;

    //Set the scale of the marker. For line strip there is only x width available
    obstBoundary.scale.x = 0.2;

    // Set the color and transparency for the marker. If not set default values are 0 for all the fields.
    obstBoundary.color.r = 1.0f;
    obstBoundary.color.g = 1.0f;
    obstBoundary.color.b = 0.0f;
    obstBoundary.color.a = 1.0; //be sure to set alpha to something non-zero, otherwise it is transparent

    obstBoundary.lifetime = ros::Duration();


    geometry_msgs::Point p;

    p.x = -width/2;
    p.y = -width/2;
    p.z = 0;

    obstBoundary.points.push_back(p);
    p.x = width/2;
    obstBoundary.points.push_back(p);
    p.y = width/2;
    obstBoundary.points.push_back(p);
    p.x = -width/2;
    obstBoundary.points.push_back(p);
    p.y = -width/2;
    obstBoundary.points.push_back(p);

    // Publish this to ROS system
    marker_pub.publish(obstBoundary);

    //Create array of Visualization markers
    visualization_msgs::Marker obst [7];

    //draw cube/rectangular type obstacles
    for (int i = 0; i<4; i++) {
      obst[i].type = visualization_msgs::Marker::CUBE;
      obst[i].header.frame_id = "map";
      obst[i].header.stamp = ros::Time::now();
      obst[i].ns = "obstacles";
      obst[i].id = i;
      obst[i].action = visualization_msgs::Marker::ADD;
      obst[i].color = obstBoundary.color;

      //Set obstacle scale
      obst[i].scale.x = obst[i].scale.z = 0.5;
      obst[i].scale.y = 8;
      obst[i].pose.position.z = 0.5;
      obst[i].pose.orientation.x = obst[i].pose.orientation.y = obst[i].pose.orientation.z = 0;
      obst[i].pose.orientation.w = 1;
      switch (i) {
        case 0 : obst[i].pose.position.x = -3;
                 obst[i].pose.position.y = -1;
                 break;
        case 1 : obst[i].pose.position.x = -1.5;
                 obst[i].pose.position.y = 2;
                 obst[i].scale.y = 5;
                 obst[i].pose.orientation.w = obst[i].pose.orientation.z = 0.7071;
                 break;
        case 2 : obst[i].pose.position.x = 3;
                 obst[i].pose.position.y = 1;
                 break;
        case 3 : obst[i].pose.position.x = 1.5;
                 obst[i].pose.position.y = -2;
                 obst[i].scale.y = 5;
                 obst[i].pose.orientation.w = obst[i].pose.orientation.z = 0.7071;
                 break;
      }

      obst[i].lifetime = ros::Duration();
      marker_pub.publish(obst[i]);
    }

    //draw cylinder type obstacles
    for (int i = 4; i<7; i++) {
      obst[i].type = visualization_msgs::Marker::CYLINDER;
      obst[i].header.frame_id = "map";
      obst[i].header.stamp = ros::Time::now();
      obst[i].ns = "obstacles";
      obst[i].id = i;
      obst[i].action = visualization_msgs::Marker::ADD;
      obst[i].color = obstBoundary.color;
      obst[i].scale.x = obst[i].scale.y = obst[i].scale.z = 2;
      obst[i].pose.position.x = 0;
      obst[i].pose.position.z = 1;

      switch (i) {
        case 4 : obst[i].pose.position.y = 4;
                 break;
        case 5 : obst[i].pose.position.y = 0;
                 break;
        case 6 : obst[i].pose.position.y = -4;
                 break;
      }
      obst[i].lifetime = ros::Duration();
      marker_pub.publish(obst[i]);
    }


    /******************** Defining start point and goal point *******************/

    visualization_msgs::Marker GoalPoint;
    GoalPoint.type = visualization_msgs::Marker::POINTS;

    GoalPoint.header.frame_id = "map";
    GoalPoint.header.stamp =  ros::Time::now();
    GoalPoint.ns =  "Goal Points";
    GoalPoint.action =  visualization_msgs::Marker::ADD;
    GoalPoint.pose.orientation.w = 1.0;

    GoalPoint.id = 0;

    GoalPoint.scale.x = 0.25;
    GoalPoint.scale.y = 0.25;

    GoalPoint.color.g = 1.0f;
    GoalPoint.color.a = 1.0;

    geometry_msgs::Point point;	// root vertex
    point.z = 0;
    point.x = -4;
    point.y = -4;

    GoalPoint.points.push_back(point);

    point.x = 4;
    point.y = 4;
    GoalPoint.points.push_back(point);
    GoalPoint.lifetime = ros::Duration();
    marker_pub.publish(GoalPoint);

  /************************* From here, we are using points, lines  *** ******************/

    //we use static type here since we want to incrementally add contents in these msgs, otherwise contents in these msgs will be cleaned in every ros spin.
    static visualization_msgs::Marker vertices, edges;

    //points
    vertices.type = visualization_msgs::Marker::POINTS;
    edges.type = visualization_msgs::Marker::LINE_LIST;

    vertices.header.frame_id = edges.header.frame_id = "map";
    vertices.header.stamp = edges.header.stamp = ros::Time::now();
    vertices.ns = edges.ns = "vertices_and_lines";
    vertices.action = edges.action = visualization_msgs::Marker::ADD;
    vertices.pose.orientation.w = edges.pose.orientation.w = 1.0;

    vertices.id = 0;
    edges.id = 1;

    // POINTS markers use x and y scale for width/height respectively
    vertices.scale.x = 0.05;
    vertices.scale.y = 0.05;

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    edges.scale.x = 0.02; //tune it yourself

    // Points are green
    vertices.color.g = 1.0f;
    vertices.color.a = 1.0;

    // Line list is red
    edges.color.r = 1.0;
    edges.color.a = 1.0;

    geometry_msgs::Point p0;	// root vertex
    p0.x = p0.y = 0;
    p0.z = 2;
    int num_slice = 20;		// e.g., to create 20 edges
    float length = 1;		//length of each edge

    static int slice_index = 0;
    int herz = 10;		//every 10 ROS frames we draw a line segment
    if(frame_count % herz == 0 && edges.points.size()<= 2*num_slice)
    {
      geometry_msgs::Point p;

      float angle = slice_index*2*M_PI/num_slice;
      slice_index ++ ;
      p.x = length * cos(angle);
      p.y = length * sin(angle);
      p.z = 2;

      vertices.points.push_back(p);	//for drawing vertices
      edges.points.push_back(p0);	//for drawing edges. The line list needs two points for each line
      edges.points.push_back(p);
    }

    //publish msgs
    marker_pub.publish(vertices);
    marker_pub.publish(edges);


  /******************** From here, we are drawing/animating a simple mobile object **************************/

    // a simple sphere
    static visualization_msgs::Marker rob;
    static visualization_msgs::Marker path;
    rob.type = visualization_msgs::Marker::SPHERE;
    path.type = visualization_msgs::Marker::LINE_STRIP;

    rob.header.frame_id = path.header.frame_id = "map";  //NOTE: this should be "paired" to the frame_id entry in Rviz, the default setting in Rviz is "map"
    rob.header.stamp = path.header.stamp = ros::Time::now();
    rob.ns = path.ns = "rob";
    rob.id = 0;
    path.id = 1;
    rob.action = path.action = visualization_msgs::Marker::ADD;
    rob.lifetime = path.lifetime = ros::Duration();

    rob.scale.x = rob.scale.y = rob.scale.z = 0.3;

    rob.color.r = 1.0f;
    rob.color.g = 0.5f;
    rob.color.b = 0.5f;
    rob.color.a = 1.0;

    // path line strip is blue
    path.color.b = 1.0;
    path.color.a = 1.0;

    path.scale.x = 0.02;
    path.pose.orientation.w = 1.0;

    int num_slice2 = 200;		// divide a circle into segments
    static int slice_index2 = 0;
    if(frame_count % 2 == 0 && path.points.size() <= num_slice2)  //update every 2 ROS frames
    {
      geometry_msgs::Point p;

      float angle = slice_index2*2*M_PI/num_slice2;
      slice_index2 ++ ;
      p.x = 4 * cos(angle) - 0.5;  	//some random circular trajectory, with radius 4, and offset (-0.5, 1, .05)
      p.y = 4 * sin(angle) + 1.0;
      p.z = 0.05;

      rob.pose.position = p;
      path.points.push_back(p);		//for drawing path, which is line strip type
    }

    marker_pub.publish(rob);
    marker_pub.publish(path);

 
  /******************** TODO: you will need to insert your code for drawing your paths and add whatever cool searching process **************************/
  
  static bool flag = 0;
  if( flag == 0)
  {
    // init publisher
    //ros::Publisher planning_pub = n.advertise<rrtInfo>("path_planning", 10);
    // init subscriber
    //ros::Subscriber planning_sub = n.subscribe("path_planning", 10, planningCallback); //<rrtInfo>("path_planning", 10, planningCallback);
    // build a tree
    rrtBuild(GoalPoint.points[0], GoalPoint.points[1], obst, marker_pub, 0.1 ); //startPoint, obst[], int maxPts, double epsilon
    // Dijkstra shortest path from start to fin.
    dijkstra();
    // Code here
    // Pop to start point
    flag =1;
  }

  // Follow Path 
  // code here
  /******************** To here, we finished displaying our components **************************/

    // check if there is a subscriber. Here our subscriber will be Rviz
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please run Rviz in another terminal.");
      sleep(1);
    }


    //ros spins, force ROS frame to refresh/update once
    ros::spinOnce();

    //leave some time interval between two frame refreshing
    loop_rate.sleep();

    ++frame_count;
  }

  return 0;
}
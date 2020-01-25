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
#include <random>
#include <iterator>

//Global Controls

const double WIDTH = 10;
const int SEED = 59073451;

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

bool collisionTest(geometry_msgs::Point p, visualization_msgs::Marker obj[])
{
  double scaleX, scaleY, objX, objY, distX, distY, dist;
  bool  right, left, top, bottom, collision = 0;
  int type;

  int len =  7; // V. bad form but sizeof(a)/sizeof(a[0]) fails

  double x = p.x;
  double y = p.y;
  double sizeP = 0;

  for(int i=0; i<len; i++)
  {
    type = obj[i].type;
    objX = obj[i].pose.position.x;
    objY = obj[i].pose.position.y;

    // Check for out of bounds width = 10
    if ( objX >= WIDTH or objY >= WIDTH ) { return (1); }

    switch(type)
    {
      case 1 : // Cube
      std::cout<<"\ncollisionTest| CUBE\n";
      scaleX = obj[i].scale.x / 2.0;
      scaleY = obj[i].scale.y / 2.0;

      
      // detect if within boundaries
      left = x>= objX - scaleX - sizeP;
      right = x<= objX + scaleX + sizeP;
      bottom = x >=objY - scaleY - sizeP;
      top = x<= objY - scaleY -sizeP;

      if( left and right and top and bottom )
      {
        collision = 1;
        std::cout << "collisionTest| Cube collision \n" << fflush;
      }

      break;

      case 3 : // Cylinder
      std::cout<<"\ncollisionTest| CYL\n";
      scaleX = obj[i].scale.x / 2.0;
      distX = pow( abs(x-objX) - sizeP , 2);
      distY = pow( abs(y-objY) - sizeP , 2);
      dist = sqrt(distX+distY);

      if( dist<= scaleX)
      {
        collision = 1;
        std::cout << "collisionTest| Cyl collision \n" << fflush;
      }

      break;

      //case 4 : // line_strip
      //std::cout<<"collisionTest| LINE!!!!";
      //break;

      default: std::cerr<<"collisionTest: WARNING: type "<<type<<" unrecognized. Aborting"<<std::endl;
      exit(0);
    }
  }
  std::cout << "collisionTest| collision: "<< collision << std::endl;
  return(collision);
}


double pointDistance(geometry_msgs::Point a, geometry_msgs::Point b)
{
  double xDiff = pow( a.x - b.x, 2 );
  double yDiff = pow( a.y - b.y, 2 );
  double dist = sqrt( xDiff + yDiff );

  return(dist);
}

void randomValidPoint( geometry_msgs::Point a, visualization_msgs::Marker obst[])
{
  std::cout<<"randomValidPoint| HIT\n"<<fflush;
  int tries = 100;
  double x,y;
  bool collision;

  for( int i = 0; i <= tries; i++)
  {
    x = fRand( -WIDTH, WIDTH);
    y = fRand( -WIDTH, WIDTH);
    std::cout<<"randomValidPoint| Point test at x: "<<x<<" y: "<<y<<"\n"<<fflush;
    collision = collisionTest(a, obst);
    if(collision==0)
    {
       a.x = x;
       a.y = y;
       std::cout<<"randomValidPoint| Valid Point found x: "<<x<<" y: "<<y<<"\n";
       return;
    }
    else{std::cout << "randomValidPoint| fail# "<< i << std::endl;}
  }
  std::cerr<<"randomValidPointPosition| Failed to find valid point after "<<tries<<" tries. ABORTING.\n";
  exit(0);
}

void RRT(geometry_msgs::Point startPoint, visualization_msgs::Marker obst[], int maxPts, double epsilon)// maxpts to use in search, epsilon smallest clipping distance
{
  geometry_msgs::Point qRandom, qNear, qNew;

  for(int i=0; i< maxPts; i++)
  {
    randomValidPoint(qRandom, obst);
    std::cout<<"qRandom.x: "<<qRandom.x
    <<" qRandom.y: "<<qRandom.y<<std::endl;
  }

}

int main(int argc, char **argv)
{

  // start random
  srand(SEED);

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

  RRT(GoalPoint.points[0], obst, 10, 0.1 ); //startPoint, obst[], int maxPts, double epsilon

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
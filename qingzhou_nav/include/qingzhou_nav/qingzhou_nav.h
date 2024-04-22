#include <ros/ros.h>
#include <signal.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <string.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include <qingzhou_cloud/current_location.h>
#include <qingzhou_cloud/stoppoint.h>
#include <qingzhou_cloud/trafficlight.h>
#include <qingzhou_cloud/roadLine.h>
#include "std_msgs/String.h"
#include <std_msgs/Float32.h>
#include <std_msgs/UInt8.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include <tf2/convert.h>
#include <tf2/utils.h>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class qingzhou_nav
{
public:
  qingzhou_nav(ros::NodeHandle nh);
  ~qingzhou_nav();

  void run();

  ros::Publisher pub_qingzhou_location;
  ros::Publisher pub_roadLine_angle;
  ros::Publisher pub_car_status;
  ros::Publisher pub_nav_status;

  ros::Subscriber sub_stopPoint;
  ros::Subscriber sub_roadlinePoints;
  ros::Subscriber sub_trafficlight;

  geometry_msgs::PoseStamped current_pose;              //current pose form feedback
  geometry_msgs::PoseStamped currentstoppoint;          //current stop point to goal
  
  bool Callback_StopPoint_flag;
  std_msgs::UInt8 car_status;                           //car state: 0.init; 1.reach stoppoint; 2.auto navigation; 3.Boot wait
  std_msgs::UInt8 nav_status;                           //nav state: 0.init; 1.traffic light; 2.lifting poker; 3.roadLine; 4.auto navigation

  qingzhou_cloud::current_location currentlocation;
  qingzhou_cloud::stoppoint stoppointMsg;               //stop point x y
  qingzhou_cloud::roadLine sRoadLinePointMsg;
  qingzhou_cloud::trafficlight currenttrafficlight; 

  float sLineControl();
  float foundNearestY( float aimDis);
  float lineDirection(float xsecond, float ysecond);
  double getYaw(geometry_msgs::PoseStamped pose);
  double getDis(geometry_msgs::PoseStamped currentpose,geometry_msgs::PoseStamped roadpose);
  int detectReachedGoal(const geometry_msgs::PoseStamped& currentPose,geometry_msgs::PoseStamped& stopPose);

  void doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result);
  void activeCb();
  void feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

  void callback_stopPoint(const qingzhou_cloud::stoppoint::ConstPtr &msg);
  void callback_roadlinePoints(const qingzhou_cloud::roadLine::ConstPtr &msg);
  void callback_trafficlight(const qingzhou_cloud::trafficlight::ConstPtr &msg);

};















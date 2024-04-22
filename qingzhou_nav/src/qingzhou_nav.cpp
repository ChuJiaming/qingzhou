#include <qingzhou_nav/qingzhou_nav.h>

qingzhou_nav::qingzhou_nav(ros::NodeHandle nh)
{

  stoppointMsg.startCommand = 2;
  car_status.data = 3;
  nav_status.data = 0;
  sRoadLinePointMsg.lineStatus = 0;

  sub_stopPoint = nh.subscribe("stoppoint",1,&qingzhou_nav::callback_stopPoint,this);
  sub_roadlinePoints = nh.subscribe("roadLine",1,&qingzhou_nav::callback_roadlinePoints,this);
  sub_trafficlight = nh.subscribe("trafficLight",1,&qingzhou_nav::callback_trafficlight,this);

  pub_qingzhou_location = nh.advertise<qingzhou_cloud::current_location>("qingzhou_location", 10 );
  pub_roadLine_angle = nh.advertise<std_msgs::Float32>("roadLine_angle", 5 );
  pub_car_status = nh.advertise<std_msgs::UInt8>( "carstatus", 10 );
  pub_nav_status = nh.advertise<std_msgs::UInt8>( "navstatus", 10 );
}

//get stop point
void qingzhou_nav::callback_stopPoint(const qingzhou_cloud::stoppoint::ConstPtr &msg)
{
  stoppointMsg = *msg;
  stoppointMsg.startCommand = 1;

	currentstoppoint.pose.position.x = stoppointMsg.X;
	currentstoppoint.pose.position.y = stoppointMsg.Y;
	currentstoppoint.pose.position.z = 0;
	currentstoppoint.pose.orientation.x = 0;
	currentstoppoint.pose.orientation.y = 0;
	currentstoppoint.pose.orientation.z = 0;
	currentstoppoint.pose.orientation.w = 1;

  Callback_StopPoint_flag = true; //run function use
}

void qingzhou_nav::callback_trafficlight(const qingzhou_cloud::trafficlight::ConstPtr &msg){    
    currenttrafficlight = *msg;
    //printf("actuator-->trafficlight distanceX = %.2f,distanceY = %.2f,status = %d\n",currenttrafficlight.X,currenttrafficlight.Y,currenttrafficlight.trafficstatus);
}

void qingzhou_nav::callback_roadlinePoints(const qingzhou_cloud::roadLine::ConstPtr &msg)
{
  sRoadLinePointMsg = *msg;
	//printf("actuator--> callback_roadlinePoints,lineStatus = %d\n",sRoadLinePointMsg.lineStatus);
}

qingzhou_nav::~qingzhou_nav()
{

}

void qingzhou_nav::run()
{
  ros::Rate loop_rate(20);
  MoveBaseClient ac("move_base", true);

  //Wait 30 seconds for the action server to become available
  if (!ac.waitForServer(ros::Duration(30)))
  {
     ROS_INFO("Can't connected to move base server");
  }
  ROS_INFO("Connected to move base server");
  
  pub_car_status.publish(car_status);
  pub_nav_status.publish(nav_status);

	while (ros::ok())
	{
    ros::spinOnce();
    if( currenttrafficlight.trafficstatus == 2)
    {
      nav_status.data = 1;
      pub_nav_status.publish(nav_status);
    }

    if(sRoadLinePointMsg.lineStatus == 0x01)
    {
	    //printf("success sroad-->linestatus = %d\n",sRoadLinePointMsg.lineStatus);
      float tagetAngle = sLineControl();
	    std_msgs::Float32 currentAngle;
	    currentAngle.data = tagetAngle;
	    pub_roadLine_angle.publish(currentAngle);

      nav_status.data = 3;
      pub_nav_status.publish(nav_status);
      continue;
    }
    else
    {
      /*if(stoppointMsg.startCommand != 1)
      {
        ROS_INFO("please pub stop point:");

        continue;
      }*/
      if(Callback_StopPoint_flag == true)
      {
        Callback_StopPoint_flag = false;

        car_status.data = 2;
        pub_car_status.publish(car_status);    
        nav_status.data = 4;
        pub_nav_status.publish(nav_status);

        move_base_msgs::MoveBaseGoal goal;
	      goal.target_pose.header.frame_id = "map";
	      goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = 	currentstoppoint.pose.position.x; 
        goal.target_pose.pose.position.y = 	currentstoppoint.pose.position.y;
        goal.target_pose.pose.position.z = 	currentstoppoint.pose.position.z; 
        goal.target_pose.pose.orientation.x = currentstoppoint.pose.orientation.x;
        goal.target_pose.pose.orientation.y = currentstoppoint.pose.orientation.y;      
        goal.target_pose.pose.orientation.z = currentstoppoint.pose.orientation.z;
        goal.target_pose.pose.orientation.w = currentstoppoint.pose.orientation.w;
       
        ac.sendGoal(goal,
                    boost::bind(&qingzhou_nav::doneCb, this, _1, _2),
                    boost::bind(&qingzhou_nav::activeCb, this),
                    boost::bind(&qingzhou_nav::feedbackCb, this, _1));             
      }     
    }
    loop_rate.sleep();  
	}
}

void qingzhou_nav::doneCb(const actionlib::SimpleClientGoalState& state,const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());
  car_status.data = 1;
  pub_car_status.publish(car_status);  
}

// Called once when the goal becomes active
void qingzhou_nav::activeCb()
{
	ROS_INFO("Goal just went aceive");
}

// Called every time feedback is received for the goal
void qingzhou_nav::feedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{      
	//ROS_INFO("Got base_position of Feedback");
  current_pose = feedback->base_position;

  currentlocation.x = feedback->base_position.pose.position.x;
	currentlocation.y = feedback->base_position.pose.position.y;
  double currentpose_yaw = tf2::getYaw(feedback->base_position.pose.orientation);
	currentlocation.heading = currentpose_yaw;
  pub_qingzhou_location.publish(currentlocation);       
}

double qingzhou_nav::getYaw(geometry_msgs::PoseStamped pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

//compute dintance
double qingzhou_nav::getDis(geometry_msgs::PoseStamped currentpose,geometry_msgs::PoseStamped roadpose)
{
  return sqrt((currentpose.pose.position.x - roadpose.pose.position.x) * (currentpose.pose.position.x - roadpose.pose.position.x)
  + (currentpose.pose.position.y - roadpose.pose.position.y) * (currentpose.pose.position.y - roadpose.pose.position.y));
}

int qingzhou_nav::detectReachedGoal(const geometry_msgs::PoseStamped& currentPose,geometry_msgs::PoseStamped& stopPose)
{
  double diffDistance = getDis(currentPose,stopPose);
  if(diffDistance < 0.3)
  {
    return 1;
  }
  else return 0;   
  }

float qingzhou_nav::foundNearestY( float aimDis)
  {
    float minX = 8888;
    float NearestY;
    for(size_t i = 0;i < (int)sRoadLinePointMsg.lineX.size();i++)
    {
      float tempfabsdis = (float)fabs(sRoadLinePointMsg.lineX[i] - aimDis);
      if(tempfabsdis < minX)
      {
        minX = tempfabsdis;
        NearestY = sRoadLinePointMsg.lineY[i];
      }
    }
    return NearestY;
  }

float qingzhou_nav::lineDirection(float xsecond, float ysecond)
{
  float alpha = 0.0;
  xsecond =+ 300;
  if ((xsecond>0)&&(ysecond>0))
  {
    alpha=atan(fabs(ysecond-0)/fabs(xsecond-0));
  }
  else if ((xsecond>0)&&(ysecond<=0))
  {
    alpha=2*M_PI-atan(fabs(ysecond-0)/fabs(xsecond-0));
  }

    return alpha;
}

float qingzhou_nav::sLineControl(){
  float NearestY = foundNearestY( 450);
  //NearestY /= 100; 
  float alpha = lineDirection(450, NearestY);
  float targetAngle = atan(2.0 * 0.31 * sin(alpha) / 0.75)*57.3;
  printf("nearestY = %.2f,alpha = %.2f,angle = %.2f\n",NearestY,alpha*57.3,targetAngle);
	return targetAngle;
      
}

#include "app.h"
#include <tf2/convert.h>
#include <tf2/utils.h>

app::app(ros::NodeHandle nh) {
     //variable init
     cloudsendrate = 5;
     recvflag = 0;
     disconnectFlag = 0;
     mfcCloudPortNum = 8050;
     heartFlag = 0;
     heartdisconnectCommand = 0;
     recvAckFlag = 0;
     clientIsConnect = 0;
     carstatusMsg.data = 0;

     nh.param("cloudsendrate",cloudsendrate,cloudsendrate);
     nh.param("mfcCloudIP",mfcCloudIP,mfcCloudIP);
     nh.param("mfcCloudPortNum",mfcCloudPortNum,mfcCloudPortNum);
     nh.param("bdebug",bdebug,bdebug);

    pub_app_command = nh.advertise<qingzhou_cloud::qingzhou_cloud>("clientCommand", 5);
    pub_start_stop_command = nh.advertise<qingzhou_cloud::startstopCommand>("startStopCommand", 5);
    pub_stop_point = nh.advertise<qingzhou_cloud::stoppoint>("stoppoint", 5);
    //pub_trafficLight = nh.advertise<dzcloud::trafficLight>("trafficLight", 5);

    clientdetectConnectThread = boost::thread(boost::bind(&app::detectConnectThread, this));
    clientRecvThread = boost::thread(boost::bind(&app::RecvThreadFromMfc, this));
    //sendHeart = boost::thread(boost::bind(&app::sendHeartThread, this));
    sub_current_position = nh.subscribe("qingzhou_location",1,&app::callback_location,this);
    sub_current_battery = nh.subscribe("battery",1,&app::callback_battery,this);
    sub_car_status = nh.subscribe("carstatus",1,&app::callback_carstatus,this);
    sub_nav_status = nh.subscribe("navstatus",1,&app::callback_navstatus,this);     
    sub_car_speed = nh.subscribe("odom",1,&app::callback_speed,this);

 }

  double getYaw(geometry_msgs::PoseStamped pose)
  {
      return tf2::getYaw(pose.pose.orientation);
  }

 app::~app()
 {
 }
void app::callback_carstatus(const std_msgs::UInt8::ConstPtr &msg){
    carstatusMsg = *msg;
 	//printf("qingzhou_cloud-->callback_carstatus:%d\n",carstatusMsg.data);
}
void app::callback_navstatus(const std_msgs::UInt8::ConstPtr &msg){
    navstatusMsg = *msg;
// 	printf("qingzhou_cloud-->callback_navstatus\n");
}
void app::callback_location(const qingzhou_cloud::current_location::ConstPtr &msg){
    current_location = *msg;
 	//printf("qingzhou_cloud-->callback_location\n");
}
  
void app::callback_battery(const std_msgs::Float32::ConstPtr &msg){
    current_battery = *msg;
    //printf("qingzhou_cloud-->callback_battery = %.2f\n",current_battery.data);
}
  
void app::callback_speed(const nav_msgs::Odometry::ConstPtr &msg){
    sOdom = *msg;
    //printf("qingzhou_cloud-->callback_speed = %.2f, %.2f\n",sOdom.twist.twist.angular.z,sOdom.twist.twist.linear.x);
}
void app::run()
{
    ros::Rate rate(cloudsendrate);
    while(ros::ok()){
    ros::spinOnce();
	if(clientIsConnect == 1){
	    dataProcKernelNet(0x01);
	}
//	dzcloud::trafficLight lightMsg;
//	pub_trafficLight.publish(lightMsg);

    rate.sleep();
    }
 }

void app::dataProcKernelNet(int carID){
    float x = current_location.x;
    float y = current_location.y;
    float theta = current_location.heading;
    float currentAngleSpeed = sOdom.twist.twist.angular.z;
    float currentLinerSpeed = sOdom.twist.twist.linear.x;
    char navstatus = 0;  
    char carstatus = carstatusMsg.data;   
 	//printf("qingzhou_cloud--> x:%.2f,y:%.2f,AngleSpeed:%.2f,LinerSpeed:%.2f,navstatus:%d,status:%d\n",x,y,currentAngleSpeed,currentLinerSpeed,navstatus,status);

    //char send_buf[126] = {0,};
    //send_frame_t* send_frame = (send_frame_t*)send_buf;
    //send_frame->head[0] = 0x02;
    //send_frame->head[1] = 0x20;
    //send_frame->head[2] = 0x02;
    //send_frame->head[3] = 0x20;
    //send_frame->len = 31;
    //send_frame->command = 0xaa;
    //memcpy(send_buf+9,&carID,4);
    //memcpy(send_buf+13,&x,4);
    //memcpy(send_buf+17,&y,4);
    //memcpy(send_buf+21,&theta,4);
    //memcpy(send_buf+25,&battery,4);
    struct info{
	int carID1;float x1;float y1;float theta1;float angleSpeed;float linerSpeed;char navstatus1;char carstatus1;
	};
	
    struct info info1{
        carID,
        x,
        y,
        theta,
        currentAngleSpeed,
	    currentLinerSpeed,
	    navstatus,
	    carstatus,
    };

    //send_buf[29] = 0x00;
    //send_buf[30] = 0x00;
    //send_buf[31] = 0x00;
    //send_buf[32] = 0x01;
    //send_buf[33] = 0x00;
    //send_buf[34] = 0x02;
    
    //send_buf[35] = 0x03;
    //send_buf[36] = 0x30;
    //send_buf[37] = 0x03;
    //send_buf[38] = 0x30;
    //int ret = send(clientfd, send_buf, send_frame->len + 8, 0);
    int ret = send(clientfd, (char *)&info1, sizeof(info1), 0);
    if(ret < 0)
	printf("qingzhou_cloud-->send to MFC server error %d\n",errno);
}



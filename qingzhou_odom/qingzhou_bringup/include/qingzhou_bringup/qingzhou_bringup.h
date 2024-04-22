#ifndef NODE_EXAMPLE_TALKER_H
#define NODE_EXAMPLE_TALKER_H

// ROS includes.
#include "ros/ros.h"
#include "ros/time.h"

#include <iostream>
#include <math.h>
#include <string.h>
#include <boost/asio.hpp>
#include <boost/bind.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Range.h>

#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <geometry_msgs/Point32.h>
#include <std_msgs/Float32.h>

//lib
#include <serial/serial.h>
#include <qingzhou_cloud/startstopCommand.h>
#include <qingzhou_cloud/trafficlight.h>
#include <qingzhou_cloud/roadLine.h>
#include <time.h>

#define CARL 0.31
#define CARW 0.325
#define PI 3.14159265
using namespace std;
using namespace boost::asio; 
using namespace boost;

typedef struct sMartcarControl{
  int TargetAngleDir;     //期望转向角度符号 0:直行；0x10:左转；0x20:右转；
  int TargetAngle;        //期望角度
  int TargetSpeed;        //期望速度
  int TargetModeSelect;   //模式选择
  int TargetShiftPosition;
  bool control;
}sMartcarControl;        

 
class actuator
{
public:
  //! Constructor.
  actuator(ros::NodeHandle nh);
  ~actuator();
  
  void run();
public:
     int m_baudrate;            
     int m_deviceName;         
     int m_runningmode;         //运行模式
     int bdebug;
     int cartype;
     std::string updatelog;
     std::string m_serialport;  //对应USB端口
     
     int encoderLeft;           
     int encoderRight;          
     int calibrate_lineSpeed;   //标定线速度
     int calibrate_angularSpeed;//标定角速度
     float ticksPerMeter;       
     float ticksPer2PI;         
     float linearSpeed;         //线速度
     float angularSpeed;        //角速度
     float batteryVoltage;      //电池电压
       
     short tempaccelX,tempaccelY,tempaccelZ;     //加速度缓存区
     short tempgyroX,tempgyroY,tempgyroZ;        //角速度缓存区
     short tempmagX,tempmagY,tempmagZ;           //磁力计缓存区
     double accelX,accelY,accelZ;                 
     double gyroX,gyroY,gyroZ;                    
     double magX,magY,magZ;                      
     int distanceA,distanceB,distanceC,distanceD;//超声波距离值
     float imuYaw;                               
	   double roll, pitch, yaw;                
     double velDeltaTime;                        //时间，存放转换成秒的时间
     double detdistance,detth;                   //计算距离和计算角度角度
     double detEncode;                           
     sMartcarControl carParasControl;            //根据之前定义的结构体，声明小车控制数据
  
     ros::NodeHandle m_handle;                   
     serial::Serial ser;                         
	 
     //msg
     sMartcarControl  moveBaseControl;          
     std_msgs::Float32  currentBattery;          
     qingzhou_cloud::trafficlight currenttrafficlight; 
     std_msgs::Float32 currentAngleMsg;          
     qingzhou_cloud::roadLine sRoadLinePointMsg;
     qingzhou_cloud::startstopCommand startstopCommandMsg; //1.start; 2.stop

	 //订阅话题
     ros::Subscriber sub_imudata;        
     ros::Subscriber sub_move_base; 
     ros::Subscriber sub_trafficlight;     
     ros::Subscriber sub_roadLine_Angle;   //订阅movebase角度
     ros::Subscriber sub_roadline_Points;  //订阅线速度
     ros::Subscriber sub_start_stopCommand;//start stop from mfc
     
	 //发布话题
     ros::Publisher pub_actuator;        
     ros::Publisher pub_odom;             
     ros::Publisher pub_imu;              
     ros::Publisher pub_mag;              
     ros::Publisher poly_pub;            //发布顶点
     ros::Publisher pub_battery;          
     
     void pub_9250();           //发布9250函数
     void recvCarInfoKernel();  //接收下位机发来数据函数
     void sendCarInfoKernel();  //发送小车数据到下位机函数
  

     void callback_imuData(const sensor_msgs::Imu::ConstPtr &msg);         
     void callback_move_base(const geometry_msgs::Twist::ConstPtr &msg);    
  
     void callback_startstopCommand(const qingzhou_cloud::startstopCommand::ConstPtr &msg);
     void callback_trafficlight(const qingzhou_cloud::trafficlight::ConstPtr &msg);
     void callback_roadLineAngle(const std_msgs::Float32::ConstPtr &msg);  
     void callback_roadlinePoints(const qingzhou_cloud::roadLine::ConstPtr &msg);
};

#endif // NODE_EXAMPLE_TALKER_H

#include "qingzhou_bringup/qingzhou_bringup.h"

long long LeftticksPerMeter = 0;    
long long rightticksPerMeter = 0;   
long long LeftticksPer2PI = 0;      
long long rightticksPer2PI = 0;     

// 构造函数，初始化
actuator::actuator(ros::NodeHandle handle) 
{ 
   m_baudrate = 115200;             
   m_serialport = "/dev/ttyUSB0";   
   linearSpeed = 0;                 
   angularSpeed = 0;                
   batteryVoltage = 0;             
   ticksPerMeter = 0;              
   ticksPer2PI = 0;              
   encoderLeft = 0;                
   encoderRight = 0;                
   imuYaw = 0;                      
   velDeltaTime = 0;                
   calibrate_lineSpeed = 0;         
   calibrate_angularSpeed = 0;      
   sRoadLinePointMsg.lineStatus = 0;
   startstopCommandMsg.startstopcommand = 2;

   memset(&moveBaseControl,0,sizeof(sMartcarControl));
   
   handle.param("mcubaudrate",m_baudrate,m_baudrate);                                   //波特率
   handle.param("mcuserialport",m_serialport,std::string("/dev/ttyUSB0"));              //定义传输的串口
   handle.param("calibrate_lineSpeed",calibrate_lineSpeed,calibrate_lineSpeed);         //标定线速度
   handle.param("calibrate_angularSpeed",calibrate_angularSpeed,calibrate_angularSpeed);//标定角速度
   handle.param("ticksPerMeter",ticksPerMeter,ticksPerMeter);                           //一米脉冲数
   handle.param("ticksPer2PI",ticksPer2PI,ticksPer2PI);                                 //每圈脉冲数

    try{ //异常处理
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize start!"<<std::endl;              
      ser.setPort(m_serialport.c_str());                                               
      ser.setBaudrate(m_baudrate);                                                    
      serial::Timeout to = serial::Timeout::simpleTimeout(30);                      
      ser.setTimeout(to);                                                            
      ser.open();                                                                     
    }
    catch (serial::IOException& e){
      std::cout<<"[qingzhou_actuator-->]"<<"Unable to open port!"<<std::endl;                
    }
    if(ser.isOpen()){
      std::cout<<"[qingzhou_actuator-->]"<<"Serial initialize successfully!"<<std::endl;       
    }
    else{
      std::cout<<"[qingzhou_actuator-->]"<<"Serial port failed!"<<std::endl;                  
    } 

    sub_start_stopCommand = handle.subscribe("startStopCommand",1,&actuator::callback_startstopCommand,this);   
    sub_trafficlight = handle.subscribe("trafficLight",1,&actuator::callback_trafficlight,this);
    sub_roadline_Points = handle.subscribe("roadLine",1,&actuator::callback_roadlinePoints,this);    
    sub_roadLine_Angle = handle.subscribe("roadLine_angle",1,&actuator::callback_roadLineAngle,this);
    sub_move_base = handle.subscribe("cmd_vel",1,&actuator::callback_move_base,this);
    
    pub_imu = handle.advertise<sensor_msgs::Imu>("raw", 5);	                                                 
    pub_mag = handle.advertise<sensor_msgs::MagneticField>("imu/mag", 5);                                    
    pub_odom = handle.advertise<nav_msgs::Odometry>("odom", 5);                                               
    poly_pub = handle.advertise<geometry_msgs::PolygonStamped>("polygon",10);                                 
    pub_battery = handle.advertise<std_msgs::Float32>("battery",10);                                          
}

//析构函数
actuator::~actuator() 
{
     
}
//1:start 2:stop
void actuator::callback_startstopCommand(const qingzhou_cloud::startstopCommand::ConstPtr &msg)
{
  startstopCommandMsg = *msg;
	//printf("actuator - callback_startstopCommand = %d\n",startstopCommandMsg.startstopcommand);
}

void actuator::callback_trafficlight(const qingzhou_cloud::trafficlight::ConstPtr &msg){    
    currenttrafficlight = *msg;
    //printf("actuator-->trafficlight distanceX = %.2f,distanceY = %.2f,status = %d\n",currenttrafficlight.X,currenttrafficlight.Y,currenttrafficlight.trafficstatus);
}

void actuator::callback_roadlinePoints(const qingzhou_cloud::roadLine::ConstPtr &msg){
    sRoadLinePointMsg = *msg;
    //printf("actuator --> callback_roadlinePoints,linestatus = %d\n",sRoadLinePointMsg.lineStatus);
  }

void actuator::callback_roadLineAngle(const std_msgs::Float32::ConstPtr &msg){
    currentAngleMsg = *msg;
}

void actuator::callback_move_base(const geometry_msgs::Twist::ConstPtr &msg) 
{
   memset(&moveBaseControl,0,sizeof(sMartcarControl));                       
   
   float v = msg->linear.x;                                                  
   float w = msg->angular.z;                                                 
     
   /*if(startstopCommandMsg.startstopcommand == 2)
   {
    ROS_INFO("please pub start command:");
      v = 0;                                                                 
	  w = 0;
   }*/
   //if(currenttrafficlight.X < 830 && currenttrafficlight.X > 100 && currenttrafficlight.Y < 300 && currenttrafficlight.Y > -400 && currenttrafficlight.trafficstatus == 2){
   if(currenttrafficlight.trafficstatus == 2){
    printf("红灯");
      v = 0;                                                                 
	  w = 0;
   //currenttrafficlight.trafficstatus = 0;
   }
   printf("%f",currenttrafficlight.trafficstatus);
   moveBaseControl.TargetSpeed = v*20*4;                                 
   moveBaseControl.TargetAngle = 0.92*round(atan(w*CARL/v)*57.3);            
   if(v>0)
   moveBaseControl.TargetAngle+=60;         
   else if(v<0)      
  moveBaseControl.TargetAngle=60-moveBaseControl.TargetAngle;                            
   
   //printf("linear:%.2f,angular:%.2f,TargetSpeed:%d,TargetAngle:%d\n",msg->linear.x,msg->angular.z,                  
	      // abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle));
		   
 if(moveBaseControl.TargetAngle < 0){
      moveBaseControl.TargetAngle = 0;             //角度小于0时 等于0
   }
   if(moveBaseControl.TargetAngle > 120){
      moveBaseControl.TargetAngle = 120;         //角度最大120
   }
   printf("linear:%.2f,angular:%.2f,TargetSpeed:%d,TargetAngle:%d\n",msg->linear.x,msg->angular.z,                  
	       moveBaseControl.TargetSpeed,moveBaseControl.TargetAngle);

   //linear speed
   if(moveBaseControl.TargetSpeed > 0)                 
      moveBaseControl.TargetShiftPosition = 0x02;  //前进
   else if(moveBaseControl.TargetSpeed < 0)         
      moveBaseControl.TargetShiftPosition = 0x01;  //后退
   else if(moveBaseControl.TargetSpeed == 0)           
      moveBaseControl.TargetShiftPosition = 0x00;  //停止
	  
   //期望转向角度符号
   if(moveBaseControl.TargetAngle-60 > 0)                 
      moveBaseControl.TargetAngleDir = 0x20;	   //右转（左右跟舵机安装方式有关） 
   else if(moveBaseControl.TargetAngle -60< 0)
      moveBaseControl.TargetAngleDir = 0x10;	   //左转
   else if(moveBaseControl.TargetAngle -60== 0)
      moveBaseControl.TargetAngleDir = 0x00;	   //直行
}
  
void actuator::run()
{
    int run_rate = 50;              
    ros::Rate rate(run_rate);      

    double x = 0.0;                 //x坐标                       
    double y = 0.0;                 //y坐标
    double th = 0.0;

    ros::Time current_time, last_time;                    
	
    while(ros::ok()){
        ros::spinOnce();                                  
		current_time = ros::Time::now();                  //获得当前时间
	    velDeltaTime = (current_time - last_time).toSec();//转换成秒
	    last_time = ros::Time::now();                     //当前时刻存放为上一时刻
		
        recvCarInfoKernel();                              //接收stm32发来的数据
	    pub_9250();                                       //发布imu数据
		
	    currentBattery.data = batteryVoltage/100.0;             //读取当前电池电压
	    pub_battery.publish(currentBattery);              //发布当前电池电压
		

    #if 1
	    if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0;
	    if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        //encoderLeft = -encoderLeft;
	    encoderRight = -encoderRight;                          

	    detEncode = (encoderLeft + encoderRight)/2;            //求编码器平均值
	    detdistance = detEncode/ticksPerMeter;                 
	    detth = (encoderRight - encoderLeft)*2*PI/ticksPer2PI; //计算当前角度 通过标定获得ticksPer2PI

	    linearSpeed = detdistance/velDeltaTime;                
	    angularSpeed = detth/velDeltaTime;                     
		
	    if(detdistance != 0){
	        x += detdistance * cos(th);                        //x坐标
	        y += detdistance * sin(th);                        //y坐标
	    }
	    if(detth != 0){
	        th += detth;                                       //总角度
	    }
       
	    if(calibrate_lineSpeed == 1){
		    printf("x=%.2f,y=%.2f,th=%.2f,linearSpeed=%.2f,detEncode=%.2f,LeftticksPerMeter = %lld,rightticksPerMeter = %lld,batteryVoltage = %.2f,\n",x,y,th,linearSpeed,detEncode,LeftticksPerMeter,rightticksPerMeter,batteryVoltage);
	    }
		
	    //send command to stm32
	    sendCarInfoKernel();                                                     
	    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th); 

	    nav_msgs::Odometry odom;                               //创建nav_msgs::Odometry类型的消息odom
	    odom.header.stamp = current_time;                     
	    odom.header.frame_id = "odom";                         
 	    odom.child_frame_id = "base_link";                     

	    //set the position
	    odom.pose.pose.position.x = x;                       
	    odom.pose.pose.position.y = y;                        
	    odom.pose.pose.position.z = 0.0;                       
	    odom.pose.pose.orientation = odom_quat;               
	
	    odom.twist.twist.linear.x = linearSpeed;               //线速度
	    odom.twist.twist.linear.y = 0;
	    odom.twist.twist.linear.z = 0;
	    odom.twist.twist.angular.x = 0;
	    odom.twist.twist.angular.y = 0;
	    odom.twist.twist.angular.z = angularSpeed;             //角速度
	    if(encoderLeft == 0 && encoderRight == 0){
	        odom.pose.covariance = {1e-9, 0, 0, 0, 0, 0,       
				        0, 1e-3, 1e-9, 0, 0, 0,
				        0, 0, 1e6, 0, 0, 0,
				        0, 0, 0, 1e6, 0, 0,
				        0, 0, 0, 0, 1e6, 0,
				        0, 0, 0, 0, 0, 1e-9};
			odom.twist.covariance = {1e-9, 0, 0, 0, 0, 0,      
						0, 1e-3, 1e-9, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e-9};
		}
		else{
			odom.pose.covariance = {1e-3, 0, 0, 0, 0, 0,       
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3};
			odom.twist.covariance = {1e-3, 0, 0, 0, 0, 0,      
						0, 1e-3, 0, 0, 0, 0,
						0, 0, 1e6, 0, 0, 0,
						0, 0, 0, 1e6, 0, 0,
						0, 0, 0, 0, 1e6, 0,
						0, 0, 0, 0, 0, 1e3};
		}
		//publish the message
		pub_odom.publish(odom);                                
		
		/*******************publish polygon message***********************/
		geometry_msgs::Point32 point[4]; 
		// coordinates described in base_link frame
		point[0].x = -0.18;  point[0].y = -0.12;
		point[1].x = 0.18;   point[1].y = -0.12;
		point[2].x = 0.18;   point[2].y = 0.12;
		point[3].x = -0.18;  point[3].y = 0.12;

		geometry_msgs::PolygonStamped poly;                    
		poly.header.stamp = current_time;
		poly.header.frame_id = "base_link";
		poly.polygon.points.push_back(point[0]);
		poly.polygon.points.push_back(point[1]);
		poly.polygon.points.push_back(point[2]);
		poly.polygon.points.push_back(point[3]);

		poly_pub.publish(poly);                                
	
#endif
        rate.sleep();
    }
}

//发送小车数据到下位机
void actuator::sendCarInfoKernel()
{
    unsigned char buf[23] = {0};
    buf[0] = 0xa5;	                                        
    buf[1] = 0x5a;	                                        
    buf[2] = 0x06;	                                        

    if(sRoadLinePointMsg.lineStatus == 0)
    {                  
	    buf[3] = (int)moveBaseControl.TargetAngleDir;	    // 0-->go straight,0x10-->turn left,0x20-->turn right
	    buf[4] = (int)abs(moveBaseControl.TargetAngle);	    //targetangle 180-->0xb4
	    buf[5] = (int)abs(moveBaseControl.TargetSpeed);		//targetSpeed
	    buf[6] = (int)moveBaseControl.TargetModeSelect;		//0-->person control,1-->auto control
	    buf[7] = (int)moveBaseControl.TargetShiftPosition;  //0-->P stop;1-->R;2-->D.期望档位 停止/倒车/前进
    }
	else if(sRoadLinePointMsg.lineStatus == 1)
    {            
	    if(currentAngleMsg.data > 59){                      
	    	currentAngleMsg.data = 59;
	    }
	    if(currentAngleMsg.data < -59){
	    	currentAngleMsg.data = -59;
	    }
	    buf[3] = 0x10;                                      //与视觉程序对应
	    buf[4] = currentAngleMsg.data + 60;                 //期望转角+60 
	    buf[5] = 0.18*32/0.43;	                            //线速度
	    buf[6] = 1;                                         //自动模式
	    buf[7] = 2;	                                        //前进
    }
  
    buf[8] = 0;		                                        
    unsigned char sum = 0;
    for(int i = 2; i < 19; ++i)                             
        sum += buf[i];
        buf[9] = (unsigned char)(sum);                      
    size_t writesize = ser.write(buf,10);
}

//接收下位机发送来的数据
void actuator::recvCarInfoKernel()
{
    std::string recvstr;                                             
    unsigned char tempdata,lenrecv;                                  
    unsigned char count,last_data,last_last_data,last_last_last_data;
    unsigned char str[100];                                          
    bool recvflag = false;                                          
    bool recvd_flag = false;                                         
    memset(&str,0,sizeof(str));                                      
    ros::Time begin_time = ros::Time::now();                         
    double clustering_time = 0;                                      

    while(1){
	    clustering_time = (ros::Time::now () - begin_time).toSec (); //计算时间差，转换成秒
	    if(clustering_time > 1){                                    
	        recvd_flag = false;                                      
	        break;                                                   
	    }
		
	    recvstr = ser.read(1);                                      		
	    if((int)recvstr.size() != 1)                                 
	        continue;                                               
		
	    tempdata = recvstr[0];                                       
	    if(last_last_last_data == 0xa5 && last_last_data == 0x5a){   
	        lenrecv = last_data;                                     
	        recvflag = true;                                         
	        count = 0;                                               
	    }
	    if(recvflag){                                               
	        str[count] = tempdata;                                   
	        count++;                                                
	        if(count == lenrecv){                                   
		        recvflag = false;                                    
		        recvd_flag = true;                                   
		    break;                                                  
	        }
	    }
	    last_last_last_data = last_last_data;                        
	    last_last_data = last_data;                                  
	    last_data = tempdata;                                        
    }

    if(recvd_flag){                                                  //数据解析，接收到的数据转存
	    memcpy(&encoderLeft,str,4);                                  
	    memcpy(&encoderRight,str+4,4);                                    
      batteryVoltage = *(int*)(&str[8]) ;                      
 	
	    memcpy(&tempaccelX,str+12,2);                                
	    memcpy(&tempaccelY,str+14,2);                                
		memcpy(&tempaccelZ,str+16,2);                              
		
		memcpy(&tempgyroX,str+18,2);                                 
		memcpy(&tempgyroY,str+20,2);                                 
		memcpy(&tempgyroZ,str+22,2);                                
		
		memcpy(&tempmagX,str+24,2);                                 
		memcpy(&tempmagY,str+26,2);                                  
		memcpy(&tempmagZ,str+28,2);                                  
	
		memcpy(&distanceA,str+30,4);                                
		memcpy(&distanceB,str+34,4);                                
		memcpy(&distanceC,str+38,4);                                 
		memcpy(&distanceD,str+42,4);                                 

		accelX = (float)tempaccelX/2048*9.8;                         //加速度处理	
		accelY = (float)tempaccelY/2048*9.8;
		accelZ = (float)tempaccelZ/2048*9.8;

		
		gyroX = (float)tempgyroX/16.4/57.3;                          //角速度处理
		gyroY = (float)tempgyroY/16.4/57.3;
		gyroZ = (float)tempgyroZ/16.4/57.3;
		
		magX = (float)tempmagX*0.14;                                 //磁力计处理
		magY = (float)tempmagY*0.14;
		magZ = (float)tempmagZ*0.14;

        if(encoderLeft > 220 || encoderLeft < -220) encoderLeft = 0; //判断编码器脉冲数是否在正确范围
        if(encoderRight > 220 || encoderRight < -220) encoderRight = 0;
        LeftticksPerMeter += encoderLeft;                            //获得左轮总脉冲数
        rightticksPerMeter += encoderRight;                          //获得右轮总脉冲数
    }
}

//发布imu函数
void actuator::pub_9250(){                         
    sensor_msgs::Imu imuMsg;                       
    sensor_msgs::MagneticField magMsg;           
	
    ros::Time current_time = ros::Time::now();    
	         
    imuMsg.header.stamp = current_time;            
    imuMsg.header.frame_id = "imu_link";          
    imuMsg.angular_velocity.x = gyroX;             
    imuMsg.angular_velocity.y = gyroY;             
    imuMsg.angular_velocity.z = gyroZ;             
    imuMsg.angular_velocity_covariance = {         
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    
    imuMsg.linear_acceleration.x = accelX;        
    imuMsg.linear_acceleration.y = accelY;         
    imuMsg.linear_acceleration.z = accelZ;        
    imuMsg.linear_acceleration_covariance = {      
      0.04,0.0,0.0,
      0.0,0.04,0.0,
      0.0,0.0,0.04
    };
    pub_imu.publish(imuMsg);                       //发布imuMsg 
     
    magMsg.header.stamp = current_time;            
    magMsg.header.frame_id = "base_link";          
    magMsg.magnetic_field.x = magX;                
    magMsg.magnetic_field.y = magY;                
    magMsg.magnetic_field.z = magZ;                
    magMsg.magnetic_field_covariance = {           
      0.0,0.0,0.0,
      0.0,0.0,0.0,
      0.0,0.0,0.0
    };
    pub_mag.publish(magMsg);                       //发布magMsg
}

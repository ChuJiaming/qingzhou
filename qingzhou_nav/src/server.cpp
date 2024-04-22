#include <unistd.h>
#include <sys/types.h>         
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string.h>
#include <stdio.h>
#include<netinet/in.h>
#include<arpa/inet.h>
#include<string.h>
#include<unistd.h>
#include<iostream>
#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include <qingzhou_cloud/trafficlight.h>
#include "geometry_msgs/Twist.h"
#include <thread>
#include<iostream>
#include"std_msgs/Float32.h"

#include <stdio.h>
#include <stdlib.h>

#pragma comment(lib,"ws2_32.lib")   
int socket_fd,fd;
geometry_msgs::PoseStamped goal_chufa,goal_zhuanghuo,goal_xiehuo,goal_podao,goal_s_wan,goal_tingche1,goal_tingche2,goal_temp,goal_temp1,goal_temp2,goal_shibie;
 int daoche_weizhi=0,start_daoche=1,v=0,v_last=0;
 float dianliang;
 ros::Publisher nav_pub;

void do_sigano(const qingzhou_cloud::trafficlight::ConstPtr &msg){    
    daoche_weizhi = msg->trafficstatus;
    ros::Duration du(2.4);
    printf(";;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;");
    if(daoche_weizhi==1)nav_pub.publish(goal_tingche1);
    else nav_pub.publish(goal_tingche2);
    du.sleep();
    du.sleep();
    du.sleep();
    nav_pub.publish(goal_chufa);
}
qingzhou_cloud::trafficlight location_xy;
void do_location(const qingzhou_cloud::trafficlight::ConstPtr &msg)
{
    location_xy=*msg;
}
void receive_dianliang(const std_msgs::Float32::ConstPtr &msg)
{
    dianliang=msg->data;
}

void do_cmd_vel(const geometry_msgs::Twist::ConstPtr &msg) {
    v_last=v;
    v=msg->linear.x;
}

void send_data()
{
    ros::Rate r(1);
    char buffs[24] ={0};
    int a =10086;
    
    while(1)
    {
        float b=(location_xy.X+location_xy.Y)/10;
        buffs[0]=((char*)&a)[0];
        buffs[1]=((char*)&a)[1];
        buffs[2]=((char*)&a)[2];
        buffs[3]=((char*)&a)[3];
        buffs[4]=((char*)&location_xy.X)[0];
        buffs[5]=((char*)&location_xy.X)[1];
        buffs[6]=((char*)&location_xy.X)[2];
        buffs[7]=((char*)&location_xy.X)[3];
        buffs[8]=((char*)&location_xy.Y)[0];
        buffs[9]=((char*)&location_xy.Y)[1];
        buffs[10]=((char*)&location_xy.Y)[2];
        buffs[11]=((char*)&location_xy.Y)[3];
        buffs[12]=((char*)&b)[0];
        buffs[13]=((char*)&b)[1];
        buffs[14]=((char*)&b)[2];
        buffs[15]=((char*)&b)[3];
        buffs[16]=0;
        buffs[17]=0;
        buffs[18]=0;
        buffs[19]=0;
        buffs[20]=((char*)&dianliang)[0];
        buffs[21]=((char*)&dianliang)[1];
        buffs[22]=((char*)&dianliang)[2];
        buffs[23]=((char*)&dianliang)[3];
        send(fd, buffs, sizeof(buffs), 0);        
        r.sleep();
    }

}

void spin(){ros::spin();}
int main(int argc, char  *argv[])
{   

    goal_chufa.header.frame_id = "map";
    goal_chufa.pose.position.x = -0.1; 
    goal_chufa.pose.position.y = 0;  
    goal_chufa.pose.position.z = 0;   
    goal_chufa.pose.orientation.x = 0;
    goal_chufa.pose.orientation.y = 0;
    goal_chufa.pose.orientation.z =-0.0277491741277;
    goal_chufa.pose.orientation.w =0.999614917523;  

    goal_zhuanghuo.header.frame_id = "map";
    goal_zhuanghuo.pose.position.x = 2.25; 
    goal_zhuanghuo.pose.position.y = -3.0;  
    goal_zhuanghuo.pose.position.z = 0;   
    goal_zhuanghuo.pose.orientation.x = 0;
    goal_zhuanghuo.pose.orientation.y = 0;
    goal_zhuanghuo.pose.orientation.z = -0.67939928447;
    goal_zhuanghuo.pose.orientation.w = 0.733768773017;  

    goal_xiehuo.header.frame_id = "map";
    goal_xiehuo.pose.position.x = -1.93; 
    goal_xiehuo.pose.position.y = -6.1;  
    goal_xiehuo.pose.position.z = 0;   
    goal_xiehuo.pose.orientation.x = 0;
    goal_xiehuo.pose.orientation.y = 0;
    goal_xiehuo.pose.orientation.z = 0.697701504311;
    goal_xiehuo.pose.orientation.w = 0.716388589303;  

    goal_podao.header.frame_id = "map";
    goal_podao.pose.position.x = 0; 
    goal_podao.pose.position.y = -7.25146245956;  
    goal_podao.pose.position.z = 0;   
    goal_podao.pose.orientation.x = 0;
    goal_podao.pose.orientation.y = 0;
    goal_podao.pose.orientation.z = 0.999944426183;
    goal_podao.pose.orientation.w = -0.010542511347;  

    goal_s_wan.header.frame_id = "map";
    goal_s_wan.pose.position.x = 1; 
    goal_s_wan.pose.position.y =-4.27;  
    goal_s_wan.pose.position.z = 0;   
    goal_s_wan.pose.orientation.x = 0;
    goal_s_wan.pose.orientation.y = 0;
    goal_s_wan.pose.orientation.z = 0.701633418115;
    goal_s_wan.pose.orientation.w = 0.712538101848;  

    goal_temp.header.frame_id = "map";
    goal_temp.pose.position.x = -0.636; 
    goal_temp.pose.position.y = -4.58;  
    goal_temp.pose.position.z = 0;   
    goal_temp.pose.orientation.x = 0;
    goal_temp.pose.orientation.y = 0;
    goal_temp.pose.orientation.z = -0.50549705982;
    goal_temp.pose.orientation.w = 0.862828327371;  

    goal_temp1.header.frame_id = "map";
    goal_temp1.pose.position.x = 0; 
    goal_temp1.pose.position.y = -5.6;  
    goal_temp1.pose.position.z = 0;   
    goal_temp1.pose.orientation.x = 0;
    goal_temp1.pose.orientation.y = 0;
    goal_temp1.pose.orientation.z = 0.337666296214;
    goal_temp1.pose.orientation.w = 0.94126588826;

    goal_temp2.header.frame_id = "map";
    goal_temp2.pose.position.x = -1.49099111557; 
    goal_temp2.pose.position.y = -0.4;  
    goal_temp2.pose.position.z = 0;   
    goal_temp2.pose.orientation.x = 0;
    goal_temp2.pose.orientation.y = 0;
    goal_temp2.pose.orientation.z = 0.337666296214;
    goal_temp2.pose.orientation.w = 0.94126588826;  

    goal_tingche1.header.frame_id = "map";
    goal_tingche1.pose.position.x = -2.1114756584; 
    goal_tingche1.pose.position.y = -0.532923221588;  
    goal_tingche1.pose.position.z = 0;   
    goal_tingche1.pose.orientation.x = 0;
    goal_tingche1.pose.orientation.y = 0;
    goal_tingche1.pose.orientation.z = 0;
    goal_tingche1.pose.orientation.w = 1;  

    goal_tingche2.header.frame_id = "map";
    goal_tingche2.pose.position.x = -2.1215514183; 
    goal_tingche2.pose.position.y = -0.97631244183;  
    goal_tingche2.pose.position.z = 0;   
    goal_tingche2.pose.orientation.x = 0;
    goal_tingche2.pose.orientation.y = 0;
    goal_tingche2.pose.orientation.z = 0;
    goal_tingche2.pose.orientation.w = 1;  

    goal_shibie.header.frame_id = "map";
    goal_shibie.pose.position.x = -1.49099111557; 
    goal_shibie.pose.position.y = -0.234701633453;  
    goal_shibie.pose.position.z = 0;   
    goal_shibie.pose.orientation.x = 0;
    goal_shibie.pose.orientation.y = 0;
    goal_shibie.pose.orientation.z = 0.696066670881;
    goal_shibie.pose.orientation.w = 0.71797715123;  

    ros::init(argc,argv,"server");
    ros::NodeHandle nh;
    nav_pub=nh.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);
    ros::Subscriber sub = nh.subscribe("daoche",1,&do_sigano);
    ros::Subscriber sub1 = nh.subscribe("battery",1,&receive_dianliang);
    ros::Subscriber sub2 = nh.subscribe("send_tf_xy",1,&do_location);
    // ros::Subscriber sub_cmd_vel = nh.subscribe("cmd_vel",1,&do_cmd_vel);

    int ret;
    char send_buff[64] = "Data was send by server.";    //连接上发给客户端的数据
    char recv_buff[64];
    char client_ip[32];
    struct sockaddr_in client_msg;
    unsigned int msg_len = sizeof(client_msg);
    struct sockaddr_in server_addr;
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(8080);
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    socket_fd = socket(AF_INET,SOCK_STREAM,0);
    if(socket_fd < 0){
        perror("socket");
        exit(0);
    }
    ret = bind(socket_fd,(struct sockaddr *)&server_addr,sizeof(server_addr));
    if(ret == -1){
        perror("bind");
        close(socket_fd);
        exit(0);
    }
 
    ret = listen(socket_fd,32);
    if(ret == -1){
        perror("listen");
        close(socket_fd);
        exit(0);
    }
    printf("listen...\n");
    fd = accept(socket_fd,(struct sockaddr *)&client_msg,&msg_len);
    inet_ntop(AF_INET,&client_msg.sin_addr.s_addr,client_ip,sizeof(client_ip));
    
    send(fd,send_buff,sizeof(send_buff),0);
    std::thread t(spin);
    std::thread aaa(send_data);
    printf("client ip addr:%s,port:%d\n\n",client_ip,client_msg.sin_port);
    while(ros::ok())
    {
        memset(recv_buff,0,sizeof(recv_buff));
        recv(fd,recv_buff,sizeof(recv_buff),0);    
        printf("%d\n",recv_buff[0]);
        if(strncmp("quit",recv_buff,4) == 0){
            printf("socket closer.\n");
            close(fd);
            close(socket_fd);
            exit(0);
        }
        if(recv_buff[0]==48)//"0"
        {
            nav_pub.publish(goal_zhuanghuo);
        }
        if(recv_buff[0]==49)//"1"
        {
            nav_pub.publish(goal_podao);
            ros::Duration du(3);
            du.sleep();
            nav_pub.publish(goal_xiehuo);
        }
        if(recv_buff[0]==50)//"2"
        {
            nav_pub.publish(goal_temp);
            ros::Duration du(1.5);
            du.sleep();
            du.sleep();
            nav_pub.publish(goal_s_wan);
        }
    }
    return 0;
}


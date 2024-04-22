#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Transform, TransformStamped
from std_msgs.msg import Header
from std_msgs.msg import Float32MultiArray
import math
import tf2_ros
import tf_conversions

class OdometryNode:
    # Set publishers
    pose_X=0
    pose_Y=0
    count=0
    def __init__(self):
        # init internals
        self.last_received_pose = Pose()
        self.last_received_twist = Twist()
        self.last_recieved_stamp = None
        self.pub_odom = rospy.Publisher('/odom', Odometry, queue_size=1)
        self.sub_data_from_stm32=rospy.Subscriber("/data_from_stm32",Float32MultiArray,self.callback,queue_size=10)
        self.tf_pub = tf2_ros.TransformBroadcaster()

    def callback(self, data_from_stm32): 
        self.pose_X +=data_from_stm32.data[1]*math.cos(data_from_stm32.data[0])*0.00333333
        self.pose_Y +=data_from_stm32.data[1]*math.sin(data_from_stm32.data[0])*0.00333333
        if self.count<20:
            self.count+=1

        else:
            cmd = Odometry()

            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = 'odom'
            cmd.child_frame_id = 'base_link'
            cmd.pose.pose.position.x=self.pose_X
            cmd.pose.pose.position.y=self.pose_Y
            cmd.pose.pose.position.z=0
            qtn = tf_conversions.transformations.quaternion_from_euler(0, 0, data_from_stm32.data[0])
            cmd.pose.pose.orientation.x=qtn[0]
            cmd.pose.pose.orientation.y=qtn[1]
            cmd.pose.pose.orientation.z=qtn[2]
            cmd.pose.pose.orientation.w=qtn[3]

            cmd.twist.twist.linear.x=data_from_stm32.data[1]
            cmd.twist.twist.linear.y=0
            cmd.twist.twist.linear.z=0
            cmd.twist.twist.angular.x=0
            cmd.twist.twist.angular.y=0
            cmd.twist.twist.angular.z=data_from_stm32.data[2]
            cmd.pose.covariance =[1e-3, 0, 0, 0, 0, 0,
                            0, 1e-3, 0, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e3]

            cmd.twist.covariance = [1e-9, 0, 0, 0, 0, 0, 
                            0, 1e-3, 1e-9, 0, 0, 0,
                            0, 0, 1e6, 0, 0, 0,
                            0, 0, 0, 1e6, 0, 0,
                            0, 0, 0, 0, 1e6, 0,
                            0, 0, 0, 0, 0, 1e-9]
            tf = TransformStamped(
                header=Header(
                    frame_id=cmd.header.frame_id,
                    stamp=cmd.header.stamp
                ),
                child_frame_id=cmd.child_frame_id,
                transform=Transform(
                    translation=cmd.pose.pose.position,
                    rotation=cmd.pose.pose.orientation
                )
            )
            self.tf_pub.sendTransform(tf)
            self.pub_odom.publish(cmd)
            self.count=0

if __name__ == '__main__':
    rospy.init_node("gazebo_odometry_node")
    node = OdometryNode()
    rospy.spin()


#!/usr/bin/env python
# coding=utf-8

import tf2_ros
import rospy
from qingzhou_cloud.msg import trafficlight


def send_tf2():
    global xy_pub
    xy_msg = trafficlight()
    rate = rospy.Rate(100)
    buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(buffer)    
    while not rospy.is_shutdown():
        rate.sleep()
        try:
            # def lookup_transform(self, target_frame, source_frame, time, timeout=rospy.Duration(0.0)):
            trans = buffer.lookup_transform("map","base_link",rospy.Time(0),rospy.Duration(0.5))
            rospy.loginfo("相对坐标:(%.2f,%.2f,%.2f)",
                        trans.transform.translation.x,
                        trans.transform.translation.y,
                        trans.transform.translation.z
                        )   
            xy_msg.X = trans.transform.translation.x
            xy_msg.Y = trans.transform.translation.y
            xy_pub.publish(xy_msg)
        except Exception as e:
            rospy.logwarn("警告:%s",e)

if __name__ == "__main__":
    rospy.init_node("send_tf_node")
    xy_pub = rospy.Publisher("/send_tf_xy",trafficlight,queue_size=10)
    send_tf2()
    
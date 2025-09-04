#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped

def odom_callback(robot_id, msg):
    # 创建 tf 广播器
    br = tf.TransformBroadcaster()

    # 设置 tf 的父子坐标系
    translation = (msg.pose.pose.position.x,
                   msg.pose.pose.position.y,
                   msg.pose.pose.position.z)
    rotation = (msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w)

    # 广播 tf
    br.sendTransform(translation,
                     rotation,
                     rospy.Time.now(),
                     "/base_link",
                      "/odom")

def callback_robot1(msg):
    odom_callback("robot1", msg)

if __name__ == '__main__':
    rospy.init_node('multi_robot_odom_tf_broadcaster')
   
    # 为每个机器人订阅 odom 话题
    rospy.Subscriber('/odom', Odometry, callback_robot1)

    rospy.spin()


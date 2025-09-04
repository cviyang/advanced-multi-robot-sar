#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import actionlib
import math
import threading
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# 固定目标点：robot1 -> (-6.25, -13.8), robot2 -> (12.2, 4.86), robot3 -> (-4.55, 26.9)
GOALS = {
    'robot1': (-6.25, -13.8, 0.0),  # (x, y, yaw)
    'robot2': (12.2,    4.86, 0.0),
    'robot3': (-4.55,  26.9,  0.0),
}

def yaw_to_quat(yaw):
    """将 yaw 转为 (x,y,z,w)，roll=pitch=0"""
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))

def send_goal_worker(robot_ns, x, y, yaw=0.0, wait_timeout=0):
    try:
        client = actionlib.SimpleActionClient('%s/move_base' % robot_ns, MoveBaseAction)
        rospy.loginfo("[%s] 等待 move_base 服务器...", robot_ns)
        client.wait_for_server()

        goal = MoveBaseGoal()
        # 若使用全局融合地图，把下一行改成: goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.frame_id = "%s/map" % robot_ns
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        qx, qy, qz, qw = yaw_to_quat(yaw)
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        rospy.loginfo("[%s] 发送目标: (%.3f, %.3f, yaw=%.3f)", robot_ns, x, y, yaw)
        client.send_goal(goal)

        if wait_timeout > 0:
            finished = client.wait_for_result(rospy.Duration(wait_timeout))
        else:
            finished = client.wait_for_result()

        state = client.get_state()
        rospy.loginfo("[%s] 结果: finished=%s, state=%s", robot_ns, finished, state)
    except rospy.ROSInterruptException:
        rospy.logwarn("[%s] 被中断", robot_ns)
    except Exception as e:
        rospy.logerr("[%s] 发送或等待结果出错: %s", robot_ns, str(e))

def main():
    rospy.init_node('multi_robot_fixed_goals_parallel', anonymous=True)

    threads = []
    for robot, (x, y, yaw) in GOALS.items():
        t = threading.Thread(target=send_goal_worker, args=(robot, x, y, yaw))
        t.daemon = True  # 随主进程退出
        t.start()
        threads.append(t)

    # 等待所有线程完成（可选）
    for t in threads:
        t.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

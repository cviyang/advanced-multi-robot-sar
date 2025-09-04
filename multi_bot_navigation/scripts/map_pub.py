#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf
import math

def main():
    rospy.init_node('link3_to_link4_tf_broadcaster')

    # 和你URDF里 Joint4 的 <origin rpy="pi/2 0 -pi/2">、<origin xyz="-0.045 0 0.055">一致
    trans = (-0.045, 0.0, 0.055)
    q = tf.transformations.quaternion_from_euler(
        1.5707963267949,  # roll =  pi/2
        0.0,              # pitch = 0
        -1.5707963267949  # yaw = -pi/2
    )

    br = tf.TransformBroadcaster()
    robots = ['robot1', 'robot2', 'robot3']
    rate = rospy.Rate(20)  # 20 Hz 和命令行一致

    rospy.loginfo("Publishing TF: %s", ", ".join(
        ["%s/Link3 -> %s/Link4" % (r, r) for r in robots]
    ))

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for r in robots:
            parent = r + "/Link3"
            child  = r + "/Link4"
            br.sendTransform(trans, q, now, child, parent)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

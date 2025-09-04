#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import json
import rospy
import tf2_ros
import tf_conversions
import geometry_msgs.msg

def load_object_positions(filename):
    with open(filename, 'r') as file:
        return json.load(file)

def publish_transforms(object_positions):
    broadcaster = tf2_ros.StaticTransformBroadcaster()
    transforms = []
    
    for object_class, objects in object_positions.items():
        for object_id, position in objects.items():
            transform = geometry_msgs.msg.TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = 'map'
            transform.child_frame_id = object_id
            transform.transform.translation.x = position['x']
            transform.transform.translation.y = position['y']
            transform.transform.translation.z = position['z']
            transform.transform.rotation.x = 0
            transform.transform.rotation.y = 0
            transform.transform.rotation.z = 0
            transform.transform.rotation.w = 1  # No rotation
            transforms.append(transform)
    
    broadcaster.sendTransform(transforms)

def main():
    rospy.init_node('object_position_publisher')
    object_positions = load_object_positions('/home/zhitao/object_positions.json')
    publish_transforms(object_positions)
    rospy.spin()  # Keep the node running

if __name__ == '__main__':
    main()

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import TransformBroadcaster
from yolov8_ros_msgs.msg import BoundingBoxes
from semantic_slam.srv import CamToReal, CamToRealRequest
from visualization_msgs.msg import Marker, MarkerArray
import math
import json
import os

class TfBroadcast:
    def __init__(self):
        rospy.init_node('tf_broadcast')
        self.tf_broadcaster = TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.marker_publisher = rospy.Publisher('/people_position', MarkerArray, queue_size=10)
        self.dist_client = rospy.ServiceProxy('cam_to_real', CamToReal)
        rospy.Subscriber('/yolov8/BoundingBoxes', BoundingBoxes, self.yolo_callback)
        self.object_positions = {}  # Dictionary to store object classes and their positions
        self.object_counts = {}  # Dictionary to count instances of each class
        self.marker_array = MarkerArray()

        rospy.on_shutdown(self.save_object_positions)

    def check_distance(self, pos1, pos2):
        distance = math.sqrt((pos1['x'] - pos2['x']) ** 2 +
                             (pos1['y'] - pos2['y']) ** 2 +
                             (pos1['z'] - pos2['z']) ** 2)
        return distance > 5

    def update_object_positions(self, box_class, transformed_pose):
        current_position = {
            'x': transformed_pose.pose.position.x,
            'y': transformed_pose.pose.position.y,
            'z': transformed_pose.pose.position.z
        }

        if box_class not in self.object_positions:
            self.object_positions[box_class] = {}
            self.object_counts[box_class] = 0
        else:
            for key, position in self.object_positions[box_class].items():
                if not self.check_distance(position, current_position):
                    return key  # Return existing object key

            self.object_counts[box_class] += 1

        new_key = "{}_{}".format(box_class, self.object_counts[box_class])
        self.object_positions[box_class][new_key] = current_position
        return new_key

    def publish_marker(self, position, box_class, object_id):
        # Sphere marker for the object
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = box_class
        marker.id = object_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position['x']
        marker.pose.position.y = position['y']
        marker.pose.position.z = position['z']
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.marker_array.markers.append(marker)

        # Text marker for the class name
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = box_class + "_text"
        text_marker.id = object_id + 1000  # Use a different ID to avoid conflict with sphere marker
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = position['x']
        text_marker.pose.position.y = position['y']
        text_marker.pose.position.z = position['z'] + 1.0  # Slightly above the object
        text_marker.pose.orientation.x = 0.0
        text_marker.pose.orientation.y = 0.0
        text_marker.pose.orientation.z = 0.0
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.5  # Font size
        text_marker.color.a = 1.0
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.text = box_class  # Display the class name
        self.marker_array.markers.append(text_marker)

        # Publish the marker array
        self.marker_publisher.publish(self.marker_array)

    def transform_and_broadcast(self, pose_stamped, box_class, index):
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'camera_frame_optical', rospy.Time(0), rospy.Duration(1.0))
            transformed_pose = do_transform_pose(pose_stamped, transform)
            new_key = self.update_object_positions(box_class, transformed_pose)

            # Publish marker
            position = self.object_positions[box_class][new_key]
            self.publish_marker(position, box_class, index)
            return transformed_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform position: %s", str(e))
            return None

    def yolo_callback(self, yolo_tmp):
        for i, box in enumerate(yolo_tmp.bounding_boxes):
            pixel_x = (box.xmin + box.xmax) / 2.0
            pixel_y = (box.ymin + box.ymax) / 2.0
            self.dist_req = CamToRealRequest(pixel_x=pixel_x, pixel_y=pixel_y)
            try:
                self.dist_resp = self.dist_client.call(self.dist_req)
                if self.dist_resp.result:
                    pose_stamped = PoseStamped()
                    pose_stamped.header.frame_id = 'camera_frame_optical'
                    pose_stamped.header.stamp = rospy.Time.now()
                    pose_stamped.pose.position.x = self.dist_resp.obj_x
                    pose_stamped.pose.position.y = self.dist_resp.obj_y
                    pose_stamped.pose.position.z = self.dist_resp.obj_z
                    pose_stamped.pose.orientation.w = 1.0
                    self.transform_and_broadcast(pose_stamped, box.Class, i)
            except rospy.ServiceException as e:
                rospy.logerr("Failed to process object position: %s", str(e))

    def save_object_positions(self):
        with open(os.path.expanduser('~/object_positions.json'), 'w') as file:
            json.dump(self.object_positions, file, indent=4)
        rospy.loginfo("Saved object positions to object_positions.json")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        tf_broadcast = TfBroadcast()
        tf_broadcast.run()
    except rospy.ROSInterruptException:
        pass

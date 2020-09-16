#!/usr/bin/env python
"""
This adds as an intermediate layer between merged point cloud and Octomap point cloud topic.
"""
import sys
import rospy
import cv2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2, Image
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2
import tf2_ros

import tf

import numpy as np

from motion_planning.msg import PointCloudFilterAction
bridge = CvBridge()

class PointCloudFilter():
    '''
    On receiving point cloud messages, merge them together
    Using message_filter for time synchronization
    '''
    def __init__(self):

        self.pcd_pub = rospy.Publisher("/camera/depth/points_filtered", PointCloud2, queue_size=10)
        self.camera_sub = rospy.Subscriber("/camera/depth/points_merged", PointCloud2, self.camera_callback)

        # specify the states and actions for filtering
        self.NO_FILTER = 0
        self.APPLY_FILTER = 1
        self.filter_state = self.NO_FILTER
        self.filter_action_sub = rospy.Subscriber("/motion_planning/pcd_filter_action", PointCloudFilterAction, self.filter_action_callback)

        # for transforming the point cloud to the proper cooridnate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        rospy.loginfo("#######################Pointcloud fiter initialized###############################")
    def filter_action_callback(self, msg):
        if msg.action == msg.StartFilter:
            # change current state to filter
            self.filter_state = self.APPLY_FILTER
            # copy the data necessary for filtering
            padding = 0.1
            self.filter_min_xyz = np.array(msg.min_xyz) - padding
            self.filter_max_xyz = np.array(msg.max_xyz) + padding
            self.filter_transform = np.array(msg.pcd_transform).reshape((4,4))

        elif msg.action == msg.StopFilter:
            self.filter_state = self.NO_FILTER

    def camera_callback(self, pcd_msg):
        # transform frame
        trans = self.tf_buffer.lookup_transform('world', pcd_msg.header.frame_id, rospy.Time(0), rospy.Duration(1))  # ask for last known info
        points = do_transform_cloud(pcd_msg, trans)
        if self.filter_state == self.NO_FILTER:
            print('##########NOT filtering point cloud########')
            points.header = std_msgs.msg.Header()
            points.header.stamp = rospy.Time.now()
            points.header.frame_id = 'world'
            self.pcd_pub.publish(points)
        else:
            print('##########filtering point cloud########')
            pcd = list(pcl2.read_points(points, field_names = ("x", "y", "z"), skip_nans=True))
            pcd = np.array(pcd)
            pcd_add_axis = np.ones((len(pcd),4))
            pcd_add_axis[:,:3] = pcd
            pcd_transformed = self.filter_transform.dot(pcd_add_axis.T).T
            pcd_transformed = pcd_transformed[:,:3]
            box_mask_min = np.prod((pcd_transformed - self.filter_min_xyz) >= 0, axis=1)  # AND across x-y-z axis
            box_mask_max = np.prod((self.filter_max_xyz - pcd_transformed) >= 0, axis=1)  # AND across x-y-z axis
            box_mask = box_mask_min * box_mask_max  # should satisfy both min and max constraints to be inside the box
            # anything else stays

            # filter by the mask
            print('before filter: pcd length: %d' % (len(pcd)))
            pcd = pcd[box_mask == 0]  # anything not inside the box
            print('after filter: pcd length: %d' % (len(pcd)))

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = 'world'
            #create pcl from points
            filtered_pcd = pcl2.create_cloud_xyz32(header, pcd)
            #publish
            self.pcd_pub.publish(filtered_pcd)



if __name__ == '__main__':
    rospy.init_node('pcd_filter')
    pointCloudFilter = PointCloudFilter()
    rospy.spin()

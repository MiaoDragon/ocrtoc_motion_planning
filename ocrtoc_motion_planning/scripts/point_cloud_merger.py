#!/usr/bin/env python
import rospy
import math
import sys

from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2
import tf2_ros

import numpy as np

class PointCloudMerger():
    '''
    On receiving point cloud messages, merge them together
    Using message_filter for time synchronization
    '''
    def __init__(self):
        self.merge_pub = rospy.Publisher("/camera/depth/points_merged", PointCloud2, queue_size=10)
        camera_subs = []
        camera_subs.append(message_filters.Subscriber('/kinect/depth/points', PointCloud2))
        camera_subs.append(message_filters.Subscriber('/realsense/depth/points', PointCloud2))

        self.ts = message_filters.ApproximateTimeSynchronizer(camera_subs, 10, 1)
        #self.ts = message_filters.TimeSynchronizer(camera_subs, 10)
        self.ts.registerCallback(self.merge)
        # for transforming the point cloud to the proper cooridnate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_sample_ratio = 0.5  # how many to keep

        rospy.loginfo("#######################Pointcloud merger initialized###############################")
    """
    def filter(xyz, radius, epsilon):
        # from https://github.com/aipiano/guided-filter-point-cloud-denoise/blob/master/main.py
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(np.array(xyz))
        kdtree = o3d.KDTreeFlann(pcd)
        points_copy = np.array(pcd.points)
        points = np.asarray(pcd.points)
        num_points = len(pcd.points)

        for i in range(num_points):
            k, idx, _ = kdtree.search_radius_vector_3d(pcd.points[i], radius)
            if k < 3:
                continue

            neighbors = points[idx, :]
            mean = np.mean(neighbors, 0)
            cov = np.cov(neighbors.T)
            e = np.linalg.inv(cov + epsilon * np.eye(3))

            A = cov.dot(e)
            b = mean - A.dot(mean)

            points_copy[i] = A.dot(points[i]) + b
        return points_copy
    """

    def merge(self, *args):
        """
            merge received point clouds
            also preprocess: delete points that are outside of the boundary
        """
        # take any number of input arguments

        merged_points = []
        for points in args:
            # obtain the tf transform for this link
            #trans = self.tf_buffer.lookup_transform('world', points.header.frame_id, points.header.stamp, rospy.Duration(1))
            print('points header frame:')
            print(points.header.frame_id)
            trans = self.tf_buffer.lookup_transform('world', points.header.frame_id, rospy.Time(0), rospy.Duration(1))  # ask for last known info
            points = do_transform_cloud(points, trans)
            points_list = []
            for p in pcl2.read_points(points, field_names = ("x", "y", "z"), skip_nans=True):
                # check boundary info
                ### TODO: edit this boundary checking for customization
                """
                if p[0] >= -0.5 and p[0] <= 0.5 and p[1] >= -0.5 and p[1] <= 0.5:
                    # remove this point cloud, otherwise can't generate collision-free planning
                    continue
                if p[2] >= 1.8:
                    # remove camera point cloud
                    continue
                """
                # subsample
                value = np.random.uniform(low=0.,high=1.0)
                if value <= self.sub_sample_ratio:
                    points_list.append([p[0],p[1],p[2]])
            merged_points += points_list
        print('length of point cloud: %d' % (len(merged_points)))
        #merged_points = filter(merged_points, 0.01, 0.1)

        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        header.frame_id = 'world'
        #create pcl from points
        merged_pointcloud2 = pcl2.create_cloud_xyz32(header, merged_points)
        #publish
        self.merge_pub.publish(merged_pointcloud2)

if __name__ == '__main__':
    rospy.init_node('pcd_merger')
    pointCloudMerger = PointCloudMerger()
    rospy.spin()

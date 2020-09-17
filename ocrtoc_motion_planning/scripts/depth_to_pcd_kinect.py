#!/usr/bin/env python
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
bridge = CvBridge()

class DepthToPCDKinect():
    '''
    On receiving point cloud messages, merge them together
    Using message_filter for time synchronization
    '''
    def __init__(self):

        self.pcd_pub = rospy.Publisher("/kinect/depth/pcd_from_depth", PointCloud2, queue_size=10)
        camera_subs = []
        camera_subs.append(message_filters.Subscriber('/kinect/color/image_raw', Image))
        camera_subs.append(message_filters.Subscriber('/kinect/depth/image_raw', Image))
        #camera_subs.append(message_filters.Subscriber('/realsense/color/image_raw', Image))
        #camera_subs.append(message_filters.Subscriber('/realsense/depth/image_raw', Image))

        self.ts = message_filters.ApproximateTimeSynchronizer(camera_subs, 10, 1)
        #self.ts = message_filters.TimeSynchronizer(camera_subs, 10)
        self.ts.registerCallback(self.to_pcd)
        # for transforming the point cloud to the proper cooridnate
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.sub_sample_ratio = 0.5  # how many to keep

        rospy.loginfo("#######################Pointcloud merger initialized###############################")
    def color_depth_to_pcd(self, color, depth, cam_intrinsics):
        color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
        depth_o3d = o3d.geometry.Image(depth)
        rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=10000, convert_rgb_to_intensity=False)
        intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic()
        intrinsics_o3d.intrinsic_matrix = cam_intrinsics
        pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_o3d)
        return pcd
    def pcd_msg_to_pcd(self, pcd_msg):
        print("pcd_msg header:")
        print(pcd_msg.header.frame_id)
        trans = self.tf_buffer.lookup_transform('world', pcd_msg.header.frame_id, rospy.Time(0), rospy.Duration(1))  # ask for last known info
        points = do_transform_cloud(pcd_msg, trans)
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
        return points_list

    def to_pcd(self, color1_msg, depth1_msg):
        """
            merge received point clouds
            also preprocess: delete points that are outside of the boundary
        """
        # ROS to cv2
        color1 = bridge.imgmsg_to_cv2(color1_msg, 'bgr8')
        depth1 = bridge.imgmsg_to_cv2(depth1_msg, 'passthrough')

        # apply filter
        kernel_size = 11
        kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
        depth1 = cv2.filter2D(depth1, -1, kernel)
        depth1 = (10000 * depth1).astype(np.uint16)
 
        #cam_intrinsics_1 = np.array([
        #    [1120.1199, 0, 640.5],
        #    [0, 1120.1199, 360.5],
        #    [0, 0, 1]
        #])
        cam_intrinsics_1 = "1120.1199067175087, 0.0, 640.5, 0.0, 1120.1199067175087, 360.5, 0.0, 0.0, 1.0"
        cam_intrinsics_1 = cam_intrinsics_1.split(", ")
        cam_intrinsics_1 = [float(v) for v in cam_intrinsics_1]
        cam_intrinsics_1 = np.array(cam_intrinsics_1).reshape((3,3))
        pcd1 = self.color_depth_to_pcd(color1, depth1, cam_intrinsics_1)

        pcd1 = np.asarray(pcd1.points)
        print('pcd1: ')
        print(np.asarray(pcd1))
        # fix orientation issue of the camera by rotating
        pcd1_transform = np.zeros((len(pcd1),4))
        pcd1_transform[:,:3] = pcd1
        pcd1_transform[:,3] = 1.
        Re = tf.transformations.euler_matrix(0, np.pi/2,-np.pi/2, 'rxyz')
        pcd1_transform = Re.dot(pcd1_transform.T).T
        pcd1 = pcd1_transform[:,:3]

        print('frame1: ')
        print(depth1_msg.header.frame_id)
        # convert to PointCloud message to transform and merge
        header = std_msgs.msg.Header()
        header.stamp = rospy.Time.now()
        #header.frame_id = 'kinect_camera_base'
        header.frame_id =depth1_msg.header.frame_id
        pcd1_msg = pcl2.create_cloud_xyz32(header, pcd1)
        print('pcd1_msg:')
        print(pcd1_msg.header)
        # pcd1_msg is correct after publishing
        self.pcd_pub.publish(pcd1_msg)  # Now we only use kinect

if __name__ == '__main__':
    rospy.init_node('depth_to_pcd')
    pointCloudMerger = DepthToPCDKinect()
    rospy.spin()

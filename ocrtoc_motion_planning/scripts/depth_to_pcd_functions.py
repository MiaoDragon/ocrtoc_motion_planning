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

from sensor_msgs.msg import PointCloud2, Image, CameraInfo
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2
import message_filters

from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import tf2_py as tf2
import tf2_ros

import tf

import numpy as np

def color_depth_to_pcd(color, depth, cam_intrinsics):
    color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    depth_o3d = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=10000, convert_rgb_to_intensity=False)
    intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic()
    intrinsics_o3d.intrinsic_matrix = cam_intrinsics
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_o3d)
    return pcd

def to_pcd(bridge, color1_msg, depth1_msg, subsample_ratio = 1.0):
    """
        merge received point clouds
        also preprocess: delete points that are outside of the boundary
    """
    # ROS to cv2
    color1 = bridge.imgmsg_to_cv2(color1_msg, 'bgr8')
    depth1 = bridge.imgmsg_to_cv2(depth1_msg, 'passthrough')

    # apply filter
    kernel_size = 11
    #kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
    #depth1 = cv2.filter2D(depth1, -1, kernel)
    # print('shape:')
    # print(depth1.shape)
    # print('max:')
    # print(depth1.max())
    # print('min:')
    # print(depth1.min())
    # print('median:')
    # print(np.median(depth1))
    depth1 = cv2.bilateralFilter(src=depth1, d=11, sigmaColor=0.05, sigmaSpace=0.65)
    
    depth1 = (10000 * depth1).astype(np.uint16)

    #cam_intrinsics_1 = np.array([
    #    [1120.1199, 0, 640.5],
    #    [0, 1120.1199, 360.5],
    #    [0, 0, 1]
    #])


    # read camera intrinsics
    #cam_intrinsics_1 = "1120.1199067175087, 0.0, 640.5, 0.0, 1120.1199067175087, 360.5, 0.0, 0.0, 1.0"
    #cam_intrinsics_1 = cam_intrinsics_1.split(", ")
    #cam_intrinsics_1 = [float(v) for v in cam_intrinsics_1]
    #cam_intrinsics_1 = np.array(cam_intrinsics_1).reshape((3,3))
    # read camera intrinsics from topic

    cam_info = rospy.wait_for_message('/kinect/color/camera_info', CameraInfo)
    cam_intrinsics_1 = cam_info.K
    cam_intrinsics_1 = np.array(cam_intrinsics_1).reshape((3,3))

    pcd1 = color_depth_to_pcd(color1, depth1, cam_intrinsics_1)

    pcd1 = np.asarray(pcd1.points)
    #print('pcd1: ')
    #print(np.asarray(pcd1))
    # fix orientation issue of the camera by rotating

    # **comment out below when not using transformation
    # pcd1_transform = np.zeros((len(pcd1),4))
    # pcd1_transform[:,:3] = pcd1
    # pcd1_transform[:,3] = 1.
    # Re = tf.transformations.euler_matrix(0, np.pi/2,-np.pi/2, 'rxyz')
    # pcd1_transform = Re.dot(pcd1_transform.T).T
    # pcd1 = pcd1_transform[:,:3]

    # subsample
    subsample_every_n = int(1 / subsample_ratio)
    pcd1 = pcd1[::subsample_every_n]


    #print('frame1: ')
    #print(depth1_msg.header.frame_id)
    # convert to PointCloud message to transform and merge
    header = std_msgs.msg.Header()
    #header.stamp = rospy.Time.now()
    #header.frame_id = 'kinect_camera_base'
    header.frame_id =depth1_msg.header.frame_id
    pcd1_msg = pcl2.create_cloud_xyz32(header, pcd1)
    #print('pcd1_msg:')
    #print(pcd1_msg.header)
    return pcd1_msg

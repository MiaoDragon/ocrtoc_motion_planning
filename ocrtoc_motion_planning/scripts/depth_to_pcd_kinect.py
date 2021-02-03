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


import tf

import numpy as np



sub_sample_ratio = 0.2

def color_depth_to_pcd(color, depth, cam_intrinsics):
    color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    depth_o3d = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=10000, convert_rgb_to_intensity=False)
    intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic()
    intrinsics_o3d.intrinsic_matrix = cam_intrinsics
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_o3d)
    return pcd


def to_pcd(color1_msg, depth1_msg):
    # ROS to cv2
    color1 = bridge.imgmsg_to_cv2(color1_msg, 'bgr8')
    depth1 = bridge.imgmsg_to_cv2(depth1_msg, 'passthrough')

    # apply filter
    kernel_size = 11
    kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
    depth1 = cv2.filter2D(depth1, -1, kernel)
    depth1 = (10000 * depth1).astype(np.uint16)

    cam_intrinsics_1 = "1120.1199067175087, 0.0, 640.5, 0.0, 1120.1199067175087, 360.5, 0.0, 0.0, 1.0"
    cam_intrinsics_1 = cam_intrinsics_1.split(", ")
    cam_intrinsics_1 = [float(v) for v in cam_intrinsics_1]
    cam_intrinsics_1 = np.array(cam_intrinsics_1).reshape((3,3))
    pcd1 = color_depth_to_pcd(color1, depth1, cam_intrinsics_1)

    pcd1 = np.asarray(pcd1.points)

    # fix orientation issue of the camera by rotating
    pcd1_transform = np.zeros((len(pcd1),4))
    pcd1_transform[:,:3] = pcd1
    pcd1_transform[:,3] = 1.
    Re = tf.transformations.euler_matrix(0, np.pi/2,-np.pi/2, 'rxyz')
    pcd1_transform = Re.dot(pcd1_transform.T).T
    pcd1 = pcd1_transform[:,:3]


    # convert to PointCloud message to transform and merge
    header = std_msgs.msg.Header()
    header.stamp = rospy.Time.now()
    #header.frame_id = 'kinect_camera_base'
    header.frame_id =depth1_msg.header.frame_id
    pcd1_msg = pcl2.create_cloud_xyz32(header, pcd1)

    return pcd1_msg

if __name__ == '__main__':
    rospy.init_node('depth_to_pcd')
    print("#######################Pointcloud merger initialized###############################")
    bridge = CvBridge()
    

    pcd_pub = rospy.Publisher("/kinect/depth/pcd_from_depth", PointCloud2, queue_size=10)
    # for transforming the point cloud to the proper cooridnate
    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        color1_msg = rospy.wait_for_message('kinect/color/image_raw', Image)
        depth1_msg = rospy.wait_for_message('kinect/depth/image_raw', Image)
        print("================kinect obtained depth info====================")
        color1 = bridge.imgmsg_to_cv2(color1_msg, 'bgr8')
        depth1 = bridge.imgmsg_to_cv2(depth1_msg, 'passthrough')

        pcd1_msg = to_pcd(color1_msg, depth1_msg)
        pcd_pub.publish(pcd1_msg)  # Now we only use kinect
        rate.sleep()
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

class PCDPublisher():
    '''
    On receiving point cloud messages, merge them together
    Using message_filter for time synchronization
    '''
    def __init__(self):
        self.pcd = None
        self.pcd_pub = rospy.Publisher("/kinect/depth/pcd_from_depth", PointCloud2, queue_size=10)
        camera_sub = rospy.Subscriber()

    def pcd_publish_loop(self):
        while True:
            if self.pcd is not None:
                self.pcd_pub.publish(self.pcd)
            rospy.sleep(1.)
                
    def to_pcd(self):


if __name__ == '__main__':
    rospy.init_node('depth_to_pcd')
    pcdPublisher = PCDPublisher()
    rospy.spin()

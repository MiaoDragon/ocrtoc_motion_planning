import sys
import rospy
import cv2
import numpy as np
import open3d as o3d
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

if __name__=='__main__':

    rospy.init_node('image_receiver', anonymous=True)
    bridge = CvBridge()

    msg = rospy.wait_for_message('kinect/color/image_raw', Image)
    color = bridge.imgmsg_to_cv2(msg, 'bgr8')

    msg = rospy.wait_for_message('kinect/depth/image_raw', Image)
    depth = bridge.imgmsg_to_cv2(msg, 'passthrough')
    # depth = cv2.fastNlMeansDenoising(depth, None, 10, 7, 21)
    kernel_size = 11
    kernel = np.ones((kernel_size, kernel_size), np.float32) / kernel_size**2
    depth = cv2.filter2D(depth, -1, kernel)
    depth = (10000 * depth).astype(np.uint16)

    # cv2.imwrite('../data/sapien_images/color/%s_color.png'%scene, color)
    # cv2.imwrite('../data/sapien_images/depth/%s_depth.png'%scene, depth)


    cam_intrinsics = np.array([
        [1120.1199, 0, 640.5],
        [0, 1120.1199, 360.5],
        [0, 0, 1]
    ])
    color_o3d = o3d.geometry.Image(cv2.cvtColor(color, cv2.COLOR_BGR2RGB))
    depth_o3d = o3d.geometry.Image(depth)
    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(color_o3d, depth_o3d, depth_scale=10000, convert_rgb_to_intensity=False)
    intrinsics_o3d = o3d.camera.PinholeCameraIntrinsic()
    intrinsics_o3d.intrinsic_matrix = cam_intrinsics
    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd, intrinsics_o3d)
    o3d.visualization.draw_geometries([pcd])
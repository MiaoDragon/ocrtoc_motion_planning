#!/usr/bin/env python

from __future__ import print_function

from motion_planning.srv import OneShotGraspPlanWithObjectPose, OneShotGraspPlanWithObjectPoseResponse
from motion_planning_functions import one_shot_grasp_with_object_pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommandActionGoal
from motion_planning_execution import execute_plan_close_loop_with_pub, execute_plan_close_loop_with_moveit, gripper_openning

from depth_to_pcd_functions import *

import tf

import rospy

from cv_bridge import CvBridge
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import sensor_msgs.point_cloud2 as pcl2
import std_msgs.msg
from sensor_msgs.msg import PointCloud2, Image
import pyassimp
from motion_planning_utils import scale_pose_to_tf, clear_octomap

#-------------------------------
#* image related *
subsample_ratio = 0.2
bridge = CvBridge()
#controller_topic = rospy.get_param('controller_topic')

arm_cmd_topic = rospy.get_param('arm_cmd_topic')
gripper_cmd_topic = rospy.get_param('gripper_cmd_topic')



pcd_pub = rospy.Publisher("/kinect/depth/pcd_from_depth", PointCloud2, queue_size=10, latch=True)
tf_buffer = tf2_ros.Buffer()
tf_listener = None# tf2_ros.TransformListener(tf_buffer)
pcd_msg_to_pub = None
#* execution related *
arm_cmd_pub = rospy.Publisher(
    rospy.resolve_name(arm_cmd_topic),
    JointTrajectory, queue_size=10)
gripper_cmd_pub = rospy.Publisher(
    rospy.resolve_name(gripper_cmd_topic),
    GripperCommandActionGoal, queue_size=10)


def time_callback(timer):
    # get once the scene information
    # color1_msg = rospy.wait_for_message('kinect/color/image_raw', Image)
    # depth1_msg = rospy.wait_for_message('kinect/depth/image_raw', Image)
    # convert to pcd and publish
    # pcd1_msg = to_pcd(bridge, color1_msg, depth1_msg)
    if pcd_msg_to_pub is not None:
        pcd_pub.publish(pcd_msg_to_pub)
        #clear_octomap()
    print('fixed time published point cloud message...')

def obtain_object_mesh(model_name, scale):
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    #pyassimp_mesh = pyassimp.load(mesh_file_name)
    import trimesh
    trimesh_mesh = trimesh.load(mesh_file_name)
    # if not pyassimp_mesh.meshes:
    #     rospy.logerr('Unable to load mesh')
    #     sys.exit(1)
    # for face in pyassimp_mesh.meshes[0].faces:
    #     triangle = MeshTriangle()
    #     triangle.vertex_indices = [face[0],
    #                                face[1],
    #                                face[2]]
    #     mesh.triangles.append(triangle)
    # for vertex in pyassimp_mesh.meshes[0].vertices:
    #     point = Point()
    #     point.x = vertex[0]*scale[0]
    #     point.y = vertex[1]*scale[1]
    #     point.z = vertex[2]*scale[2]
    #     mesh.vertices.append(point)
    # for box filtering (notice that this is unscaled)
    min_xyz = np.array(trimesh_mesh.vertices).min(axis=0)
    max_xyz = np.array(trimesh_mesh.vertices).max(axis=0)
    # print('min_xyz: ')
    # print(min_xyz)
    # print('max_xyz: ')
    # print(max_xyz)
    #pyassimp.release(pyassimp_mesh)
    return min_xyz, max_xyz


def filter_object_pcd(pcd_msg, model_name, scale, obj_pose):
    # get the transformed pcd in world frame
    trans = tf_buffer.lookup_transform('world', pcd_msg.header.frame_id, rospy.Time(0), rospy.Duration(1))  # world <- camera

    t = np.array([trans.transform.translation.x,trans.transform.translation.y,trans.transform.translation.z])
    r = np.array([trans.transform.rotation.x,trans.transform.rotation.y,trans.transform.rotation.z,trans.transform.rotation.w])
    r = tf.transformations.euler_from_quaternion(r)
    T = tf.transformations.compose_matrix(translate=t, angles=r)


    pcd = list(pcl2.read_points(pcd_msg, field_names = ("x", "y", "z"), skip_nans=True))

    padding = 0.045
    min_xyz, max_xyz = obtain_object_mesh(model_name, scale)
    min_xyz -= padding
    max_xyz += padding

    filter_transform =  tf.transformations.inverse_matrix(scale_pose_to_tf(scale=scale, pose=obj_pose))  # obj <- world

    filter_transform = filter_transform.dot(T)  # obj <- camera

    pcd = np.array(pcd)
    pcd_add_axis = np.ones((len(pcd),4))
    pcd_add_axis[:,:3] = pcd
    pcd_transformed = filter_transform.dot(pcd_add_axis.T).T
    pcd_transformed = pcd_transformed[:,:3]
    box_mask_min = np.prod((pcd_transformed - min_xyz) >= 0, axis=1)  # AND across x-y-z axis
    box_mask_max = np.prod((max_xyz - pcd_transformed) >= 0, axis=1)  # AND across x-y-z axis
    box_mask = box_mask_min * box_mask_max  # should satisfy both min and max constraints to be inside the box
    # anything else stays

    # filter by the mask
    print('before filter: pcd length: %d' % (len(pcd)))
    pcd = pcd[box_mask == 0]  # anything not inside the box
    print('after filter: pcd length: %d' % (len(pcd)))

    header = std_msgs.msg.Header()
    #header.stamp = rospy.Time(0)
    header.frame_id = pcd_msg.header.frame_id
    #create pcl from points
    filtered_pcd = pcl2.create_cloud_xyz32(header, pcd)
    #publish
    return filtered_pcd

def publish_pcd_with_filter(model_name, scale, obj_pose):
    global pcd_msg_to_pub
    # get once the scene information
    color1_topic = rospy.get_param('color_topic')
    depth1_topic = rospy.get_param('depth_topic')
    color1_msg = rospy.wait_for_message(color1_topic, Image)
    depth1_msg = rospy.wait_for_message(depth1_topic, Image)

    # color1_msg = rospy.wait_for_message('/remapped/kinect/color/image_raw', Image)
    # depth1_msg = rospy.wait_for_message('/remapped/kinect/depth/image_raw', Image)

    #depth1_msg = rospy.wait_for_message('kinect/depth_to_rgb/image_raw', Image)

    # convert to pcd and publish
    pcd1_msg = to_pcd(bridge, color1_msg, depth1_msg, subsample_ratio=subsample_ratio)
    # filter the object
    pcd1_msg = filter_object_pcd(pcd1_msg, model_name, scale, obj_pose)
    pcd_pub.publish(pcd1_msg)

    pcd_msg_to_pub = pcd1_msg
    print('published point cloud message...')
    # wait for it to appear in motion planner
    rospy.sleep(1.0)

def publish_pcd():
    global pcd_msg_to_pub
    # get once the scene information
    color1_topic = rospy.get_param('color_topic')
    depth1_topic = rospy.get_param('depth_topic')
    color1_msg = rospy.wait_for_message(color1_topic, Image)
    depth1_msg = rospy.wait_for_message(depth1_topic, Image)
    # color1_msg = rospy.wait_for_message('/remapped/kinect/color/image_raw', Image)
    # depth1_msg = rospy.wait_for_message('/remapped/kinect/depth/image_raw', Image)
    #depth1_msg = rospy.wait_for_message('kinect/depth_to_rgb/image_raw', Image)

    # convert to pcd and publish
    pcd1_msg = to_pcd(bridge, color1_msg, depth1_msg, subsample_ratio=subsample_ratio)

    pcd_pub.publish(pcd1_msg)
    pcd_msg_to_pub = pcd1_msg

    print('published point cloud message...')
    # wait for it to appear in motion planner
    rospy.sleep(1.0)

def handle_grasp_plan(req):
    # open gripper before plan
    gripper_openning(gripper_cmd_pub)
    clear_octomap()  # clear the previous octomap, and listen to new scene change
    try:
        publish_pcd_with_filter(req.object_name, req.object_scale, req.object_pose1)
    except:
        rospy.logerr("Motion Planning filtering: object mesh model is not found. Not filtering point cloud.")
        publish_pcd()
    #publish_pcd()

    #hello = raw_input("after pcd with filter...\n")
    #rospy.sleep(1)
    #clear_octomap()
    rospy.sleep(2)  # wait long enough to get the octomap

    response = OneShotGraspPlanWithObjectPoseResponse()
    # start plannning
    #plan_res = one_shot_grasp_with_object_pose(req.object_name, req.object_scale, req.object_pose1, req.object_pose2)
    try:
        plan_res = one_shot_grasp_with_object_pose(req.object_name, req.object_scale, req.object_pose1, req.object_pose2)
    except:
        rospy.logerr("Grasp plan failed.")
        # plan is unsuccessful at some point
        response.result = response.FAILURE
    else:
        # plan is successful
        rospy.loginfo("Grasp plan successfully generated.")
        response.pre_grasp_trajectory = plan_res['pre_grasp_trajectory']
        response.pre_to_grasp_trajectory = plan_res['pre_to_grasp_trajectory']
        response.post_grasp_trajectory = plan_res['post_grasp_trajectory']
        response.place_trajectory = plan_res['place_trajectory']
        response.post_place_trajectory = plan_res['post_place_trajectory']
        response.reset_trajectory = plan_res['reset_trajectory']

        response.result = response.SUCCESS
        rospy.loginfo("Start executing grasp plan...")
        exp_stage = rospy.get_param('stage')
        gripper_success = execute_plan_close_loop_with_pub(arm_cmd_pub, gripper_cmd_pub, plan_res)

        # if exp_stage == 'sim':
        #     print('in sim stage, using publisher...')
        #     gripper_success = execute_plan_close_loop_with_pub(arm_cmd_pub, gripper_cmd_pub, plan_res)
        # else:
        #     print('in real stage, using moveit...')
        #     gripper_success = execute_plan_close_loop_with_moveit(arm_cmd_pub, gripper_cmd_pub, plan_res)


        rospy.loginfo("motion planning: end of execution.")
        # publish update to map
        publish_pcd()
        if not gripper_success:
            response.result = response.FAILURE

    return response

def motion_planning_server():
    global tf_listener
    rospy.init_node('one_shot_grasp_with_object_pose_server')
    # hot start to wait for octomap
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    rospy.sleep(2.)

    publish_pcd()

    #timer = rospy.Timer(rospy.Duration(5), time_callback)
    s = rospy.Service('/motion_planning/one_shot_grasp_with_object_pose', OneShotGraspPlanWithObjectPose, handle_grasp_plan)

    print("Ready for grasp plan.")
    rospy.spin()

if __name__ == "__main__":
    motion_planning_server()

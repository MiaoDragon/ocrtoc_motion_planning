#! /usr/bin/env python

"""
motion planning components:

specify several functions to be used for motion planning

"""

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point

import numpy as np
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
#from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf
from control_msgs.msg import GripperCommandActionGoal

from motion_planning_utils import *  # tool box
import motion_planning_execution
from motion_planning.msg import PointCloudFilterAction

import moveit_connection
robot = moveit_connection.robot
group = moveit_connection.group
scene = moveit_connection.scene

gripper_open_value = motion_planning_execution.gripper_open_value
gripper_close_value = motion_planning_execution.gripper_close_value
gripper_driver_name = motion_planning_execution.gripper_driver_name

def filter_grasp_pose(global_grasp_pose, obj_pose1, obj_pose2):
    #* sort the generated grasp proposals given the score
    #* IK and collision check using pre_grasp_pose, and grasp_pose (we use initial solution from last IK)
    # for both, we allow retreat up to some point
    retreat_num_step = 10
    last_valid_state = None  # we use this for init IK
    
    found_grasp_pose_i = -1
    found_pre_grasp_pose = None
    found_grasp_pose = None
    found_target_pose = None
    print('length of pre_grasp_pose: %d' % (len(global_grasp_pose.pre_grasp_poses)))
    for grasp_pose_i in range(len(global_grasp_pose.pre_grasp_poses)):
        pre_grasp_pose = global_grasp_pose.pre_grasp_poses[grasp_pose_i]
        grasp_pose = global_grasp_pose.grasp_poses[grasp_pose_i]
        # calculate retreat_vector
        pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
        grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
        pre_grasp_retreat_vec = retreat_vec_calculation(pre_grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))
        grasp_retreat_vec = retreat_vec_calculation(grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))

        current_retreat_step = 0.
        # find pre_grasp_pose
        retreat_step_size = 0.01
        for grasp_verify_i in range(retreat_num_step):
            # check for pregrasp pose
            current_retreat_step = retreat_step_size * grasp_verify_i
            # obtain the retreated grasping pose
            current_pose = pre_grasp_pose_np + current_retreat_step * pre_grasp_retreat_vec
            pre_grasp_pose.position.x = current_pose[0]
            pre_grasp_pose.position.y = current_pose[1]
            pre_grasp_pose.position.z = current_pose[2]
            # verify if the pose is valid by IK and Collision Check
            last_valid_state, robot_state, pre_grasp_status = verify_pose(pre_grasp_pose, init_robot_state=last_valid_state)
            if pre_grasp_status:
                break
        if not pre_grasp_status:
            # try another grasp pose
            continue
        # otherwise the found one ise pre_grasp_pose
        print('found pre_grasp_pose')
        # find grasp_pose, smaller retreat step
        retreat_step_size = 0.008
        for grasp_verify_i in range(retreat_num_step):            
            # check for grasp pose
            current_retreat_step = retreat_step_size * grasp_verify_i
            # obtain the retreated grasping pose
            current_pose = grasp_pose_np + current_retreat_step * grasp_retreat_vec
            grasp_pose.position.x = current_pose[0]
            grasp_pose.position.y = current_pose[1]
            grasp_pose.position.z = current_pose[2]
            # verify if the pose is valid by IK and Collision Check
            last_valid_state, robot_state, grasp_status = verify_pose(grasp_pose, init_robot_state=last_valid_state)
            if grasp_status:
                break
        if not grasp_status:
            # try another grasp pose
            continue
        print('found pre_grasp_pose and grasp_pose')
        # obtain target arm pose based on grasp pose and object poses
        target_pose = grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2, grasp_pose)
        # check if target_pose is in collision
        _, _, target_status = verify_pose(target_pose, init_robot_state=None)
        # if all succeeded, then break
        if target_status:
            found_grasp_pose_i = grasp_pose_i
            found_pre_grasp_pose = pre_grasp_pose
            found_grasp_pose = grasp_pose
            found_target_pose = target_pose
            break
    if found_grasp_pose_i == -1:
        # all grasp positions failed, error
        rospy.logerr("generated grasp poses are invalid (in collision or not valid IK).")
        sys.exit(1)

    return found_pre_grasp_pose, found_grasp_pose, found_target_pose, found_grasp_pose_i


def pre_grasp(start_state, target_pose):
    x = group.get_current_pose()
    x.pose = target_pose
    last_valid_state = None

    for planning_attempt_i in range(10):
        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            #print('plan is successful')
            pre_grasp_plan = plan
            break
        else:
            pre_grasp_plan = None
    return pre_grasp_plan


def straight_line_move(start_state, start_pose, target_pose):
    # firstly generate list of waypoints from start_pose to target_pose, until collision happens
    # use around 20 points in between
    _, robot_state, error_code = ik(start_pose, collision=False, init_robot_state=start_state)
    num_waypoints = 20
    total_duration = 2.  # we want 2s to go to the desired location
    time_step = total_duration / num_waypoints
    start_pose_np = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    target_pose_np = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    step = (target_pose_np - start_pose_np) / num_waypoints
    waypoint_list = []
    straight_line_trajectory = JointTrajectory()
    straight_line_trajectory.header = Header()
    #straight_line_trajectory.header.stamp = rospy.Time.now()
    straight_line_trajectory.header.frame_id = group.get_planning_frame()
    straight_line_trajectory.joint_names = start_state.joint_state.name[:6]  # extract arm
    last_valid_state = start_state
    for i in range(1,num_waypoints+1):
        inter_pose_np = start_pose_np + step * i
        inter_pose = copy.deepcopy(start_pose)
        inter_pose.position.x = inter_pose_np[0]
        inter_pose.position.y = inter_pose_np[1]
        inter_pose.position.z = inter_pose_np[2]
        _, robot_state, error_code = ik(inter_pose, collision=True, init_robot_state=last_valid_state)
        if error_code.val == 1:
            # if IK is successful
            # append robot_state into trajectory
            inter_joint_traj_point = JointTrajectoryPoint()
            inter_joint_traj_point.positions = robot_state.joint_state.position[:6]  # filter out the gripper joints
            inter_joint_traj_point.velocities = robot_state.joint_state.velocity[:6]
            #inter_joint_traj_point.accelerations = 0.
            inter_joint_traj_point.effort = robot_state.joint_state.effort[:6]
            inter_joint_traj_point.time_from_start = rospy.Duration.from_sec(time_step * i)
            straight_line_trajectory.points.append(inter_joint_traj_point)
            last_valid_state = robot_state
    return straight_line_trajectory


from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point
import pyassimp 
def attach_object_to_gripper(model_name, scale, obj_pose, grasp_state):
    # achieve this by modifying the start robot state
    # MoveIt also uses this from http://docs.ros.org/api/moveit_commander/html/planning__scene__interface_8py_source.html
    attached_obj = AttachedCollisionObject()
    attached_obj.link_name = "robotiq_2f_85_left_pad"
    touch_links = ["robotiq_2f_85_left_pad", "robotiq_2f_85_right_pad", \
                   "robotiq_2f_85_left_spring_link", "robotiq_2f_85_right_spring_link", \
                   "robotiq_2f_85_left_follower", "robotiq_2f_85_right_follower", \
                   "robotiq_2f_85_left_driver", "robotiq_2f_85_right_driver", \
                   "robotiq_2f_85_left_coupler", "robotiq_2f_85_right_coupler", \
                   "robotiq_arg2f_base_link", "robotiq_2f_85_base", \
                   "robotiq_ur_coupler", "tool0", \
                   "robotiq_arg2f_base_link", "realsense_camera_link", \
                   "table"]
    attached_obj.touch_links = touch_links
    # specify the attached object shape
    attached_obj_shape = CollisionObject()
    attached_obj_shape.id = model_name
    mesh = Mesh()
    # load mesh file
    #pyassimp_mesh = pyassimp.load("/root/ocrtoc_ws/src/ocrtoc_motion_planning/data/"+model_name+"/visual_meshes/cloud.ply")
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    pyassimp_mesh = pyassimp.load(mesh_file_name)

    if not pyassimp_mesh.meshes:
        rospy.logerr('Unable to load mesh')
        sys.exit(1)
    for face in pyassimp_mesh.meshes[0].faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = [face[0],
                                   face[1],
                                   face[2]]
        mesh.triangles.append(triangle)
    for vertex in pyassimp_mesh.meshes[0].vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)
    
    # for box filtering (notice that this is unscaled)
    min_xyz = np.array(pyassimp_mesh.meshes[0].vertices).min(axis=0)
    max_xyz = np.array(pyassimp_mesh.meshes[0].vertices).max(axis=0)

    attached_obj_shape.meshes = [mesh]
    attached_obj_shape.mesh_poses = [obj_pose]
    attached_obj_shape.operation = CollisionObject.ADD
    attached_obj_shape.header = group.get_current_pose().header
    pyassimp.release(pyassimp_mesh)
    attached_obj.object = attached_obj_shape
    grasp_state.attached_collision_objects.append(attached_obj)
    grasp_state.is_diff = True  # is different from others since we attached the object

    """
    # Bellow will cause the start-state to be in collision, even after updating the octomap
    # set the gripper state to be close
    found_gripper_driver = False
    for i in range(len(grasp_state.joint_state.name)):
        if grasp_state.joint_state.name[i] == gripper_driver_name:
            grasp_state.joint_state.position[i] = 0.5*gripper_close_value
            found_gripper_driver = True
            break
    if not found_gripper_driver:
        # add
        grasp_state.joint_state.name.append(gripper_driver_name)
        grasp_state.joint_state.position = list(grasp_state.joint_state.position) + [0.5*gripper_close_value]
    """

    # start filter
    """
    filter_action = PointCloudFilterAction()
    filter_action.action = filter_action.StartFilter
    # obtain filter parameters
    pcd_transform = tf.transformations.inverse_matrix(scale_pose_to_tf(scale=scale, pose=obj_pose))
    filter_action.min_xyz = min_xyz
    filter_action.max_xyz = max_xyz
    filter_action.pcd_transform = pcd_transform.flatten()
    pcd_filter_action_pub = rospy.Publisher(
        rospy.resolve_name('/motion_planning/pcd_filter_action'),
        PointCloudFilterAction, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    pcd_filter_action_pub.publish(filter_action)
    print("Publishing filter action...")
    rospy.sleep(1.0)
    """
    # update octomap by clearing existing ones
    clear_octomap()

    # publish the robot state for visualization
    #display_robot_state(grasp_state)
    #hello = raw_input("end of attaching object. Please input...\n")
    rospy.sleep(1.0) # allow publisher to initialize
    return grasp_state

def place(start_state, target_pose):
    group.clear_pose_targets()
    # verify if the target pose is collision free
    # status = verify_pose(target_pose)
    # if not status:
    #     place_plan = None
    #     print('Goal pose is not valid. Another attempt is tried...')
    #     continue
    # plan
    z_padding_limit = 0.2
    #target_pose.position.z += 0.1  # add padding
    target_pose = copy.deepcopy(target_pose)
    max_attempt = 10
    z_padding_step = z_padding_limit / max_attempt
    for planning_attempt_i in range(max_attempt):
        group.set_start_state(start_state)
        target_pose.position.z += z_padding_step
        group.set_pose_target(target_pose)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            place_plan = plan
            break
        else:
            place_plan = None
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0
    return place_plan



def arm_reset(start_state):
    # plan a path to reset position (up)
    group.clear_pose_targets()
    [-1.57, -1.57, 1.57, 0, 0, 0]
    # up: [0, -1.57,0, -1.57, 0, 0]
    up_joints = {"elbow_joint": -1.57, "shoulder_lift_joint": -1.577, "shoulder_pan_joint": 1.57, 
                 "wrist_1_joint": 0., "wrist_2_joint": 0., "wrist_3_joint": 0.}
    for planning_attempt_i in range(10):
        group.set_start_state(start_state)
        group.set_joint_value_target(up_joints)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            reset_plan = plan
            break
        else:
            reset_plan = None
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0
    return reset_plan

def one_shot_grasp_with_object_pose(model_name, scale, obj_pose1, obj_pose2):
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    <-- execute gripper closing -->
    4. Attach the object to the gripper. plan a path to drop pose
    <-- execute gipper openning -->

    input:
    ===========================================
    model_name, obj_pose1, obj_pose2

    format: geometry_msgs/Pose

    output:
    ===========================================
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory

    format: geometry_msgs/Pose
    """
    print('end effector:')
    print(group.get_end_effector_link())
    #########################################
    # reset the planning scene
    clear_octomap()
    print('get object:')
    print(scene.get_objects())
    scene.remove_world_object(name=model_name)  # clean scene afterwards
    scene.remove_world_object()
    rospy.sleep(1.0)
    # stop filter
    """
    from motion_planning.msg import PointCloudFilterAction
    filter_action = PointCloudFilterAction()
    filter_action.action = filter_action.StopFilter
    pcd_filter_action_pub = rospy.Publisher(
        rospy.resolve_name('/motion_planning/pcd_filter_action'),
        PointCloudFilterAction, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    pcd_filter_action_pub.publish(filter_action)
    print("Publishing filter action...")
    rospy.sleep(1.0)
    """

    #########################################

    #** stage 1: generate grasp pose proposals **
    from grasp_srv.msg import ObjectPoses, Grasps
    from grasp_srv.srv import GraspGen, GraspGenResponse
    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    grasp_srv_request_poses.object_scales.append(scale[0])
    grasp_srv_request_poses.object_poses.append(obj_pose1)
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        resp1 = grasp_gen(grasp_srv_request_poses)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

    #* select pose for use based on score *
    #* if the grasp_pose has length 0, then abort
    if len(resp1.grasps.global_grasp_poses) == 0:
        rospy.logerr("grasp_pose generation failed.")
        sys.exit(1)
    if len(resp1.grasps.global_grasp_poses[0].grasp_poses) == 0 or len(resp1.grasps.global_grasp_poses[0].pre_grasp_poses) == 0:
        respy.logerr("grasp_pose generation failed.")
        sys.exit(1)

    global_grasp_pose = resp1.grasps.global_grasp_poses[0]

    pre_grasp_pose, grasp_pose, target_pose, grasp_pose_i = filter_grasp_pose(global_grasp_pose, obj_pose1, obj_pose2)
    # ***
    # Note: Here we TRUST the grasp poses generated by GraspGen algorithm,
    # and use the score it provides to decide the graspability of objects.
    # Hence we just go over the sorted grasp poses, and try until we find
    # a valid pose.
    # ***

    #** stage 1.5: precondition check for motion plan **
    # ***
    # Note: use the start grasp_pose and goal grasp_pose to do collision checking.
    # TODO: should we attach the object for checking at goal?
    # ***


    # TODO: check if target pose is collision-free

    ## TODO: search if we can do multi-goal planning
    print("============ Printing generated pre-grasp pose")
    print(pre_grasp_pose)

    # we assume the gripper is open
    group.clear_pose_targets()

    #** stage 2: generate pre-grasp plan **
    rospy.loginfo("generating pre-grasp plan...")
    print('============ move arm...')
    #display_robot_state(robot.get_current_state())
    #raw_input("display input...")
    pre_grasp_plan = pre_grasp(robot.get_current_state(), pre_grasp_pose)
    
    pre_grasp_trajectory = pre_grasp_plan.joint_trajectory
    # remember the end state to be used for next stage
    pre_grasp_plan_end_state = robot_state_from_plan(pre_grasp_plan)

    #** stage 3: plan cartesian path to grasp pose **
    # directly using IK to achieve this part
    pre_to_grasp_trajectory = straight_line_move(pre_grasp_plan_end_state, pre_grasp_pose, grasp_pose)
    grasp_plan_end_state = robot_state_from_joint_trajectory(pre_to_grasp_trajectory)
    #** stage 4: attach the object to the gripper. plan the place trajectory to move the arm **
    #** attach object to gripper **
    grasp_plan_end_state = attach_object_to_gripper(model_name, scale, obj_pose1, grasp_plan_end_state)


    #** plan **
    place_plan = place(start_state=grasp_plan_end_state, target_pose=target_pose)
    place_trajectory = place_plan.joint_trajectory
    
    # remove the object from the planning scene
    scene.remove_attached_object(link="robotiq_2f_85_left_pad", name=model_name)
    # remove filter
    """
    filter_action = PointCloudFilterAction()
    filter_action.action = filter_action.StopFilter
    rospy.sleep(1.0) # allow publisher to initialize
    pcd_filter_action_pub.publish(filter_action)
    print("Publishing filter action...")
    rospy.sleep(1.0)
    """
    # add object target_pose to the scene, because we don't want to collide with the object
    # or use some other ways, like straight-line up
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose2
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0], scale[1], scale[2]))
    reset_plan = arm_reset(start_state=robot_state_from_plan(place_plan))
    reset_trajectory = reset_plan.joint_trajectory
    scene.remove_world_object(name=model_name)  # clean scene afterwards
    return pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory




def one_shot_grasp_with_object_pose_close_loop(model_name, scale, obj_pose1, obj_pose2):
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    <-- execute gripper closing -->
    4. Attach the object to the gripper. plan a path to drop pose
    <-- execute gipper openning -->

    input:
    ===========================================
    model_name, obj_pose1, obj_pose2

    format: geometry_msgs/Pose

    output:
    ===========================================
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory

    format: geometry_msgs/Pose
    """
    print('end effector:')
    print(group.get_end_effector_link())
    #########################################
    # reset the planning scene
    clear_octomap()
    print('get object:')
    print(scene.get_objects())
    scene.remove_world_object(name=model_name)  # clean scene afterwards
    scene.remove_world_object()
    rospy.sleep(1.0)
    # stop filter
    """
    from motion_planning.msg import PointCloudFilterAction
    filter_action = PointCloudFilterAction()
    filter_action.action = filter_action.StopFilter
    pcd_filter_action_pub = rospy.Publisher(
        rospy.resolve_name('/motion_planning/pcd_filter_action'),
        PointCloudFilterAction, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    pcd_filter_action_pub.publish(filter_action)
    print("Publishing filter action...")
    rospy.sleep(1.0)
    """

    #########################################

    #** stage 1: generate grasp pose proposals **
    from grasp_srv.msg import ObjectPoses, Grasps
    from grasp_srv.srv import GraspGen, GraspGenResponse
    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    grasp_srv_request_poses.object_scales.append(scale[0])
    grasp_srv_request_poses.object_poses.append(obj_pose1)
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        resp1 = grasp_gen(grasp_srv_request_poses)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)

    #* select pose for use based on score *
    #* if the grasp_pose has length 0, then abort
    if len(resp1.grasps.global_grasp_poses) == 0:
        rospy.logerr("grasp_pose generation failed.")
        sys.exit(1)
    if len(resp1.grasps.global_grasp_poses[0].grasp_poses) == 0 or len(resp1.grasps.global_grasp_poses[0].pre_grasp_poses) == 0:
        respy.logerr("grasp_pose generation failed.")
        sys.exit(1)

    global_grasp_pose = resp1.grasps.global_grasp_poses[0]

    pre_grasp_pose, grasp_pose, target_pose, grasp_pose_i = filter_grasp_pose(global_grasp_pose, obj_pose1, obj_pose2)
    # ***
    # Note: Here we TRUST the grasp poses generated by GraspGen algorithm,
    # and use the score it provides to decide the graspability of objects.
    # Hence we just go over the sorted grasp poses, and try until we find
    # a valid pose.
    # ***

    #** stage 1.5: precondition check for motion plan **
    # ***
    # Note: use the start grasp_pose and goal grasp_pose to do collision checking.
    # TODO: should we attach the object for checking at goal?
    # ***


    # TODO: check if target pose is collision-free

    ## TODO: search if we can do multi-goal planning
    print("============ Printing generated pre-grasp pose")
    print(pre_grasp_pose)

    # we assume the gripper is open
    group.clear_pose_targets()

    #** stage 2: generate pre-grasp plan **
    rospy.loginfo("generating pre-grasp plan...")
    print('============ move arm...')
    #display_robot_state(robot.get_current_state())
    #raw_input("display input...")
    pre_grasp_plan = pre_grasp(robot.get_current_state(), pre_grasp_pose)
    
    pre_grasp_trajectory = pre_grasp_plan.joint_trajectory
    # remember the end state to be used for next stage
    pre_grasp_plan_end_state = robot_state_from_plan(pre_grasp_plan)

    #** stage 3: plan cartesian path to grasp pose **
    # directly using IK to achieve this part
    pre_to_grasp_trajectory = straight_line_move(pre_grasp_plan_end_state, pre_grasp_pose, grasp_pose)
    grasp_plan_end_state = robot_state_from_joint_trajectory(pre_to_grasp_trajectory)
    #** stage 4: attach the object to the gripper. plan the place trajectory to move the arm **
    #** attach object to gripper **
    grasp_plan_end_state = attach_object_to_gripper(model_name, scale, obj_pose1, grasp_plan_end_state)

    #** plan **
    place_plan = place(start_state=grasp_plan_end_state, target_pose=target_pose)
    place_trajectory = place_plan.joint_trajectory
    
    # remove the object from the planning scene
    scene.remove_attached_object(link="robotiq_2f_85_left_pad", name=model_name)
    # remove filter
    """
    filter_action = PointCloudFilterAction()
    filter_action.action = filter_action.StopFilter
    rospy.sleep(1.0) # allow publisher to initialize
    pcd_filter_action_pub.publish(filter_action)
    print("Publishing filter action...")
    rospy.sleep(1.0)
    """
    # add object target_pose to the scene, because we don't want to collide with the object
    # or use some other ways, like straight-line up
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose2
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0], scale[1], scale[2]))
    reset_plan = arm_reset(start_state=robot_state_from_plan(place_plan))
    reset_trajectory = reset_plan.joint_trajectory
    scene.remove_world_object(name=model_name)  # clean scene afterwards
    return pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory, global_grasp_pose, grasp_pose_i



#======================================================================================================

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_test',
                    anonymous=True)
    import json
    scene_pose = open('../scene_info/1-1.json', 'r')
    scene_pose = json.load(scene_pose)
    # jenga
    #data_str = '-0.0599994957447052 -0.31989747285842896 0.014999995008111 2.261107306891991e-09 -1.8113155351356218e-09 -6.071384286259734e-09'
    # potted_meat_can
    #model_name = 'potted_meat_can'
    #data_str = '0.15989847481250763 0.29000547528266907 0.04212800785899162 -4.840692449249814e-08 -4.5454357156415357e-07 -6.927437318496676e-08'

    # master chef can
    #model_name = 'master_chef_can'
    #data_str = '-0.03987777605652809 0.32008427381515503 0.03530200198292732 -1.4431422264283453e-06 -7.782947982250227e-07 -3.5607089180211774e-07'

    # wood_block
    model_name = 'wood_block'
    data_str = '0.09994173049926758 -0.31943440437316895 0.022629257291555405 -0.005208467812430025 0.005718712075542679 0.005303920438713571'
    scale = np.array([0.5,0.5,0.5])
    # pudding_box
    #model_name = 'pudding_box'
    #data_str = '0.25974133610725403 -0.3102521002292633 0.009862901642918587 0.022348699698035056 -0.01139162625176719 -9.36981416203137e-05'

    # find the model_name in the scene_pose
    for i in range(len(scene_pose)):
        if scene_pose[i]['name'] == model_name:
            # record the pose
            print(scene_pose[i])
            start_tf = np.array(scene_pose[i]['pose_world'])
            _, shear, angles, trans, persp = tf.transformations.decompose_matrix(start_tf)
            break
    #print(start_tf)
    #print(scale)

    # parsing the string
    data = data_str.split()
    for i in range(len(data)):
        data[i] = float(data[i])

    data = np.array(data)
    position = data[:3]
    orientation = data[3:]

    # compose into TF matrix
    target_tf = tf.transformations.compose_matrix(scale=scale, shear=shear, angles=orientation, translate=position, perspective=persp)
    scale, start_pose = tf_to_scale_pose(start_tf)
    scale, target_pose = tf_to_scale_pose(target_tf)

    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory, global_grasp_pose, grasp_pose_i = one_shot_grasp_with_object_pose_close_loop(model_name, scale, start_pose, target_pose)
    motion_planning_execution.execute_plan(pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory, model_name, global_grasp_pose, grasp_pose_i)

if __name__ == "__main__":
    main()

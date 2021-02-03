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
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
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

from moveit_msgs.msg import CollisionObject

co_pub = rospy.Publisher('/collision_object', CollisionObject, queue_size=100)


gripper_open_value = motion_planning_execution.gripper_open_value
gripper_close_value = motion_planning_execution.gripper_close_value
gripper_driver_name = motion_planning_execution.gripper_driver_name

scale_padding = 1.0


# symmetric information of each model
import pandas as pd
import rospkg

rospack = rospkg.RosPack()
pkg_path = rospack.get_path('motion_planning')
symmetry_df = pd.read_csv(pkg_path+'/data/symmetry.csv')
symmetry_df = symmetry_df.set_index(symmetry_df['model_name'])
# remove rubric
#symmetry_df = symmetry_df.drop(index='rubiks_cube')


# keys: symmetry (symmetry w.r.t. any rotation), symmetry_180 (symmetry after 180 degrees rotation),
#       asymmetry (not symmetric)

def pose_precondition_check(pose, pos_np, retreat_vec, init_robot_state, retreat_step, collision=False, attach_object=None):
    """
    the returned robot_state won't contain the attached object
    """
    pose = copy.deepcopy(pose)
    current_pose = pos_np + retreat_step * retreat_vec
    pose.position.x = current_pose[0]
    pose.position.y = current_pose[1]
    pose.position.z = current_pose[2]
    # verify if the pose is valid by IK
    init_robot_state, robot_state, status = \
        ik(pose, collision=collision, init_robot_state=init_robot_state)

    #init_robot_state_2 = copy.deepcopy(init_robot_state)
    # vis_pose = copy.deepcopy(pose)
    # down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
    # vis_pose.orientation.x = down_quat[0]
    # vis_pose.orientation.y = down_quat[1]
    # vis_pose.orientation.z = down_quat[2]
    # vis_pose.orientation.w = down_quat[3]

    #_, dis_robot_state, dis_status = \
    #    ik(pose, collision=False, init_robot_state=init_robot_state_2)
    #print('display status: %d' % (dis_status))
    #display_robot_state(dis_robot_state, block=True)

    if not status:
        rospy.logwarn('precondition check for the current pose failed.')
        return pose, status, init_robot_state

    # if checking for attach_object, then use collision cehcking service
    if attach_object is not None:
        #rospy.logwarn('checking for attached object...')
        cc_robot_state = copy.deepcopy(robot_state)
        cc_robot_state.attached_collision_objects = list(cc_robot_state.attached_collision_objects) + [attach_object]
        cc_robot_state.is_diff = False
        # check collision
        cc_valid = collision_check_with_model(cc_robot_state, cc_robot_state.attached_collision_objects[0].object.id, group_name="robot_arm")
        #rospy.logwarn('after attaching object, the robot state is: %d' % (cc_valid))
        status = status and cc_valid
        #display_robot_state(cc_robot_state)

    if not status:
        rospy.logwarn('precondition check for the current pose failed.')
    return pose, status, init_robot_state


def binary_search_position(start_retreat_step, end_retreat_step, retreat_num_step, retreat_threshold, \
                            pose, pos_np, retreat_vec, init_robot_state, collision=False, attach_object=None):
    # use binary search to find the first pose between these two that is collision-free
    # assume: start_pose is in-collision or near-collision   end_pose is collision-free
    # make sure the end place is collision-free
    status = False
    for verify_i in range(retreat_num_step):
        # compute middle point in [start_retreat, end_retreat], where start_retreat is
        # guaranteed in collision.
        # until start_retreat - end_retreat <= retreat_threshold
        #           AND end_retreat not in collision
        print('verify iteration: %d' %(verify_i))
        print('window: [%f, %f]' % (start_retreat_step, end_retreat_step))
        print('window size: %f' % (np.abs(start_retreat_step - end_retreat_step)))

        if np.abs(start_retreat_step - end_retreat_step) <= retreat_threshold:
            print('found the pose, retreat_step: %f' % (end_retreat_step))
            # found
            status = True                # obtain the retreated grasping pose
            current_pose = pos_np + end_retreat_step * retreat_vec
            pose.position.x = current_pose[0]
            pose.position.y = current_pose[1]
            pose.position.z = current_pose[2]
            break
        # test mid-point
        mid_retreat_step = (start_retreat_step + end_retreat_step) / 2
        print('testing mid_retreat_step: %f' % (mid_retreat_step))
        # obtain the retreated grasping pose
        current_pose = pos_np + mid_retreat_step * retreat_vec
        pose.position.x = current_pose[0]
        pose.position.y = current_pose[1]
        pose.position.z = current_pose[2]
        # assume the first IK success
        init_robot_state, robot_state, mid_status = \
            ik(pose, collision=collision, init_robot_state=init_robot_state)

        # check for attached object
        if mid_status and attach_object is not None:
            rospy.logwarn('checking for attached object...')
            cc_robot_state = copy.deepcopy(robot_state)
            # modify the pose of the attached object based on the retreat
            current_attach_object = copy.deepcopy(attach_object)
            current_attach_obj_pose = current_attach_object.object.mesh_poses[0]
            current_attach_obj_pose_np = np.array([current_attach_obj_pose.position.x,current_attach_obj_pose.position.y,current_attach_obj_pose.position.z])
            current_attach_obj_pose_np += mid_retreat_step * retreat_vec
            current_attach_obj_pose.position.x = current_attach_obj_pose_np[0]
            current_attach_obj_pose.position.y = current_attach_obj_pose_np[1]
            current_attach_obj_pose.position.z = current_attach_obj_pose_np[2]
            current_attach_object.object.mesh_poses = [current_attach_obj_pose]
            cc_robot_state.attached_collision_objects = [current_attach_object]
            cc_robot_state.is_diff = False
            # check collision
            cc_valid = collision_check_with_model(cc_robot_state, cc_robot_state.attached_collision_objects[0].object.id, group_name="robot_arm")
            print('after attaching object, the robot state is: %d' % (cc_valid))
            mid_status = mid_status and cc_valid
            #display_robot_state(cc_robot_state)

        # display
        # init_robot_state_2 = copy.deepcopy(init_robot_state)
        # _, dis_robot_state, dis_status = \
        #     ik(pose, collision=False, init_robot_state=init_robot_state_2)
        # print('display status: %d' % (dis_status))
        # display_robot_state(dis_robot_state)


        print('after IK: status: %d' % (mid_status))
        if mid_status:
            # if collision-free, set as end_retreat_step
            end_retreat_step = mid_retreat_step
            print('IK valid, set to end_retreat_step')

        else:
            start_retreat_step = mid_retreat_step
            print('IK invalid, set to start_retreat_step')
    return pose, status, init_robot_state


def motion_planning_precondition_check(global_grasp_pose, obj_pose1, obj_pose2, scale, model_name):
    # database storeing failed grasp_pose, used to determine if near pose is likely to fail too
    fail_grasp_pose = []
    near_fail_grasp_pose = []  # we won't put the pose filtered by pos and ori inside failed list
    fail_pos_threshold = 0.015
    fail_ori_threshold = 20 # 10 degrees

    found_grasp_pose_i = -1
    found_pre_grasp_pose = None
    found_grasp_pose = None
    found_target_pose = None
    # *for pre-grasp-pose, we check against the point cloud
    # *for grasp-pose, we check against table and object mesh model
    # spawn the object model according to obj_pose1
    pre_grasp_last_valid_state = None
    grasp_last_valid_state_without_attach = None
    target_pose_last_valid_state = None

    print('length of pre_grasp_pose: %d' % (len(global_grasp_pose.pre_grasp_poses)))


    # * check if target pose if in collision *
    # check collision at the target location for the object vs. environemnt
    # if colliding, then directly exit
    target_end_retreat_step = 0.04
    target_retreat_vec = np.array([0.,0,1])

    obj_pose2_retreated = copy.deepcopy(obj_pose2)
    obj_pose2_retreated_np = np.array([obj_pose2_retreated.position.x,obj_pose2_retreated.position.y,obj_pose2_retreated.position.z])
    obj_pose2_retreated_np = obj_pose2_retreated_np + target_retreat_vec * target_end_retreat_step
    obj_pose2_retreated.position.x = obj_pose2_retreated_np[0]
    obj_pose2_retreated.position.y = obj_pose2_retreated_np[1]
    obj_pose2_retreated.position.z = obj_pose2_retreated_np[2]
    try:
        attached_obj = obtain_attach_object(model_name, scale, obj_pose2_retreated)
        attached_obj.object.operation = attached_obj.object.ADD # we only want to change the pose
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
        attached_obj = None

    cc_robot_state = robot.get_current_state()
    cc_robot_state = copy.deepcopy(cc_robot_state)
    if attached_obj is not None:
        cc_robot_state.attached_collision_objects = list(cc_robot_state.attached_collision_objects) + [attached_obj]
    cc_robot_state.is_diff = False
    # check collision
    cc_result = get_state_validity(cc_robot_state, group_name="robot_arm")
    # check if there exists model vs. octomap OR model vs. table collision
    if not cc_result.valid:
        for i in range(len(cc_result.contacts)):
            failed = False
            if cc_result.contacts[i].contact_body_1 == model_name and cc_result.contacts[i].body_type_2 == 1:
                failed = True
            if cc_result.contacts[i].contact_body_2 == model_name and cc_result.contacts[i].body_type_1 == 1:
                failed = True
            if cc_result.contacts[i].contact_body_1 == model_name and cc_result.contacts[i].contact_body_2 == 'table':
                failed = True
            if cc_result.contacts[i].contact_body_2 == model_name and cc_result.contacts[i].contact_body_1 == 'table':
                failed = True
            if failed:
                rospy.logerr("Target pose is in collision. Motion planning precondition failed.")
                sys.exit(1)
                return found_grasp_pose_i, pre_grasp_last_valid_state, grasp_last_valid_state_without_attach, target_pose_last_valid_state, obj_pose2


    for grasp_pose_i in range(len(global_grasp_pose.pre_grasp_poses)):
        print('checking grasp_pose %d...' % (grasp_pose_i))
        pre_grasp_pose = global_grasp_pose.pre_grasp_poses[grasp_pose_i]
        grasp_pose = global_grasp_pose.grasp_poses[grasp_pose_i]
        # check if close to failed case
        if len(fail_grasp_pose):
            near_fail = False
            for k in range(len(fail_grasp_pose)):
                dis_p, dis_o = pose_distance(fail_grasp_pose[k], grasp_pose)
                print('distance to failed case: [%f, %f]' % (dis_p, dis_o))
                if dis_p <= fail_pos_threshold and dis_o <= fail_ori_threshold * np.pi/180:
                    # near fail case, check next one
                    print('Near failed case, skip...')
                    near_fail = True
                    break
            if near_fail:
                continue
        # calculate retreat_vector
        pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
        grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
        pre_grasp_retreat_vec = retreat_vec_calculation(pre_grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))
        grasp_retreat_vec = retreat_vec_calculation(grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))

        # pre_grasp_pose parameters
        pre_grasp_end_retreat_step = -0.01

        grasp_end_retreat_step = 0.005

        retreated_grasp_pose = copy.deepcopy(grasp_pose)
        retreated_grasp_pose_np = grasp_pose_np + grasp_retreat_vec * grasp_end_retreat_step
        retreated_grasp_pose.position.x = retreated_grasp_pose_np[0]
        retreated_grasp_pose.position.y = retreated_grasp_pose_np[1]
        retreated_grasp_pose.position.z = retreated_grasp_pose_np[2]




        # * precondition check for pre-grasp pose *
        _, status, pre_grasp_last_valid_state = \
            pose_precondition_check(pre_grasp_pose, pre_grasp_pose_np, pre_grasp_retreat_vec, \
                                    pre_grasp_last_valid_state, pre_grasp_end_retreat_step, collision=True)
        if not status:
            # precondition failed
            fail_grasp_pose.append(grasp_pose)
            continue
        # * precondition check for grasp pose *
        _, status, grasp_last_valid_state_without_attach = \
            pose_precondition_check(grasp_pose, grasp_pose_np, grasp_retreat_vec, \
                                    pre_grasp_last_valid_state, grasp_end_retreat_step, collision=True) # only self-collision
        if not status:
            # precondition failed
            fail_grasp_pose.append(grasp_pose)
            continue
        # * precondition check for target pose *
        obj_pose2_retreated = copy.deepcopy(obj_pose2)
        obj_pose2_retreated_np = np.array([obj_pose2_retreated.position.x,obj_pose2_retreated.position.y,obj_pose2_retreated.position.z])
        obj_pose2_retreated_np = obj_pose2_retreated_np + target_retreat_vec * target_end_retreat_step
        obj_pose2_retreated.position.x = obj_pose2_retreated_np[0]
        obj_pose2_retreated.position.y = obj_pose2_retreated_np[1]
        obj_pose2_retreated.position.z = obj_pose2_retreated_np[2]
        try:
            attached_obj = obtain_attach_object(model_name, scale, obj_pose2_retreated)
            attached_obj.object.operation = attached_obj.object.ADD # we only want to change the pose
        except:
            rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
            attached_obj = None

        # when checking target pose, check if the object is symmetric to rotation first, and rotate until
        # a valid pose is found
        rotations = [None]
        if model_name not in symmetry_df.index:
            rospy.logerr("Motion Planning: object not in symmetry file. Treating as Box.")
            rotations.append(((1,0,0), np.pi))
            rotations.append(((1,0,0), np.pi/2))
            rotations.append(((1,0,0), 3*np.pi/2))

            rotations.append(((0,1,0), np.pi))
            rotations.append(((0,1,0), np.pi/2))
            rotations.append(((0,1,0), np.pi/2*3))

            rotations.append(((0,0,1), np.pi))
            rotations.append(((0,0,1), np.pi/2))
            rotations.append(((0,0,1), np.pi/2*3))

        else:
            if symmetry_df.loc[model_name,'symmetry_x'] in ['symmetry_90', 'symmetry']:
                #x_rots.append(np.pi)  # can rotate 90 degrees
                rotations.append(((1,0,0), np.pi/2))
                rotations.append(((1,0,0), np.pi))
                rotations.append(((1,0,0), 3*np.pi/2))
            elif symmetry_df.loc[model_name,'symmetry_x'] == 'symmetry_180':
                rotations.append(((1,0,0), np.pi))

            if symmetry_df.loc[model_name,'symmetry_y'] in ['symmetry_90', 'symmetry']:
                #x_rots.append(np.pi)  # can rotate 90 degrees
                rotations.append(((0,1,0), np.pi/2))
                rotations.append(((0,1,0), np.pi))
                rotations.append(((0,1,0), 3*np.pi/2))
            elif symmetry_df.loc[model_name,'symmetry_y'] == 'symmetry_180':
                rotations.append(((0,1,0), np.pi))

            if symmetry_df.loc[model_name,'symmetry_z'] in ['symmetry_90', 'symmetry']:
                #x_rots.append(np.pi)  # can rotate 90 degrees
                rotations.append(((0,0,1), np.pi/2))
                rotations.append(((0,0,1), np.pi))
                rotations.append(((0,0,1), 3*np.pi/2))
            elif symmetry_df.loc[model_name,'symmetry_z'] == 'symmetry_180':
                rotations.append(((0,0,1), np.pi))


        obj_pose2_tf = scale_pose_to_tf(pose=obj_pose2)
        for rot_i in range(len(rotations)):

            # obtain the rotation matrix
            if rotations[rot_i] is None:
                obj_pose2_rotation = obj_pose2
            else:
                # obtain rotation around the axis
                R = tf.transformations.rotation_matrix(angle=rotations[rot_i][1], direction=rotations[rot_i][0])
                obj_pose2_rotation_tf = obj_pose2_tf.dot(R)
                _, obj_pose2_rotation = tf_to_scale_pose(obj_pose2_rotation_tf)

            target_pose = grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2_rotation, retreated_grasp_pose)
            target_pose_np = np.array([target_pose.position.x,target_pose.position.y,target_pose.position.z])
            _, status, target_pose_rotation_last_valid_state = \
                pose_precondition_check(target_pose, target_pose_np, target_retreat_vec, \
                                        target_pose_last_valid_state, target_end_retreat_step, collision=True, attach_object=attached_obj)
            if status:
                target_pose_last_valid_state = target_pose_rotation_last_valid_state
                break
        if not status:
            # precondition failed
            fail_grasp_pose.append(grasp_pose)
            continue
        # otherwise, we found our pose to work on
        found_grasp_pose_i = grasp_pose_i
        break

    if found_grasp_pose_i == -1:
        # all grasp positions failed, error
        rospy.logerr("precondition check grasp poses are invalid (in collision or not valid IK).")
        sys.exit(1)

    return found_grasp_pose_i, pre_grasp_last_valid_state, grasp_last_valid_state_without_attach, target_pose_last_valid_state, obj_pose2_rotation



def find_grasp_pose(global_grasp_pose, obj_pose1, obj_pose2, scale, model_name, \
                    grasp_pose_i, pre_grasp_last_valid_state, grasp_last_valid_state_without_attach, target_pose_last_valid_state):

    # load parameters
    pre_grasp_pose = global_grasp_pose.pre_grasp_poses[grasp_pose_i]
    grasp_pose = global_grasp_pose.grasp_poses[grasp_pose_i]

    # calculate retreat_vector
    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    pre_grasp_retreat_vec = retreat_vec_calculation(pre_grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))
    grasp_retreat_vec = retreat_vec_calculation(grasp_pose, local_retreat_vec=np.array([-1.,0.,0.]))

    # pre_grasp_pose parameters
    pre_grasp_retreat_threshold = 0.0005
    pre_grasp_retreat_num_step = 10
    pre_grasp_start_retreat_step = -0.02
    pre_grasp_end_retreat_step = -0.01

    grasp_retreat_threshold = 0.0005
    grasp_retreat_num_step = 10
    grasp_start_retreat_step = -0.01
    grasp_end_retreat_step = 0.005

    # * binary search to obtain the pose *
    pre_grasp_pose, pre_grasp_status, pre_grasp_last_valid_state = \
            binary_search_position(pre_grasp_start_retreat_step, pre_grasp_end_retreat_step, \
                                pre_grasp_retreat_num_step, pre_grasp_retreat_threshold, \
                                pre_grasp_pose, pre_grasp_pose_np, pre_grasp_retreat_vec, \
                                pre_grasp_last_valid_state, collision=True)
    if not pre_grasp_status:
        rospy.logerr("Can't find pre_grasp pose for the found pose.")
        sys.exit(1)
    # otherwise the found one ise pre_grasp_pose
    print('found pre_grasp_pose')
    # find grasp_pose, smaller retreat step
    # try using binary search to find the first place that does not have collision (within some threshold defined)


    if grasp_last_valid_state_without_attach is None:
        grasp_last_valid_state_without_attach = pre_grasp_last_valid_state

    grasp_pose, grasp_status, grasp_last_valid_state_without_attach = \
            binary_search_position(grasp_start_retreat_step, grasp_end_retreat_step, \
                                grasp_retreat_num_step, grasp_retreat_threshold, \
                                grasp_pose, grasp_pose_np, grasp_retreat_vec, \
                                grasp_last_valid_state_without_attach, collision=True)  # original false

    if not grasp_status:
        rospy.logerr("Can't find grasp pose for the found pose.")
        sys.exit(1)

    print('found pre_grasp_pose and grasp_pose')
    target_pose = grasp_pose_transformation_from_object_pose(obj_pose1, obj_pose2, grasp_pose)
    target_retreat_threshold = 0.01
    target_retreat_num_step = 10
    target_start_retreat_step = 0.
    target_end_retreat_step = 0.05
    target_retreat_vec = np.array([0.,0,1])
    target_pose_np = np.array([target_pose.position.x,target_pose.position.y,target_pose.position.z])

    obj_pose2_retreated = copy.deepcopy(obj_pose2)
    try:
        attached_obj = obtain_attach_object(model_name, scale, obj_pose2_retreated)
        attached_obj.object.operation = attached_obj.object.ADD
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
        attached_obj = None

    # obtain target arm pose based on grasp pose and object poses
    target_pose, target_status, target_pose_last_valid_state = \
            binary_search_position(target_start_retreat_step, target_end_retreat_step, \
                                    target_retreat_num_step, target_retreat_threshold, \
                                    target_pose, target_pose_np, target_retreat_vec, target_pose_last_valid_state, \
                                    collision=True, attach_object=attached_obj)
    if not target_status:
        rospy.logerr("Can't find target pose for the found pose.")
        sys.exit(1)
    # grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    # grasp_pose_np = grasp_pose_np - grasp_retreat_vec * 0.005
    # grasp_pose.position.x = grasp_pose_np[0]
    # grasp_pose.position.y = grasp_pose_np[1]
    # grasp_pose.position.z = grasp_pose_np[2]
    return pre_grasp_pose, grasp_pose, target_pose, grasp_pose_i


def pre_grasp(start_state, target_pose):
    x = group.get_current_pose()
    x.pose = target_pose
    last_valid_state = None
    # set the start to be our current state
    state = robot.get_current_state()
    group.set_start_state(state)
    planning_time = 6
    for planning_attempt_i in range(5):
        group.set_pose_target(x)
        group.set_planning_time(planning_time)
        group.set_num_planning_attempts(100)
        group.allow_replanning(False)
        plan = group.plan()  # returned value: tuple (flag, RobotTrajectory, planning_time, error_code)
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan[0]:
            pre_grasp_plan = plan[1]
            break
        else:
            pre_grasp_plan = None
            planning_time += 0.5
    return pre_grasp_plan


def straight_line_move(start_state, start_pose, target_pose):
    # firstly generate list of waypoints from start_pose to target_pose, until collision happens
    # use around 20 points in between
    _, robot_state, error_code_val = ik(start_pose, collision=True, init_robot_state=start_state)
    num_waypoints = 20
    total_duration = 4.  # we want 2s to go to the desired location
    time_step = total_duration / num_waypoints


    start_pose_np = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    target_pose_np = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])

    # set the resolution
    eef_step = 0.005
    num_waypoints = int(np.round(np.linalg.norm(target_pose_np - start_pose_np, ord=np.inf) / eef_step))
    total_duration = 2.
    time_step = total_duration / num_waypoints

    step = (target_pose_np - start_pose_np) / num_waypoints
    waypoint_list = []
    straight_line_trajectory = JointTrajectory()
    straight_line_trajectory.header = Header()
    #straight_line_trajectory.header.stamp = rospy.Time.now()
    straight_line_trajectory.header.frame_id = group.get_planning_frame()
    straight_line_trajectory.joint_names = start_state.joint_state.name[:6]  # extract arm
    last_valid_state = start_state
    last_valid_pose = start_pose
    last_valid_i = -1

    jumped_num = 0
    jump_threshold = 2  # can only jump max 2 waypoints
    min_clearance_ratio = 0.5 # at least 50% should be valid
    last_clearance_ratio = 0.9  # last node should be >= 90%
    joint_continuous_threshold = 45. * np.pi / 180

    previous_joints = np.array(start_state.joint_state.position[:6])
    for i in range(1,num_waypoints+1):
        inter_pose_np = start_pose_np + step * i
        inter_pose = copy.deepcopy(start_pose)
        inter_pose.position.x = inter_pose_np[0]
        inter_pose.position.y = inter_pose_np[1]
        inter_pose.position.z = inter_pose_np[2]
        _, robot_state, error_code_val = ik(inter_pose, collision=True, init_robot_state=last_valid_state)

        # check if joint value is continuous enough (difference is within some threshold)
        if error_code_val == 1:
            current_joints = np.array(robot_state.joint_state.position[:6])
            distance = np.linalg.norm(current_joints - previous_joints, ord=np.inf)
            if distance >= joint_continuous_threshold:
                # larger than the threshold, not continuous
                error_code_val = 0
                rospy.logwarn("straight-line-move: joint value not continuous. distance degree: %f" % (distance * 180 / np.pi))
            previous_joints = current_joints

        if error_code_val == 1:
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
            last_valid_pose = inter_pose
            last_valid_i = i
            jump_num = 0  # reset jump_um
        else:
            jumped_num += 1
            if jumped_num > jump_threshold:
                rospy.logwarn("straight_line_move: exceeding jump_threshold.")
                break

    # check if success
    if jumped_num > jump_threshold or (float(last_valid_i) / num_waypoints) < last_clearance_ratio or len(straight_line_trajectory.points) <= min_clearance_ratio:
        # considered fail
        x = group.get_current_pose()
        x.pose = target_pose
        # set the start to be our current state
        state = robot.get_current_state()
        group.set_start_state(start_state)

        for planning_attempt_i in range(3):
            group.set_pose_target(x)
            group.set_planning_time(5)
            group.set_num_planning_attempts(50)
            group.allow_replanning(False)
            plan = group.plan()
            group.clear_pose_targets()
            group.clear_path_constraints()
            if plan[0]:
                straight_line_plan = plan[1]
                break
            else:
                straight_line_plan = None
        if straight_line_plan is not None:
            straight_line_trajectory = straight_line_plan.joint_trajectory
            last_valid_state = robot_state_from_plan(straight_line_plan)
            last_valid_pose = target_pose
        else:
            # failed
            rospy.logerr("striaght-line path failed.")
            sys.error(1)
    return straight_line_trajectory, last_valid_state, last_valid_pose


from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from shape_msgs.msg import Mesh, MeshTriangle, SolidPrimitive
from geometry_msgs.msg import Point
import pyassimp

def add_mesh(model_name, scale, obj_pose):
    collision_obj = obtain_collision_object(model_name, scale, obj_pose)
    co_pub.publish(collision_obj)
    #rospy.sleep(1)

def obtain_collision_object(model_name, scale, obj_pose):
    collision_obj = CollisionObject()
    collision_obj.id = model_name
    mesh = Mesh()
    # load mesh file
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"

    # use trimesh
    import trimesh
    trimesh_mesh = trimesh.load(mesh_file_name)
    for face in trimesh_mesh.faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = [face[0], face[1], face[2]]
        mesh.triangles.append(triangle)
    for vertex in trimesh_mesh.vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)

    collision_obj.meshes = [mesh]
    collision_obj.mesh_poses = [obj_pose]
    collision_obj.operation = CollisionObject.ADD  # will replace the object if it existed before
    collision_obj.header = group.get_current_pose().header
    return collision_obj

def obtain_attach_object(model_name, scale, obj_pose):
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
    #mesh_file_name = "/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/collision.obj"

    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/visual_meshes/visual.dae"

    # pyassimp_mesh = pyassimp.load(mesh_file_name)

    # use trimesh
    import trimesh
    trimesh_mesh = trimesh.load(mesh_file_name)
    for face in trimesh_mesh.faces:
        triangle = MeshTriangle()
        triangle.vertex_indices = [face[0], face[1], face[2]]
        mesh.triangles.append(triangle)
    for vertex in trimesh_mesh.vertices:
        point = Point()
        point.x = vertex[0]*scale[0]
        point.y = vertex[1]*scale[1]
        point.z = vertex[2]*scale[2]
        mesh.vertices.append(point)

    # for box filtering (notice that this is unscaled)
    min_xyz = np.array(trimesh_mesh.vertices).min(axis=0)
    max_xyz = np.array(trimesh_mesh.vertices).max(axis=0)
    attached_obj_shape.meshes = [mesh]
    attached_obj_shape.mesh_poses = [obj_pose]
    attached_obj_shape.operation = CollisionObject.ADD  # will replace the object if it existed before
    attached_obj_shape.header = group.get_current_pose().header

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

    # # for box filtering (notice that this is unscaled)
    # min_xyz = np.array(pyassimp_mesh.meshes[0].vertices).min(axis=0)
    # max_xyz = np.array(pyassimp_mesh.meshes[0].vertices).max(axis=0)

    # attached_obj_shape.meshes = [mesh]
    # attached_obj_shape.mesh_poses = [obj_pose]
    # attached_obj_shape.operation = CollisionObject.ADD  # will replace the object if it existed before
    # attached_obj_shape.header = group.get_current_pose().header
    # pyassimp.release(pyassimp_mesh)

    attached_obj.object = attached_obj_shape
    return attached_obj
def attach_object_to_gripper(model_name, scale, obj_pose, grasp_state):
    try:
        attached_obj = obtain_attach_object(model_name, scale, obj_pose)
        grasp_state.attached_collision_objects.append(attached_obj)
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")

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
    #clear_octomap()

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
    z_padding_limit = 0.1
    #target_pose.position.z += 0.1  # add padding
    target_pose = copy.deepcopy(target_pose)
    max_attempt = 5
    z_padding_step = z_padding_limit / max_attempt
    planning_time = 6.5
    for planning_attempt_i in range(max_attempt):
        group.set_start_state(start_state)
        target_pose.position.z += z_padding_step
        group.set_pose_target(target_pose)
        group.set_planning_time(planning_time)
        group.set_num_planning_attempts(100)
        group.allow_replanning(False)
        # set constraint: gripper can't be upward too much (y, and z axis rotation limit)
        constraint = Constraints()
        constraint.name = "gripper_downward_constraint"
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = 'world'
        ori_constraint.link_name = 'ee_link'
        
        """
        x_axis_rot = 0.
        y_axis_rot = -np.pi/2
        z_axis_rot = 0.#np.pi/2
        display_rot = [(0., 0., 0.), (0., -np.pi/2, 0.), (0., np.pi/2, 0.), (0., 0., np.pi/2), (0., 0., -np.pi/2), (0., -np.pi/2, -np.pi/2),
                       (0., np.pi/2, -np.pi/2), (0., -np.pi/2, np.pi/2), (0., np.pi/2, np.pi/2), (-np.pi/2, -np.pi/2, -np.pi/2),
                       (-np.pi/2, -np.pi/2, np.pi/2), (-np.pi/2, np.pi/2, -np.pi/2), (-np.pi/2, np.pi/2, np.pi/2)]
        for x_axis_rot, y_axis_rot, z_axis_rot in display_rot:
            print('axis angles: %f, %f, %f' % (x_axis_rot, y_axis_rot, z_axis_rot))           
            down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
            # visualize the downward pose
            pose = group.get_current_pose().pose
            pose.orientation.x = down_quat[0]
            pose.orientation.y = down_quat[1]
            pose.orientation.z = down_quat[2]
            pose.orientation.w = down_quat[3]
            init_robot_state, robot_state, status = \
                ik(pose, collision=False, init_robot_state=None)
            display_robot_state(robot_state)
            down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
            dif_quat = tf.transformations.quaternion_from_euler(x_axis_rot, y_axis_rot, z_axis_rot)
            down_quat = tf.transformations.quaternion_multiply(down_quat, dif_quat)
            #down_quat = tf.transformations.quaternion_from_euler(0+x_axis_rot, np.pi/2+y_axis_rot, 0+z_axis_rot) # clockwise 90 degrees around y
            # visualize the downward pose
            pose = group.get_current_pose().pose
            pose.orientation.x = down_quat[0]
            pose.orientation.y = down_quat[1]
            pose.orientation.z = down_quat[2]
            pose.orientation.w = down_quat[3]
            init_robot_state, robot_state, status = \
                ik(pose, collision=False, init_robot_state=None)
            display_robot_state(robot_state)
        
        # sample within the tolerance and display
        for rot_i in range(20):
            print('rotation: %d' % (rot_i))
            x_axis_rot = np.random.random() * (2*np.pi) - (np.pi)
            y_axis_rot = np.random.random() * (np.pi+10*np.pi/180) - (np.pi/2 + 5*np.pi/180)
            z_axis_rot = np.random.random() * (np.pi+10*np.pi/180) - (np.pi/2 + 5*np.pi/180)

            print('axis angles: %f, %f, %f' % (x_axis_rot, y_axis_rot, z_axis_rot))           
            down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
            # visualize the downward pose
            pose = group.get_current_pose().pose
            pose.orientation.x = down_quat[0]
            pose.orientation.y = down_quat[1]
            pose.orientation.z = down_quat[2]
            pose.orientation.w = down_quat[3]
            init_robot_state, robot_state, status = \
                ik(pose, collision=False, init_robot_state=None)
            display_robot_state(robot_state)
            down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
            dif_quat = tf.transformations.quaternion_from_euler(x_axis_rot, y_axis_rot, z_axis_rot)
            down_quat = tf.transformations.quaternion_multiply(down_quat, dif_quat)
            #down_quat = tf.transformations.quaternion_from_euler(0+x_axis_rot, np.pi/2+y_axis_rot, 0+z_axis_rot) # clockwise 90 degrees around y
            # visualize the downward pose
            pose = group.get_current_pose().pose
            pose.orientation.x = down_quat[0]
            pose.orientation.y = down_quat[1]
            pose.orientation.z = down_quat[2]
            pose.orientation.w = down_quat[3]
            init_robot_state, robot_state, status = \
                ik(pose, collision=False, init_robot_state=None)
            display_robot_state(robot_state)
        """


        down_quat = tf.transformations.quaternion_from_euler(0, np.pi/2, 0) # clockwise 90 degrees around y
        ori_constraint.orientation = Quaternion(down_quat[0],down_quat[1],down_quat[2],down_quat[3]) # define orientation so that x-axis is downward, y and z are in horizontal plane
        ori_constraint.absolute_x_axis_tolerance = np.pi  # allow rotation around x
        ori_constraint.absolute_y_axis_tolerance = np.pi/2 + 5. * np.pi / 180  # allow rotation around y
        ori_constraint.absolute_z_axis_tolerance = np.pi/2 + 5. * np.pi / 180   # allow rotation around z
        ori_constraint.weight = 1
        constraint.orientation_constraints = [ori_constraint]
        group.set_path_constraints(constraint)
        group.set_goal_orientation_tolerance(5*np.pi/180) # we want this region to be tight
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan[0]:
            place_plan = plan[1]
            break
        else:
            place_plan = None
            planning_time += 0.5
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0
    return place_plan, target_pose



def arm_reset(start_state):
    # plan a path to reset position (up)
    group.clear_pose_targets()
    #[-1.57, -1.57, 1.57, 0, 0, 0]
    # up: [0, -1.57,0, -1.57, 0, 0]
    # arm_cmd.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', \
    #                        'elbow_joint', 'wrist_1_joint', \
    #                        'wrist_2_joint', 'wrist_3_joint']
    # waypoint = JointTrajectoryPoint()
    # waypoint.positions = [-0.68, -0.63, 0.69, -0.88, -0.53, -0.19]

    #up_joints = {"elbow_joint": -1.57, "shoulder_lift_joint": -1.577, "shoulder_pan_joint": 1.57,
    #             "wrist_1_joint": 0., "wrist_2_joint": 0., "wrist_3_joint": 0.}
    up_joints = {"elbow_joint": 0.69, "shoulder_lift_joint": -0.63, "shoulder_pan_joint": -0.68,
                 "wrist_1_joint": -0.88, "wrist_2_joint": -0.53, "wrist_3_joint": -0.19}
    planning_time = 6
    for planning_attempt_i in range(5):
        group.set_start_state(start_state)
        group.set_joint_value_target(up_joints)
        group.set_planning_time(planning_time)
        group.set_num_planning_attempts(100)
        group.allow_replanning(False)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan[0]:
            #if plan:
            reset_plan = plan[1]
            break
        else:
            reset_plan = None
            print('planning failed. Another attempt is tried...')
            planning_time += 1
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
    scale = np.array(scale)
    #########################################
    # reset the planning scene
    #clear_octomap()
    #print('get object:')
    #print(scene.get_objects())
    scene.remove_world_object(name=model_name)  # clean scene afterwards
    scene.remove_world_object()


    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose1
    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    #mesh_file_name = "/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/collision.obj"

    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/visual_meshes/visual.dae"
    # add a little padding so that we can ensure grasp is successful
    try:
        add_mesh(model_name, scale*scale_padding, obj_pose1)
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
    #scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0]*scale_padding, scale[1]*scale_padding, scale[2]*scale_padding))
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
        rospy.logerr("grasp_pose generation failed.")
        sys.exit(1)

    global_grasp_pose = resp1.grasps.global_grasp_poses[0]
        # ***
    # Note: Here we TRUST the grasp poses generated by GraspGen algorithm,
    # and use the score it provides to decide the graspability of objects.
    # Hence we just go over the sorted grasp poses, and try until we find
    # a valid pose.
    # ***

    #** stage 1.5: precondition check for motion plan **
    rospy.loginfo("grasp_pose received. Start precondition check...")
    # update obj_pose2 to symmetric and collision-free one
    grasp_pose_i, pre_grasp_last_valid_state, grasp_last_valid_state_without_attach, target_pose_last_valid_state, obj_pose2 = \
                motion_planning_precondition_check(global_grasp_pose, obj_pose1, obj_pose2, scale, model_name)
    pre_grasp_pose, grasp_pose, target_pose, grasp_pose_i = \
                find_grasp_pose(global_grasp_pose, obj_pose1, obj_pose2, scale, model_name, \
                                grasp_pose_i, pre_grasp_last_valid_state, grasp_last_valid_state_without_attach, \
                                target_pose_last_valid_state)
    # we assume the gripper is open
    group.clear_pose_targets()
    # reset mesh position
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose1
    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    #mesh_file_name = "/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/collision.obj"

    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/visual_meshes/visual.dae"
    try:
        add_mesh(model_name, scale*scale_padding, obj_pose1)#obj_pose_stamped)
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
    #scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0]*scale_padding, scale[1]*scale_padding, scale[2]*scale_padding))

    #** stage 2: generate pre-grasp plan **
    rospy.loginfo("generating pre-grasp plan...")
    #display_robot_state(robot.get_current_state())
    #raw_input("display input...")
    pre_grasp_plan = pre_grasp(robot.get_current_state(), pre_grasp_pose)
    #print(pre_grasp_plan)
    #print(pre_grasp_plan.joint_trajectory)
    pre_grasp_trajectory = pre_grasp_plan.joint_trajectory
    # remember the end state to be used for next stage
    pre_grasp_plan_end_state = robot_state_from_plan(pre_grasp_plan)

    #** stage 3: plan cartesian path to grasp pose **
    # directly using IK to achieve this part
    rospy.loginfo("generating pre-to-grasp plan...")
    # we can safely remove target object for collision checking
    scene.remove_world_object()
    pre_to_grasp_trajectory, grasp_plan_end_state, grasp_plan_end_pose = straight_line_move(pre_grasp_plan_end_state, pre_grasp_pose, grasp_pose)
    #** stage 4: attach the object to the gripper. plan the place trajectory to move the arm **
    #** attach object to gripper **
    #grasp_plan_end_state = attach_object_to_gripper(model_name, scale, obj_pose1, grasp_plan_end_state)


    #** stage 3.5: lift the object up (post_grasp_plan)
    rospy.loginfo("generating post_grasp plan...")

    # post_grasp_plan_z_offset = 0.1  # 10 cm
    # post_grasp_pose = copy.deepcopy(grasp_pose)
    # post_grasp_pose.position.z += post_grasp_plan_z_offset
    #rospy.logwarn("before calculation..")

    post_grasp_offset = 0.15 #5cm
    grasp_retreat_vec = retreat_vec_calculation(grasp_plan_end_pose, local_retreat_vec=np.array([-1.,0.,0.]))
    post_grasp_pose = copy.deepcopy(grasp_plan_end_pose)
    grasp_pose_np = np.array([grasp_plan_end_pose.position.x, grasp_plan_end_pose.position.y, grasp_plan_end_pose.position.z])
    post_grasp_pose_np = grasp_pose_np + grasp_retreat_vec * post_grasp_offset  # 5cm?
    post_grasp_pose.position.x = post_grasp_pose_np[0]
    post_grasp_pose.position.y = post_grasp_pose_np[1]
    post_grasp_pose.position.z = post_grasp_pose_np[2]
    # post_grasp_pose = copy.deepcopy(pre_grasp_pose)
    #rospy.logwarn("before straight_ilne_move...")

    post_grasp_trajectory, post_grasp_end_state, post_grasp_end_pose = straight_line_move(grasp_plan_end_state, grasp_pose, post_grasp_pose)
    post_grasp_end_pose_np = np.array([post_grasp_end_pose.position.x,post_grasp_end_pose.position.y,post_grasp_end_pose.position.z])
    change_np = post_grasp_end_pose_np - grasp_pose_np
    post_obj_pose1 = copy.deepcopy(obj_pose1)
    post_obj_pose_np = np.array([post_obj_pose1.position.x, post_obj_pose1.position.y, post_obj_pose1.position.z])
    post_obj_pose_np += change_np
    post_obj_pose1.position.x = post_obj_pose_np[0]
    post_obj_pose1.position.y = post_obj_pose_np[1]
    post_obj_pose1.position.z = post_obj_pose_np[2]
    #rospy.logwarn("before attaching object...")

    # after post grasp, we can safely ignore the attached object in the scene
    post_grasp_end_state = attach_object_to_gripper(model_name, scale, post_obj_pose1, post_grasp_end_state)

    #** stage 4: place
    rospy.loginfo("generating place plan...")

    place_plan, target_pose = place(start_state=post_grasp_end_state, target_pose=target_pose)
    place_trajectory = place_plan.joint_trajectory
    place_end_state = robot_state_from_plan(place_plan)
    # remove the object from the planning scene
    #scene.remove_attached_object(link="robotiq_2f_85_left_pad", name=model_name)
    # check if the object is still attached
    #print('attached objects: ')
    #print(scene.get_attached_objects())
    #hello = raw_input("end of attaching object. Please input...\n")
    # since we provided start state, there is no attached object

    # modify the pose of object to the placed place
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose2
    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    #mesh_file_name = "/root/ocrtoc_ws/src/ocrtoc_motion_planning/ocrtoc_motion_planning/data/models/"+model_name+"/visual_meshes/collision.obj"

    #mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/visual_meshes/visual.dae"
    try:
        add_mesh(model_name, scale*scale_padding, obj_pose2)# obj_pose_stamped)
    except:
        rospy.logerr("Motion Planning: error loading mesh file. must be new object.")
    #scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0]*scale_padding, scale[1]*scale_padding, scale[2]*scale_padding))
    #print('object in scene:')
    #print(scene.get_objects().keys())
    #hello = raw_input("end of changing pose of object. Please input...\n")


    place_end_state.attached_collision_objects = []
    place_end_state.is_diff = True

    # TODO detach here
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
    """
    obj_pose_stamped = PoseStamped()
    obj_pose_stamped.header.frame_id = "world"
    obj_pose_stamped.pose = obj_pose2
    mesh_file_name = "/root/ocrtoc_materials/models/"+model_name+"/collision_meshes/collision.obj"
    scene.add_mesh(name=model_name, pose=obj_pose_stamped, filename=mesh_file_name, size=(scale[0], scale[1], scale[2]))
    """

    #** stage 4.5: retreat
    # retreat the gripper first, and then plan wihout mesh
    post_place_offset = 0.2 #10cm
    grasp_retreat_vec = retreat_vec_calculation(target_pose, local_retreat_vec=np.array([-1.,0.,0.]))
    pose_place_pose = copy.deepcopy(target_pose)
    target_pose_np = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
    target_pose_np += grasp_retreat_vec * post_place_offset  # 10cm?
    pose_place_pose.position.x = target_pose_np[0]
    pose_place_pose.position.y = target_pose_np[1]
    pose_place_pose.position.z = target_pose_np[2]
    try:
        post_place_trajectory, post_place_end_state, post_place_end_pose = straight_line_move(place_end_state, target_pose, pose_place_pose)
    except:
        # try lifting up instead of retreating as a backup
        post_place_offset = 0.15 #10cm
        grasp_retreat_vec = np.array([0.,0,-1])
        pose_place_pose = copy.deepcopy(target_pose)
        target_pose_np = np.array([target_pose.position.x, target_pose.position.y, target_pose.position.z])
        target_pose_np += grasp_retreat_vec * post_place_offset  # 10cm?
        pose_place_pose.position.x = target_pose_np[0]
        pose_place_pose.position.y = target_pose_np[1]
        pose_place_pose.position.z = target_pose_np[2]
        post_place_trajectory, post_place_end_state, post_place_end_pose = straight_line_move(place_end_state, target_pose, pose_place_pose)




    rospy.loginfo("generating reset plan...")
    reset_plan = arm_reset(start_state=post_place_end_state)
    reset_trajectory = reset_plan.joint_trajectory


    scene.remove_world_object(name=model_name)  # clean scene afterwards

    plan_res = {}
    plan_res['pre_grasp_trajectory'] = pre_grasp_trajectory
    plan_res['pre_to_grasp_trajectory'] = pre_to_grasp_trajectory
    plan_res['post_grasp_trajectory'] = post_grasp_trajectory
    plan_res['place_trajectory'] = place_trajectory
    plan_res['post_place_trajectory'] = post_place_trajectory
    plan_res['reset_trajectory'] = reset_trajectory


    plan_res['global_grasp_pose'] = global_grasp_pose
    plan_res['grasp_id'] = grasp_pose_i
    plan_res['pre_grasp_end_pose'] = pre_grasp_pose
    plan_res['grasp_end_pose'] = grasp_plan_end_pose
    plan_res['post_grasp_end_pose'] = post_grasp_end_pose
    plan_res['place_end_pose'] = target_pose
    plan_res['post_place_end_pose'] = post_place_end_pose
    return plan_res



#======================================================================================================

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planning_test',
                    anonymous=True)
    import json
    scene_pose = open('../scene_info/1-1.json', 'r')
    scene_pose = json.load(scene_pose)
    # jenga
    # model_name = 'jenga'
    # data_str = '-0.0599994957447052 -0.31989747285842896 0.014999995008111 2.261107306891991e-09 -1.8113155351356218e-09 -6.071384286259734e-09'
    # scale = np.array([1., 1., 1.])
    # potted_meat_can
    # model_name = 'potted_meat_can'
    # data_str = '0.15989847481250763 0.29000547528266907 0.04212800785899162 -4.840692449249814e-08 -4.5454357156415357e-07 -6.927437318496676e-08'
    # scale = np.array([1., 1., 1.])
    # master chef can
    # model_name = 'master_chef_can'
    # data_str = '-0.03987777605652809 0.32008427381515503 0.03530200198292732 -1.4431422264283453e-06 -7.782947982250227e-07 -3.5607089180211774e-07'
    # scale = np.array([0.5,0.5,0.5])


    #wood_block
    # model_name = 'wood_block'
    # data_str = '0.09994173049926758 -0.31943440437316895 0.022629257291555405 -0.005208467812430025 0.005718712075542679 0.005303920438713571'
    # scale = np.array([0.5,0.5,0.5])

    # pudding_box
    model_name = 'pudding_box'
    data_str = '0.25974133610725403 -0.3102521002292633 0.009862901642918587 0.022348699698035056 -0.01139162625176719 -9.36981416203137e-05'
    scale = np.array([0.5,0.5,0.5])

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

    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)
    # compose into TF matrix
    target_tf = tf.transformations.compose_matrix(scale=scale, shear=shear, angles=orientation, translate=position, perspective=persp)
    scale, start_pose = tf_to_scale_pose(start_tf)
    scale, target_pose = tf_to_scale_pose(target_tf)
    plan_res = one_shot_grasp_with_object_pose(model_name, scale, start_pose, target_pose)
    #pre_grasp_trajectory, pre_to_grasp_trajectory, post_grasp_trajectory, place_trajectory, reset_trajectory, global_grasp_pose, grasp_pose_i = one_shot_grasp_with_object_pose(model_name, scale, start_pose, target_pose)
    motion_planning_execution.execute_plan_close_loop_with_pub(arm_cmd_pub, gripper_cmd_pub, plan_res)

if __name__ == "__main__":
    main()

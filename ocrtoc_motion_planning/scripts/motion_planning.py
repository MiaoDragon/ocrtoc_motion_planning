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

import numpy as np

def plan_arm_state_to_state(x0, xG):
    """
    function:
    ===========================================
    given start and goal joint states, return a plan for execution

    input:
    ===========================================
    x0, xG: start and goal state joints

    format: {'joint_name': joint_value}
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    # We can get the name of the reference frame for this robot:
    planning_frame = group.get_planning_frame()
    print("============ Reference frame: %s" % planning_frame)

    # We can also print the name of the end-effector link for this group:
    eef_link = group.get_end_effector_link()
    print("============ End effector: %s" % eef_link)

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print("============ Robot Groups:", robot.get_group_names())

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print("============ Printing robot state")
    print(robot.get_current_state())

    # We can get the joint values from the group and adjust some of the values:
    joint = group.get_active_joints()
    print(joint)

    target_joint = {}
    target_joint['shoulder_pan_joint'] = 0.
    target_joint['shoulder_lift_joint'] = 0.
    target_joint['elbow_joint'] = 0.
    target_joint['wrist_1_joint'] = 0.
    target_joint['wrist_2_joint'] = np.pi/4
    target_joint['wrist_3_joint'] = 0.

    group.set_joint_value_target(xG)
    group.plan()


# IK
from std_msgs.msg import Header
from moveit_msgs.msg import PositionIKRequest, RobotState, MoveItErrorCodes
from moveit_msgs.srv import GetPositionIK
def ik_generation(x):
    """
    function:
    ===========================================
    given end-effector pose, return the IK solution as joint values.

    input:
    ===========================================
    x: end-effector pose

    format: geometry_msgs/PoseStamped
    """

    # test case
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)
    print('============ Printing robot pose')
    x = group.get_current_pose()
    print(x)
    #x.pose.orientation.w = .7
    #x.pose.position.x = 0.
    #x.pose.position.y = -.7
    #x.pose.position.x += .1
    #x.pose.position.y += .1
    #x.pose.position.y = -0.4182031551
    #x.pose.position.z += .5

    x.pose.orientation.x = -7.2028429049e-05
    x.pose.orientation.y = 0.707122745785
    x.pose.orientation.z = 4.89692439807e-05
    x.pose.orientation.w = 0.707090810864
    x.pose.position.x = 0.284507210148
    x.pose.position.y = -0.4182031551
    x.pose.position.z = 0.062993339788
    robot = moveit_commander.RobotCommander()
    print(robot.get_current_state())

    service = "/compute_ik"
    iksvc = rospy.ServiceProxy(service, GetPositionIK)
    ikreq = PositionIKRequest()
    ikreq.group_name = "robot_arm"
    ikreq.robot_state = robot.get_current_state()
    ikreq.avoid_collisions = True
    #ikreq.pose_stamped = None
    #ikreq.timeout = None
    ikreq.attempts = 5
    ikreq.pose_stamped.pose = x.pose
    hdr = Header(stamp=rospy.Time.now(), frame_id=x.header.frame_id)
    ikreq.pose_stamped.header = hdr

    print('checking for IK using the following pose:')
    print(x.pose)

    try:
        rospy.wait_for_service(service, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        #rospy.logerr("Service call failed: %s" % (e,))

    if (resp.error_code.val == 1):
        robot_state = resp.solution
        print("SUCCESS - Valid Joint Solution Found:")
    else:
        rospy.logerr("service failed with error code: %d" % (resp.error_code.val))


def plan_arm_pose_to_pose(x0, xG):
    """
    function:
    ===========================================
    given start and goal end-effector pose, return a plan for execution

    input:
    ===========================================
    x0, xG: start and goal state joints

    format: geometry_msgs/PoseStamped
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # test data

    #rospy.sleep(2)
    #scene.remove_world_object("master_chef_can")

    #rospy.sleep(2)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.1
    p.pose.position.y = 0.3
    p.pose.position.z = 0.1
    p.pose.orientation.x = 0.
    p.pose.orientation.y = 0.
    p.pose.orientation.z = 0.
    #p.pose.orientation.w = 1.
    scene.add_mesh(name='master_chef_can', pose=p, \
                filename='/root/ocrtoc_materials/models/master_chef_can/collision_meshes/collision.obj', size=(1, 1, 1))


    print("============ Printing robot state")
    print(robot.get_current_state())
    print('============ Printing robot pose')
    print(group.get_current_pose())
    x = group.get_current_pose()

    # x.pose.orientation.x = -7.2028429049e-05
    # x.pose.orientation.y = 0.707122745785
    # x.pose.orientation.z = 4.89692439807e-05
    # x.pose.orientation.w = 0.707090810864
    # x.pose.position.x = 0.284507210148
    # x.pose.position.y = 0.393134287145
    # x.pose.position.z = 0.062993339788

    x.pose.orientation.x = -7.2028429049e-05
    x.pose.orientation.y = 0.707122745785
    x.pose.orientation.z = 4.89692439807e-05
    x.pose.orientation.w = 0.707090810864
    x.pose.position.x = 0.35
    x.pose.position.y = 0.2
    x.pose.position.z = 0.062993339788

    # x.pose.orientation.x = -7.2028429049e-05
    # x.pose.orientation.y = 0.707122745785
    # x.pose.orientation.z = 4.89692439807e-05
    # x.pose.orientation.w = 0.707090810864
    # x.pose.position.x = 0.284507210148
    # x.pose.position.y = -0.4182031551
    # x.pose.position.z = 0.062993339788

    #hello = raw_input("please input\n")
    #rospy.sleep(2)
    #scene.remove_world_object("master_chef_can")
    #rospy.sleep(2)

    group.set_pose_target(xG)
    group.set_planning_time(5)
    group.set_num_planning_attempts(100)
    group.allow_replanning(True)
    plan = group.plan()
    return plan




from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


def plan_arm_pose_to_pose_with_constraints(x0, xG):
    """
    function:
    ===========================================
    given start and goal end-effector pose, return a plan for execution
    add constraints for the height

    input:
    ===========================================
    x0, xG: start and goal state joints

    format: geometry_msgs/PoseStamped
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # test data
    rospy.sleep(2)
    scene.remove_world_object("master_chef_can")

    rospy.sleep(2)
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.1
    p.pose.position.y = 0.3
    p.pose.position.z = 0.1
    p.pose.orientation.x = 0.
    p.pose.orientation.y = 0.
    p.pose.orientation.z = 0.
    #p.pose.orientation.w = 1.
    scene.add_mesh(name='master_chef_can', pose=p, \
                filename='/root/ocrtoc_materials/models/master_chef_can/collision_meshes/collision.obj', size=(1, 1, 1))


    print("============ Printing robot state")
    print(robot.get_current_state())
    print('============ Printing robot pose')
    print(group.get_current_pose())
    x = group.get_current_pose()

    # x.pose.orientation.x = -7.2028429049e-05
    # x.pose.orientation.y = 0.707122745785
    # x.pose.orientation.z = 4.89692439807e-05
    # x.pose.orientation.w = 0.707090810864
    # x.pose.position.x = 0.264507210148
    # x.pose.position.y = 0.353134287145
    # x.pose.position.z = 0.062993339788

    # x.pose.orientation.x = 0.499808452717
    # x.pose.orientation.y = 0.500224158554
    # x.pose.orientation.z = 0.500200764333
    # x.pose.orientation.w = 0.499766442604
    # x.pose.position.x = 0.198593318728
    # x.pose.position.y = 0.111470015393
    # x.pose.position.z = 0.0579882904131


    x.pose.position.x = 0.153060058338
    x.pose.position.y = 0.364150680179
    x.pose.position.z = 0.201367325843
    x.pose.orientation.x = 0.661078444373
    x.pose.orientation.y = 0.610665976707
    x.pose.orientation.z = 0.206114152065
    x.pose.orientation.w = 0.384160528419


    group.set_pose_target(x)
    group.set_planning_time(5)
    group.set_num_planning_attempts(100)
    group.allow_replanning(True)

    # set constraint

    path_constraint = Constraints()
    path_constraint.name = "boundary_constraint"
    position_constraint = PositionConstraint()
    position_constraint.header.frame_id = robot.get_planning_frame()
    position_constraint.link_name = group.get_end_effector_link()
    #position_constrint.target_point_offset =

    # define the bounding volume of the constraint as a box around the table
    constraint_region = BoundingVolume()
    primitive = SolidPrimitive()
    primitive.type = 1
    table_x_max = 0.8
    table_y_max = 0.8
    z_max = 0.4
    primitive.dimensions = [table_x_max*2, table_y_max*2, z_max*2]
    constraint_region.primitives.append(primitive)

    position_constraint.constraint_region = constraint_region
    path_constraint.position_constraints.append(position_constraint)

    group.set_path_constraints(path_constraint)

    plan = group.plan()

    group.clear_path_constraints()


    hello = raw_input("please input\n")
    rospy.sleep(2)
    scene.remove_world_object("master_chef_can")
    rospy.sleep(2)


def quarternion_to_matrix(x, y, z, w):
    ori_matrix = np.zeros((3,3))

    ori_matrix[0,0] = 1 - 2*y*y - 2*z*z
    ori_matrix[0,1] = 2*x*y - 2*z*w
    ori_matrix[0,2] = 2*x*z + 2*y*w
    ori_matrix[1,0] = 2*x*y + 2*z*w
    ori_matrix[1,1] = 1-2*x*x - 2*z*z
    ori_matrix[1,2] = 2*y*z - 2*x*w
    ori_matrix[2,0] = 2*x*z - 2*y*w
    ori_matrix[2,1] = 2*y*z + 2*x*w
    ori_matrix[2,2] = 1-2*x*x - 2*y*y
    return ori_matrix

def matrix_to_quarternion(ori_matrix):
    w = np.sqrt(1+ori_matrix[0,0]+ori_matrix[1,1]+ori_matrix[2,2])/2
    x = (ori_matrix[2,1] - ori_matrix[1,2]) / (4*w)
    y = (ori_matrix[0,2] - ori_matrix[2,0]) / (4*w)
    z = (ori_matrix[1,0] - ori_matrix[0,1]) / (4*w)
    return (x, y, z, w)

def gripper_retreat(x, retreat_step_size):
    """
    function:
    ===========================================
    given start pose, first setup the arm in that pose. Then retreat the gripper by some amount.

    details:
    ===========================================
    1. plan and execute the arm to the given pose.
    2. retreat the arm for retreat_step_size

    input:
    ===========================================
    x: arm pose for planning
        format: geometry_msgs/PoseStamped

    retreat_step_size: step size for retreating
        format: float
    output:
    ===========================================
    """
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    4. plan gripper closing
    5. plan a path to drop pose
    6 plan gripper openning

    input:
    ===========================================
    x0, x_obj, x_grasp, x_drop: start, object, grasp and drop poses.

    format: geometry_msgs/PoseStamped

    output:
    ===========================================

    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm_to_gripper_base"
    group_gripper_name = "robot_gripper"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())
    #sys.exit(1)
    # test data
    #scene.remove_world_object(model_name)
    #rospy.sleep(2)
    """
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.1
    p.pose.position.y = 0.3
    p.pose.position.z = 0.1#0.1
    p.pose.orientation.x = 0.
    p.pose.orientation.y = 0.
    p.pose.orientation.z = 0.
    p.pose.orientation.w = 1.

    scene.add_mesh(name=model_name, pose=p, \
                    filename='/root/ocrtoc_materials/models/%s/meshes/textured.obj' % (model_name), size=(1, 1, 1))
    """
    # plan to the target position first
    pre_grasp_pose = Pose()
    """
    pre_grasp_pose.position.x = 0.176507042334
    #pre_grasp_pose.position.y = 0.262969680106
    pre_grasp_pose.position.y = 0.06
    pre_grasp_pose.position.z = 0.145262507441
    pre_grasp_pose.orientation.x = -0.00404578006085
    pre_grasp_pose.orientation.y = -0.699499529354
    pre_grasp_pose.orientation.z = 0.00302342393036
    pre_grasp_pose.orientation.w = 0.714615210449
    """
    x = group.get_current_pose()
    print('header of pose:')
    print(x.header)
    pre_grasp_pose = x.pose
    # now we assume it returns a local frame relative to object
    print('==end_effector name:==')
    print(group.get_end_effector_link())
    x.pose = pre_grasp_pose

    group.set_pose_target(x)
    group.set_planning_time(5)
    group.set_num_planning_attempts(100)
    group.allow_replanning(True)
    plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    group.clear_path_constraints()

    hello = raw_input("please input\n")
    rospy.sleep(2)


    # ** retreat the gripper
    # calcualte the retreating vector using the orientation of the pose
    rot_matrix = quarternion_to_matrix(pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, \
                                       pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w)
    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    #retreat_vec = np.array([0.,0,1]).dot(rot_matrix)
    retreat_vec = rot_matrix.dot(np.array([0.,0.,-1.]))
    retreat_step_size = 0.2
    current_retreat_step = 0.

    current_retreat_step = 0.1
    print('current retreat step: %f' % (current_retreat_step))
    # obtain the retreated grasping pose
    current_pose = pre_grasp_pose_np + current_retreat_step * retreat_vec
    pre_grasp_pose.position.x = current_pose[0]
    pre_grasp_pose.position.y = current_pose[1]
    pre_grasp_pose.position.z = current_pose[2]
    x = group.get_current_pose()
    # now we assume it returns a local frame relative to object
    x.pose = pre_grasp_pose

    group.set_pose_target(x)
    group.set_planning_time(5)
    group.set_num_planning_attempts(100)
    group.allow_replanning(True)
    plan = group.plan()
    #plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    group.clear_path_constraints()
    print("move_group plan result: ")
    print(plan)

    hello = raw_input("please input\n")
    rospy.sleep(2)


def gripper_openning_plan():
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    4. plan gripper closing
    5. plan a path to drop pose
    6 plan gripper openning

    input:
    ===========================================
    x0, x_obj, x_grasp, x_drop: start, object, grasp and drop poses.

    format: geometry_msgs/PoseStamped

    output:
    ===========================================

    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #group_arm_name = "robot_arm_to_gripper_base"
    group_gripper_name = "robot_gripper"
    group = moveit_commander.MoveGroupCommander(group_gripper_name)

    print("============ Printing robot state")
    print(robot.get_current_state())
    #print('============ Printing gripper joint')
    #print(group.get_current_joint_values())

    init_joint_value = 0.
    joint_name = group.get_joints()
    joint_value = group.get_current_joint_values()
    x = dict()
    for i in range(len(joint_value)):
        if joint_name[i] == 'robotiq_2f_85_right_driver_joint':
            x[joint_name[i]] = joint_value[i]

    # now we assume it returns a local frame relative to object
    print('============ Printing robot joint')
    print(x)
    x['robotiq_2f_85_right_driver_joint'] = init_joint_value
    joint_name = group.get_joints()
    #print('============ Printing robot joint name')
    #print(joint_name)
    # set the driver_joint to be 0.4

    group.set_joint_value_target(x)
    group.set_planning_time(5)
    group.set_num_planning_attempts(100)

    #group.set_goal_tolerance(goal_tol)

    group.allow_replanning(True)

    # set constraint

    plan = group.plan()
    #plan = group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    #group.stop()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    group.clear_pose_targets()
    group.clear_path_constraints()

    hello = raw_input("please input\n")
    rospy.sleep(2)

from control_msgs.msg import GripperCommandActionGoal
def gripper_openning():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    # publish this so that gazebo can move accordingly
    # Example: control ur5e by topic
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = 0.
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for openning")
    rospy.sleep(1.0)

def gripper_closing_plan():
    """
    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    #group_arm_name = "robot_arm_to_gripper_base"
    group_gripper_name = "robot_gripper"
    group = moveit_commander.MoveGroupCommander(group_gripper_name)

    print("============ Printing robot state")
    print(robot.get_current_state())
    #print('============ Printing gripper joint names')
    #print(group.get_joints())
    #print('============ Printing gripper joint')
    #print(group.get_current_joint_values())

    init_joint_value = 0.5

    joint_name = group.get_joints()
    joint_value = group.get_current_joint_values()
    x = dict()
    for i in range(len(joint_value)):
        if joint_name[i] == 'robotiq_2f_85_right_driver_joint':
            x[joint_name[i]] = joint_value[i]

    # now we assume it returns a local frame relative to object
    print('============ Printing robot joint')
    print(x)
    joint_value_decrease_step = 0.02


    for planning_attempt_i in range(20):
        x['robotiq_2f_85_right_driver_joint'] = init_joint_value - joint_value_decrease_step * planning_attempt_i
        # set the driver_joint to be 0.4

        group.set_joint_value_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)

        #group.set_goal_tolerance(goal_tol)

        group.allow_replanning(True)

        # set constraint

        plan = group.plan()
        #plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        #group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:
            break

    hello = raw_input("please input\n")
    rospy.sleep(2)

def gripper_closing():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    # publish this so that gazebo can move accordingly
    # Example: control ur5e by topic
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = 0.6
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for closing")
    rospy.sleep(1.0)

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def plan_grasp(x0, x_obj, x_grasp, x_drop, model_name = None):
    """
    function:
    ===========================================
    given start pose, object pose, grasp pose and drop pose, return a plan for execution

    details:
    ===========================================
    1. generate pre-grasp pose
    2. plan a path to pre-grasp pose
    3. plan a cartesian path to grasp pose
    4. plan gripper closing
    5. plan a path to drop pose
    6 plan gripper openning

    input:
    ===========================================
    x0, x_obj, x_grasp, x_drop: start, object, grasp and drop poses.

    format: geometry_msgs/PoseStamped

    output:
    ===========================================

    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm_to_gripper_base"
    group_gripper_name = "robot_gripper"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())
    #sys.exit(1)

    #model_name = "large_marker"

    # test data
    scene.remove_world_object(model_name)
    rospy.sleep(2)

    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.1
    p.pose.position.y = 0.3
    p.pose.position.z = 0.031#0.1
    p.pose.orientation.x = 0.
    p.pose.orientation.y = 0.
    p.pose.orientation.z = 0.
    p.pose.orientation.w = 1.

    #scene.add_mesh(name=model_name, pose=p, \
    #                filename='/root/ocrtoc_materials/models/%s/meshes/textured.obj' % (model_name), size=(1, 1, 1))


    #p.pose.orientation.w = 1.
    #scene.add_mesh(name='a_cups', pose=p, \
    #                filename='/root/ocrtoc_materials/models/a_cups/meshes/textured.obj', size=(1, 1, 1))
    # use pcd service to generate grasp pose
    """
    from grasp_srv.msg import ObjectPoses, Grasps, GlobalGraspPose
    from grasp_srv.srv import GraspGen, GraspGenResponse

    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    obj_pose = Pose()
    obj_pose.position = p.pose.position
    obj_pose.orientation = p.pose.orientation
    grasp_srv_request_poses.object_poses.append(obj_pose)
    try:
        grasp_gen = rospy.ServiceProxy('grasp_gen', GraspGen)
        resp1 = grasp_gen(grasp_srv_request_poses)
        print('pose generation result:')
        print(resp1)
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        sys.exit(1)
    grasp_poses = resp1.grasps.global_grasp_poses[0].grasp_poses
    pre_grasp_poses = resp1.grasps.global_grasp_poses[0].pre_grasp_poses

    grasp_pose = grasp_poses[0]
    pre_grasp_pose = pre_grasp_poses[0]

    # rotation of grasp pose to correct the orientation
    orientation = pre_grasp_pose.orientation
    ori_matrix = np.zeros((3,3))
    ori_matrix = quarternion_to_matrix(orientation.x, orientation.y, orientation.z, orientation.w)

    #from scipy.spatial.transform import Rotation as R
    #ori_matrix = R.from_quat([0, 0, np.sin(np.pi/4), np.cos(np.pi/4)])
    #ori_matrix = ori_matrix.as_matrix()

    #frame_rotation_matrix = [[0., 0., 1.],
    #                         [0., 1., 0.],
    #                         [-1., 0., 0.]]
    frame_rotation_matrix = [[1., 0., 0.],
                             [0., 1., 0.],
                             [0., 0., 1.]]

    frame_rotation_matrix = np.array(frame_rotation_matrix)
    ori_matrix = ori_matrix.dot(frame_rotation_matrix)
    x, y, z, w = matrix_to_quarternion(ori_matrix)
    orientation.x = x
    orientation.y = y
    orientation.z = z
    orientation.w = w
    pre_grasp_pose.orientation = orientation



    print("============ Printing generated grasp pose")
    print(pre_grasp_pose)
    """


    pre_grasp_pose = Pose()
    pre_grasp_pose.position.x = 0.176507042334
    pre_grasp_pose.position.y = 0.302969680106#0.262969680106
    pre_grasp_pose.position.z = 0.046262507441
    pre_grasp_pose.orientation.x = -0.00404578006085
    pre_grasp_pose.orientation.y = -0.699499529354
    pre_grasp_pose.orientation.z = 0.00302342393036
    pre_grasp_pose.orientation.w = 0.714615210449

    #print("============ Printing robot state")
    #print(robot.get_current_state())
    #print('============ Printing robot pose')
    #print(group.get_current_pose())

    print('============ openning gripper...')
    gripper_openning()
    group.clear_pose_targets()

    #** stage 1: generate pre-grasp **
    #** stage 2: plan cartesian path to grasp pose **
    print('============ move arm...')
    x = group.get_current_pose()
    # now we assume it returns a local frame relative to object
    x.pose = pre_grasp_pose

    #print('============ Printing robot target pose')
    #print(x)
    goal_joint_tol = group.get_goal_joint_tolerance()
    goal_position_tol = group.get_goal_position_tolerance()
    goal_orientation_tol = group.get_goal_orientation_tolerance()

    # calcualte the retreating vector using the orientation of the pose
    rot_matrix = quarternion_to_matrix(pre_grasp_pose.orientation.x, pre_grasp_pose.orientation.y, \
                                       pre_grasp_pose.orientation.z, pre_grasp_pose.orientation.w)
    pre_grasp_pose_np = np.array([pre_grasp_pose.position.x, pre_grasp_pose.position.y, pre_grasp_pose.position.z])
    retreat_vec = rot_matrix.dot(np.array([0.,0,-1]))
    retreat_step_size = 0.05
    current_retreat_step = 0.

    for planning_attempt_i in range(10):
        #print('planning attempt: %d' % (planning_attempt_i))
        #print('goal_joint_tolerance: %f' % (goal_joint_tol))
        #print('goal_position_tolerance: %f' % (goal_position_tol))
        #print('goal_orientation_tolerance: %f' % (goal_orientation_tol))
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = pre_grasp_pose_np + current_retreat_step * retreat_vec
        pre_grasp_pose.position.x = current_pose[0]
        pre_grasp_pose.position.y = current_pose[1]
        pre_grasp_pose.position.z = current_pose[2]
        x.pose = pre_grasp_pose

        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)

        #group.set_goal_tolerance(goal_tol)
        group.set_goal_position_tolerance(goal_position_tol)

        group.allow_replanning(True)

        # set constraint
        """
        path_constraint = Constraints()
        path_constraint.name = "boundary_constraint"
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = robot.get_planning_frame()
        position_constraint.link_name = group.get_end_effector_link()
        #position_constrint.target_point_offset =

        # define the bounding volume of the constraint as a box around the table
        constraint_region = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = 1
        table_x_max = 0.8
        table_y_max = 0.8
        z_max = 0.4
        primitive.dimensions = [table_x_max*2, table_y_max*2, z_max*2]
        constraint_region.primitives.append(primitive)
        position_constraint.constraint_region = constraint_region
        path_constraint.position_constraints.append(position_constraint)
        group.set_path_constraints(path_constraint)
        """

        plan = group.plan()
        #plan = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        #group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        group.clear_pose_targets()
        group.clear_path_constraints()
        #print("move_group plan result: ")
        #print(plan)
        #print('=======================')
        #print('joint trajectory:')
        #print(plan.joint_trajectory)
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            arm_cmd_pub = rospy.Publisher(
                rospy.resolve_name('arm_controller/command'),
                JointTrajectory, queue_size=10)

            rospy.sleep(1.0) # allow publisher to initialize

            # publish this so that gazebo can move accordingly
            # Example: control ur5e by topic
            arm_cmd = plan.joint_trajectory
            # arm_cmd.joint_names = group.get_active_joints()
            # waypoint = JointTrajectoryPoint()
            # waypoint.positions = [-0.68, -0.63, 0.69, -0.88, -0.53, -0.19]
            # waypoint.time_from_start.secs = 2.0
            # arm_cmd.points.append(waypoint)
            arm_cmd_pub.publish(arm_cmd)
            rospy.loginfo("Pub arm_cmd")
            rospy.sleep(1.0)
            break
        else:
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0

    hello = raw_input("please input\n")
    rospy.sleep(2)

    #** stage 3: close gripper to pick up object **
    print('============ closing gripper...')

    gripper_closing()

    #group.detach_object(model_name)
    hello = raw_input("please input\n")
    rospy.sleep(2)
    #scene.remove_world_object(model_name)
    #rospy.sleep(2)


import copy
def place():
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm_to_gripper_base"
    group_gripper_name = "robot_gripper"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    group.clear_pose_targets()

    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    # compute the cartesian path for the end effector to go up
    target_pose_up = group.get_current_pose().pose
    current_z = target_pose_up.position.z
    target_z = 0.5
    step_sz = 0.2
    num_steps = int((target_z - current_z) / step_sz)
    waypoints = [target_pose_up]
    for i in range(num_steps):
        target_pose_up.position.z += step_sz
        waypoints.append(copy.deepcopy(target_pose_up))
    target_pose_up.position.z = target_z
    waypoints.append(copy.deepcopy(target_pose_up))

    path_up, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0., avoid_collisions=False)

    # Gazebo follow the waypoints
    arm_cmd = path_up.joint_trajectory
    #arm_cmd.joint_names = group.get_active_joints()
    # waypoint = JointTrajectoryPoint()
    # waypoint.positions = [-0.68, -0.63, 0.69, -0.88, -0.53, -0.19]
    # waypoint.time_from_start.secs = 2.0
    # arm_cmd.points.append(waypoint)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    group.clear_pose_targets()



    target_pose = Pose()
    target_pose.position.x = 0.776507042334
    target_pose.position.y = 0.102969680106#0.262969680106
    target_pose.position.z = 0.046262507441
    target_pose.orientation.x = -0.00404578006085
    target_pose.orientation.y = -0.699499529354
    target_pose.orientation.z = 0.00302342393036
    target_pose.orientation.w = 0.714615210449
    # move to the desire location in the air
    target_pose_xy = copy.deepcopy(target_pose)
    target_pose_xy.position.z = group.get_current_pose().pose.position.z
    waypoints = [group.get_current_pose().pose, target_pose_xy]

    path, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0., avoid_collisions=False)
    arm_cmd = path_up.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    # move down
    waypoints = [group.get_current_pose().pose, target_pose]
    path, fraction = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0., avoid_collisions=False)
    arm_cmd = path_up.joint_trajectory

    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")

    #** open gripper to release object **
    gripper_openning()




from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    #plan_arm_state_to_state(None, None)
    #ik_generation(None)
    #plan_arm_pose_to_pose_with_constraints(None, None)
    plan_grasp(None, None, None, None, model_name=sys.argv[1])
    place()
    #gripper_retreat(None, 0.3)
    #gripper_openning()

if __name__ == "__main__":
    main()

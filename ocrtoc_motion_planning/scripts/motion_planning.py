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






from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive
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
import tf

def object_transformation(object_pose1, object_pose2):
    # given two poses, compute the transformation
    R1 = quaternion_matrix([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
    R2 = quaternion_matrix([object_pose2.orientation.x,object_pose2.orientation.y,object_pose2.orientation.z,object_pose2.orientation.w])
    R = R2.dot(tf.transformations.inverse_matrix(R1))
    angles = tf.transformations.euler_from_matrix(R)
    trans = np.array([object_pose2.position.x - object_pose1.position.x, object_pose2.position.y - object_pose1.position.y, object_pose2.position.z - object_pose1.position.z])
    M = tf.transformations.compose_matrix(angles=angles, translate=trans)
    return M



def plan_grasp(obj_pose1, obj_pose2):
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
    obj_pose1, obj_pose2

    format: geometry_msgs/PoseStamped

    output:
    ===========================================

    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())
    # specify pose from json
    import json
    scene_pose = open('../scene_info/1-1.json', 'r')
    scene_pose = json.load(scene_pose)
    model_name = 'pudding_box'
    # find the model_name in the scene_pose
    for i in range(len(scene_pose)):
        if scene_pose[i]['name'] == model_name:
            # record the pose
            pose = np.array(scene_pose[i]['pose_world'])
            scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(pose)
            R = tf.transformations.euler_matrix(*angles)
            quat = tf.transformations.quaternion_from_matrix(R)
            p = Pose()
            p.position.x = pose[0,3]
            p.position.y = pose[1,3]
            p.position.z = pose[2,3]
            p.orientation.x = quat[0]
            p.orientation.y = quat[1]
            p.orientation.z = quat[2]
            p.orientation.w = quat[3]
            break

    # use pcd service to generate grasp pose

    from grasp_srv.msg import ObjectPoses, Grasps
    from grasp_srv.srv import GraspGen, GraspGenResponse

    rospy.wait_for_service('grasp_gen')
    # generate message
    grasp_srv_request_poses = ObjectPoses()
    grasp_srv_request_poses.object_names.append(model_name)
    obj_pose = p
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

    print("============ Printing generated grasp pose")
    print(pre_grasp_pose)

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
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            grasp_plan = plan
            break
        else:
            grasp_plan = None
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0

    # pre_grasp to grasp
    waypoints = [pre_grasp_pose, grasp_pose]
    cartesian_plan = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0., avoid_collisions=False)

    grasp_plan_end_state = robot.get_current_state()
    grasp_plan_end_state.joint_state.header = cartesian_plan.joint_trajectory.header
    grasp_plan_end_state.joint_state.name = cartesian_plan.joint_trajectory.joint_names
    grasp_plan_end_state.joint_state.position = cartesian_plan.joint_trajectory.points[-1].positions
    grasp_plan_end_state.joint_state.velocity = cartesian_plan.joint_trajectory.points[-1].velocities
    grasp_plan_end_state.joint_state.effort = cartesian_plan.joint_trajectory.points[-1].effort

    # obtain transformation matrix of object
    M = object_transformation(obj_pose, object_pose2)

    # transform arm pose into the desire one
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(M)
    q = tf.transformations.quaternion_from_euler(angles[0], angles[1], angles[2])

    # apply q to the current pose
    target_pose = grasp_pose
    position = np.array([target_pose.position.x,target_pose.position.y,target_pose.position.z])
    position = position + trans
    orientation = np.array([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z, target_pose.orientation.w])
    orientation = tf.transformations.quaternion_multiply(q, orientation)
    target_pose.position.x = position[0]
    target_pose.position.y = position[1]
    target_pose.position.z = position[2]
    target_pose.orientation.x = orientation[0]
    target_pose.orientation.y = orientation[1]
    target_pose.orientation.z = orientation[2]
    target_pose.orientation.w = orientation[3]
    place_plan = place(start_state=grasp_plan_end_state, target_pose)


    # ** execution of plan **
    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = grasp_plan.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)

    # pregrasp to grasp
    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = cartesian_plan.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)


    #** stage 3: close gripper to pick up object **
    print('============ closing gripper...')
    gripper_closing()

    #group.detach_object(model_name)
    hello = raw_input("please input\n")
    rospy.sleep(2)
    arm_cmd = place_plan.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)

import copy
def place(start_state, target_pose):
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    group.clear_pose_targets()


    for planning_attempt_i in range(10):
        # obtain the retreated grasping pose
        start_state = group.get_current_state()
        start_state.joint_state


        group.set_start_state()
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

def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    #plan_arm_state_to_state(None, None)
    #ik_generation(None)
    #plan_arm_pose_to_pose_with_constraints(None, None)
    plan_grasp(None, None, None, None)
    #gripper_retreat(None, 0.3)
    #gripper_openning()

if __name__ == "__main__":
    main()

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
    group_arm_name = "robot_arm"
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
    retreat_vec = rot_matrix.dot(np.array([-1.,0.,0.]))
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

def tf_decompose(obj_tf):
    """
    decompose the TF matrix. Return the scale and transformed Pose

    scale: vector of size 3
    pose: geometry_msgs.msg/Pose
    """
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(obj_tf)
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
    return scale, p

def object_transformation(object_pose1, object_pose2):
    # given two poses, compute the transformation
    t1 = np.array([object_pose1.position.x,object_pose1.position.y,object_pose1.position.z])
    r1 = np.array([object_pose1.orientation.x,object_pose1.orientation.y,object_pose1.orientation.z,object_pose1.orientation.w])
    r1 = tf.transformations.euler_from_quaternion(r1)
    T1 = tf.transformations.compose_matrix(translate=t1, angles=r1)
    t2 = np.array([object_pose2.position.x,object_pose2.position.y,object_pose2.position.z])
    r2 = np.array([object_pose2.orientation.x,object_pose2.orientation.y,object_pose2.orientation.z,object_pose2.orientation.w])
    r2 = tf.transformations.euler_from_quaternion(r2)
    T2 = tf.transformations.compose_matrix(translate=t2, angles=r2)
    T = T2.dot(tf.transformations.inverse_matrix(T1))

    return T



def plan_grasp(model_name, obj_tf_1, obj_tf_2):
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
    obj_tf_1, obj_tf_2

    format: geometry_msgs/Pose

    output:
    ===========================================

    """
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_arm_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_arm_name)
    print('end effector:')
    print(group.get_end_effector_link())

    # obtain pose from tf input
    scale, obj_pose1 = tf_decompose(obj_tf_1)
    _, obj_pose2 = tf_decompose(obj_tf_2)

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
    retreat_vec = rot_matrix.dot(np.array([-1.,0,0]))
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
    # generate a number of waypoints in between

    """
    inter_pose = copy.deepcopy(pre_grasp_pose)
    pre_pose_xyz = np.array([inter_pose.position.x,inter_pose.position.y,inter_pose.position.z])
    target_pose_xyz = np.array([grasp_pose.position.x,grasp_pose.position.y,grasp_pose.position.z])
    num_steps = 10
    step = (target_pose_xyz - pre_pose_xyz) / num_steps
    for i in range(1,num_steps):
        pose_xyz = pre_pose_xyz + i*step
        inter_pose = copy.deepcopy(inter_pose)
        inter_pose.position.x = pose_xyz[0]
        inter_pose.position.y = pose_xyz[1]
        inter_pose.position.z = pose_xyz[2]
        waypoints = [copy.deepcopy(inter_pose)]
    cartesian_plan, factor = group.compute_cartesian_path(waypoints, eef_step=0.01, jump_threshold=0., avoid_collisions=False)
    print('cartesian plan:')
    print(cartesian_plan)
    """

    # execute plan
    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)
    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = grasp_plan.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    # pregrasp to grasp

    rot_matrix = quarternion_to_matrix(grasp_pose.orientation.x, grasp_pose.orientation.y, \
                                       grasp_pose.orientation.z, grasp_pose.orientation.w)
    grasp_pose_np = np.array([grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z])
    retreat_vec = rot_matrix.dot(np.array([-1.,0,0]))
    retreat_step_size = 0.05
    current_retreat_step = 0.

    for planning_attempt_i in range(10):
        current_retreat_step = retreat_step_size * planning_attempt_i
        print('current retreat step: %f' % (current_retreat_step))
        # obtain the retreated grasping pose
        current_pose = grasp_pose_np + current_retreat_step * retreat_vec
        grasp_pose.position.x = current_pose[0]
        grasp_pose.position.y = current_pose[1]
        grasp_pose.position.z = current_pose[2]
        x.pose = grasp_pose

        group.set_pose_target(x)
        group.set_planning_time(5)
        group.set_num_planning_attempts(100)
        group.allow_replanning(True)
        plan = group.plan()
        group.clear_pose_targets()
        group.clear_path_constraints()
        if plan.joint_trajectory.points:  # True if trajectory contains points
            #if plan:
            cartesian_plan = plan
            break
        else:
            cartesian_plan = None
            print('planning failed. Another attempt is tried...')
            #goal_position_tol = goal_position_tol * 4.0





    arm_cmd = cartesian_plan.joint_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    print('============ closing gripper...')
    gripper_closing()

    # define grasp_plan_end_state to connect plan
    grasp_plan_end_state = robot.get_current_state()
    grasp_plan_end_state.joint_state.header = cartesian_plan.joint_trajectory.header
    grasp_plan_end_state.joint_state.name = cartesian_plan.joint_trajectory.joint_names
    grasp_plan_end_state.joint_state.position = cartesian_plan.joint_trajectory.points[-1].positions
    grasp_plan_end_state.joint_state.velocity = cartesian_plan.joint_trajectory.points[-1].velocities
    grasp_plan_end_state.joint_state.effort = cartesian_plan.joint_trajectory.points[-1].effort

    # obtain transformation matrix of object
    T = object_transformation(obj_pose1, obj_pose2)

    # transform arm pose into the desire one
    target_pose = grasp_pose
    t = np.array([target_pose.position.x,target_pose.position.y,target_pose.position.z])
    r = np.array([target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w])
    r = tf.transformations.euler_from_quaternion(r)
    T_grasp = tf.transformations.compose_matrix(translate=t, angles=r)
    T_target_grasp = T.dot(T_pose)
    scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(T_target_grasp)
    target_pose.position.x = trans[0]
    target_pose.position.y = trans[1]
    target_pose.position.z = trans[2]
    orientation = tf.transformations.quaternion_from_euler(*angles)
    target_pose.orientation.x = orientation[0]
    target_pose.orientation.y = orientation[1]
    target_pose.orientation.z = orientation[2]
    target_pose.orientation.w = orientation[3]
    place_plan = place(start_state=grasp_plan_end_state, target_pose=target_pose)

    # ** execution of plan **


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

    import json
    scene_pose = open('../scene_info/1-1.json', 'r')
    scene_pose = json.load(scene_pose)
    model_name = 'pudding_box'
    # find the model_name in the scene_pose
    for i in range(len(scene_pose)):
        if scene_pose[i]['name'] == model_name:
            # record the pose
            start_tf = np.array(scene_pose[i]['pose_world'])
            scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(start_tf)
            break
    target_pose = Pose()
    position = np.array([0.25974133610725403, -0.3102521002292633, 0.009862901642918587])
    orientation = np.array([0.022348699698035056, -0.01139162625176719, -9.36981416203137e-05])

    # compose into TF matrix
    target_tf = tf.transformations.compose_matrix(scale=scale, shear=shear, angles=orientation, trans=position, persp=persp)
    plan_grasp(model_name, start_tf, target_tf)
    #gripper_retreat(None, 0.3)
    #gripper_openning()

if __name__ == "__main__":
    main()

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

import moveit_connection

robot = moveit_connection.robot
group = moveit_connection.group
scene = moveit_connection.scene
gripper_open_value = 0.
gripper_close_value = .085  # 0 -> 0.044  # rotated grasp pose need smaller value?
gripper_driver_name = 'robotiq_2f_85_left_driver_joint'
def gripper_openning(gripper_cmd_pub):
    #gripper_cmd_pub = rospy.Publisher(
    #    rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
    #    GripperCommandActionGoal, queue_size=10)
    
    # real stage moveit does not know the current gripper status
    # # check if the current gripper is open, if not, then open it
    # state = robot.get_current_state().joint_state
    # # go over each name to find the gripper driver
    # for i in range(len(state.name)):
    #     if state.name[i] == gripper_driver_name:
    #         if np.abs(state.position[i] - gripper_open_value) <= 0.001:
    #             return  # consider as open, no need to open
    # # otherwise open

    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = gripper_open_value
    gripper_cmd.goal.command.max_effort = 50.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for openning")
    gripper_block_until_near(gripper_open_value)


def gripper_closing(gripper_cmd_pub):
    # gripper_cmd_pub = rospy.Publisher(
    #     rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
    #     GripperCommandActionGoal, queue_size=10)
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = gripper_close_value # sapien  # 0.6 # Gaezebo
    gripper_cmd.goal.command.max_effort = 90.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for closing")
    gripper_block_until_near(gripper_close_value)


def gripper_closing_and_store_pose(gripper_cmd_pub, model_name, global_grasp_pose, grasp_id):
    close_value = gripper_close_value  # 0 -> 0.044
    # gripper_cmd_pub = rospy.Publisher(
    #     rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
    #     GripperCommandActionGoal, queue_size=10)

    # publish this so that gazebo can move accordingly
    # Example: control ur5e by topic
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = close_value # sapien  # 0.6 # Gaezebo
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for closing")
    rospy.sleep(1.0)
    # publish 
    store_pose_based_on_gripper(global_grasp_pose, grasp_id)

def detect_gripper_closing():
    # detect after grasping, the joint position. If it is 90% threshold to the value fed, then considered as FAILURE
    joint_state = robot.get_current_state().joint_state
    # check the driver joint value
    gripper_driver_i = -1
    for i in range(len(joint_state.name)):
        if joint_state.name[i] == 'robotiq_2f_85_left_driver_joint':
            gripper_driver_i = i
            break
    ratio = joint_state.position[gripper_driver_i] / gripper_close_value
    threshold = 0.9
    print('ratio: %f' % (ratio))
    if ratio >= threshold:
        print('#####gripper close FAILURE!!!')
        return False  # gripper close failure
    else:
        print('#####gripper close SUCCESS:)')
        return True  # gripper close success        


def store_pose_based_on_gripper(global_grasp_pose, grasp_id):
    gripper_closed = detect_gripper_closing()
    if not gripper_closed:
        # the gripper closes too much, consider as a Failure
        pass
    else:        
        # TODO: here or after placing? detect if object falled
        from grasp_srv.msg import SaveGrasp
        save_grasp_pub = rospy.Publisher(
        rospy.resolve_name('save_grasp'), SaveGrasp, queue_size=10)
        rospy.sleep(1.0) # allow publisher to initialize
        save_grasp_msg = SaveGrasp()
        save_grasp_msg.global_grasp_pose = global_grasp_pose
        save_grasp_msg.grasp_ids = [grasp_id]
        save_grasp_pub.publish(save_grasp_msg)
        rospy.loginfo("publishing save_grasp message...")
        rospy.sleep(1.0)




def execute_plan(pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory, model_name=None, global_grasp_pose=None, grasp_id=None):
    # ** execution of plan **
    # execute plan for grasping
    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    hello = raw_input("please input\n")
    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = pre_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    arm_cmd = pre_to_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)
    hello = raw_input("please input\n")
    rospy.sleep(2)

    print('============ closing gripper...')
    gripper_closing(gripper_cmd_pub)
    #gripper_closing_and_store_pose(model_name, global_grasp_pose, grasp_id)

    # execute plan for placing
    hello = raw_input("please input\n")
    rospy.sleep(2)
    arm_cmd = place_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    hello = raw_input("please input\n")


    # detect if the object didn't slip, and the grasp is successful
    store_pose_based_on_gripper(global_grasp_pose, grasp_id)

    rospy.sleep(1.0)
    # open gripper
    print('=========== openning gripper...')
    gripper_openning(gripper_cmd_pub)
    # execute plan for resetting
    hello = raw_input("please input\n")
    rospy.sleep(2)
    arm_cmd = reset_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("Pub arm_cmd")
    rospy.sleep(1.0)



def arm_joint_value_at_target(joint_trajectory):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    robot_state = robot.get_current_state()
    joint_name_to_position = dict(zip(joint_trajectory.joint_names, joint_trajectory.points[-1].positions))
    state_name_to_position = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))

    near = True  # if all joint values are close enough
    near_threshold = 1e-2
    for key, value in joint_name_to_position.items():
        # find same name in state_name_to_position
        if key in state_name_to_position:
            # compare the difference between joint value
            if np.abs(state_name_to_position[key] - value) > near_threshold:
                near = False  # found one distsant value, not there yet
                break
    return near  # if distant, then False, if Near, then True



def arm_pose_at_target(target_pose, pos_dif_threshold=0.07, ori_dif_threshold=10):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    pose = group.get_current_pose().pose
    pos_dif, ori_dif = pose_distance(pose, target_pose)
    return pos_dif <= pos_dif_threshold and ori_dif <= ori_dif_threshold * np.pi/180



def arm_joint_to_pose_at_target(joint_trajectory, pos_dif_threshold=0.07, ori_dif_threshold=10):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    robot_state = robot_state_from_joint_trajectory(joint_trajectory)
    target_pose = fk(robot_state)

    return arm_pose_at_target(target_pose, pos_dif_threshold, ori_dif_threshold)



def gripper_joint_value_at_target(value):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    robot_state = robot.get_current_state()
    joint_name_to_position = {gripper_driver_name: value}
    state_name_to_position = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))

    near = True  # if all joint values are close enough
    near_threshold = 1e-2
    for key, value in joint_name_to_position.items():
        # find same name in state_name_to_position
        if key in state_name_to_position:
            # compare the difference between joint value
            if np.abs(state_name_to_position[key] - value) > near_threshold:
                near = False  # found one distsant value, not there yet
                break
    return near  # if distant, then False, if Near, then True


def arm_block_until_near(end_pose=None, joint_trajectory=None, pos_dif_threshold=0.02, ori_dif_threshold=10):
    while True:
        at_target = False
        if end_pose is not None:
            at_target = arm_pose_at_target(end_pose, pos_dif_threshold, ori_dif_threshold)
        else:
            at_target = arm_joint_to_pose_at_target(joint_trajectory, pos_dif_threshold, ori_dif_threshold)
        if at_target:
            break
        previous_pose = group.get_current_pose().pose
        rospy.sleep(2.)
        # check if the pose has changed within some time
        at_target = arm_pose_at_target(previous_pose, pos_dif_threshold, ori_dif_threshold)
        # if the same, then we already at the correct position
        if at_target:
            break

    rospy.sleep(1.)  # sleep to ensure we are actually there

def gripper_block_until_near(value):
    """
    while not gripper_joint_value_at_target(value):
        # loop until joint_value is at the target
        rospy.sleep(0.1)
    """
    rospy.sleep(1.8)


#pre_grasp_trajectory, pre_to_grasp_trajectory, post_grasp_trajectory, place_trajectory, reset_trajectory, model_name=None, global_grasp_pose=None, grasp_id=None


def add_velocity(joint_trajectory):
    # set small ratio to the time
    speedup = 0.5
    points = []
    # waypoint.time_from_start.secs = 2.0
    #for i in range(len(joint_trajectory.points)):
    #    joint_trajectory.points[i].time_from_start.secs = joint_trajectory.points[i].time_from_start.secs * speedup
    for i in range(len(joint_trajectory.points)-1):
        if i % int(1 / speedup) == 0:
            points.append(joint_trajectory.points[i])
    points.append(joint_trajectory.points[-1])
    joint_trajectory.points = points
    return joint_trajectory


def execute_plan_close_loop_with_pub(arm_cmd_pub, gripper_cmd_pub, args):
    # ** execution of plan **
    # open gripper before path
    gripper_openning(gripper_cmd_pub)
    rospy.loginfo("openning gripper...")

    arm_cmd = args['pre_grasp_trajectory']
    print('before add_velocity: len(arm_cmd.points): %d' % (len(arm_cmd.points)))
    #arm_cmd = add_velocity(arm_cmd)
    print('after add_velocity: len(arm_cmd.points): %d' % (len(arm_cmd.points)))

    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.005, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['pre_grasp_end_pose'])

    arm_cmd = args['pre_to_grasp_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp to grasp executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.0005, ori_dif_threshold=10)

    #arm_block_until_near(end_pose=args['grasp_end_pose'])

    rospy.loginfo("closing gripper...")
    gripper_closing(gripper_cmd_pub)
    #gripper_closing_and_store_pose(model_name, global_grasp_pose, grasp_id)

    # post_grasp
    arm_cmd = args['post_grasp_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("post_grasp executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.005, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['post_grasp_end_pose'])
    
    # execute plan for placing

    arm_cmd = args['place_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("place plan executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.005, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['place_end_pose'])


    # detect if the object didn't slip, and the grasp is successful
    store_pose_based_on_gripper(args['global_grasp_pose'], args['grasp_id'])

     # detect if gripper is closed
    gripper_success = detect_gripper_closing()
    # open gripper
    gripper_openning(gripper_cmd_pub)
    rospy.sleep(0.5)
    rospy.loginfo("openning gripper...")
    # execute plan for resetting    
 

    # post_place_execute
    arm_cmd = args['post_place_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("post-place plan executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.005, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['post_place_end_pose'])



    arm_cmd = args['reset_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("reset plan executing...")
    arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.005, ori_dif_threshold=10)
    return gripper_success






def execute_plan_close_loop_with_moveit(arm_cmd_pub, gripper_cmd_pub, args):
    from moveit_msgs.msg import RobotTrajectory
    # ** execution of plan **
    # open gripper before path
    gripper_openning(gripper_cmd_pub)
    rospy.loginfo("openning gripper...")

    arm_cmd = args['pre_grasp_trajectory']
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.01, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['pre_grasp_end_pose'])

    arm_cmd = args['pre_to_grasp_trajectory']
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp to grasp executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.001, ori_dif_threshold=10)

    #arm_block_until_near(end_pose=args['grasp_end_pose'])

    rospy.loginfo("closing gripper...")
    gripper_closing(gripper_cmd_pub)
    #gripper_closing_and_store_pose(model_name, global_grasp_pose, grasp_id)

    # post_grasp
    arm_cmd = args['post_grasp_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("post_grasp executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.01, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['post_grasp_end_pose'])
    
    # execute plan for placing

    arm_cmd = args['place_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("place plan executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.01, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['place_end_pose'])


    # detect if the object didn't slip, and the grasp is successful
    store_pose_based_on_gripper(args['global_grasp_pose'], args['grasp_id'])

     # detect if gripper is closed
    gripper_success = detect_gripper_closing()
    # open gripper
    gripper_openning(gripper_cmd_pub)
    rospy.sleep(0.5)
    rospy.loginfo("openning gripper...")
    # execute plan for resetting    
 

    # post_place_execute
    arm_cmd = args['post_place_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("post-place plan executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.01, ori_dif_threshold=10)
    #arm_block_until_near(end_pose=args['post_place_end_pose'])



    arm_cmd = args['reset_trajectory']
    #arm_cmd = add_velocity(arm_cmd)
    # construct plan
    arm_traj = RobotTrajectory()
    arm_traj.joint_trajectory = arm_cmd
    #arm_cmd = add_velocity(arm_cmd)
    group.execute(arm_traj, wait=True)
    #arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("reset plan executing...")
    #arm_block_until_near(joint_trajectory=arm_cmd, pos_dif_threshold=0.01, ori_dif_threshold=10)
    return gripper_success
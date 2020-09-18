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
gripper_close_value = .044  # 0 -> 0.044  # rotated grasp pose need smaller value?
gripper_driver_name = 'robotiq_2f_85_left_driver_joint'
def gripper_openning():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = gripper_open_value
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for openning")
    gripper_block_until_near(gripper_open_value)


def gripper_closing():
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize
    gripper_cmd = GripperCommandActionGoal()
    gripper_cmd.goal.command.position = gripper_close_value # sapien  # 0.6 # Gaezebo
    gripper_cmd.goal.command.max_effort = 0.0
    gripper_cmd_pub.publish(gripper_cmd)
    rospy.loginfo("Pub gripper_cmd for closing")
    gripper_block_until_near(gripper_close_value)


def gripper_closing_and_store_pose(model_name, global_grasp_pose, grasp_id):
    close_value = gripper_close_value  # 0 -> 0.044
    gripper_cmd_pub = rospy.Publisher(
        rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
        GripperCommandActionGoal, queue_size=10)

    rospy.sleep(1.0) # allow publisher to initialize

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

def store_pose_based_on_gripper(global_grasp_pose, grasp_id):
    # detect after grasping, the joint position. If it is 90% threshold to the value fed, then considered as FAILURE
    joint_state = robot.get_current_state().joint_state
    # check the driver joint value
    gripper_driver_i = -1
    for i in range(len(joint_state.name)):
        if joint_state.name[i] == 'robotiq_2f_85_left_driver_joint':
            gripper_driver_i = i
            break
    ratio = joint_state.position[gripper_driver_i] / gripper_close_value
    print('ratio: %f' % (ratio))
    threshold = 0.9
    if ratio >= threshold:
        # the gripper closes too much, consider as a Failure
        print('#####gripper close FAILURE!!!')
        pass
    else:        
        # TODO: here or after placing? detect if object falled
        print('#####gripper close SUCCESS:)')
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
    gripper_closing()
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
    gripper_openning()
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
    near_threshold = 1e-3
    for key, value in joint_name_to_position.items():
        # find same name in state_name_to_position
        if key in state_name_to_position:
            # compare the difference between joint value
            if np.abs(state_name_to_position[key] - value) > near_threshold:
                near = False  # found one distsant value, not there yet
                break
    return near  # if distant, then False, if Near, then True

def gripper_joint_value_at_target(value):
    # check if current robot joint value is at target
    # TODO: also check velocity and acceleration?
    robot_state = robot.get_current_state()
    joint_name_to_position = {gripper_driver_name: value}
    state_name_to_position = dict(zip(robot_state.joint_state.name, robot_state.joint_state.position))

    near = True  # if all joint values are close enough
    near_threshold = 1e-3
    for key, value in joint_name_to_position.items():
        # find same name in state_name_to_position
        if key in state_name_to_position:
            # compare the difference between joint value
            if np.abs(state_name_to_position[key] - value) > near_threshold:
                near = False  # found one distsant value, not there yet
                break
    return near  # if distant, then False, if Near, then True


def arm_block_until_near(joint_trajectory):
    while not arm_joint_value_at_target(joint_trajectory):
        # loop until joint_value is at the target
        pass

def gripper_block_until_near(value):
    while not gripper_joint_value_at_target(value):
        # loop until joint_value is at the target
        pass


def execute_plan_close_loop(pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory, model_name=None, global_grasp_pose=None, grasp_id=None):
    # ** execution of plan **
    # execute plan for grasping
    arm_cmd_pub = rospy.Publisher(
        rospy.resolve_name('arm_controller/command'),
        JointTrajectory, queue_size=10)
    rospy.sleep(1.0) # allow publisher to initialize

    arm_cmd = pre_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp executing...")
    arm_block_until_near(arm_cmd)

    arm_cmd = pre_to_grasp_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("pre_grasp to grasp executing...")
    arm_block_until_near(arm_cmd)

    rospy.loginfo("closing gripper...")
    gripper_closing()
    #gripper_closing_and_store_pose(model_name, global_grasp_pose, grasp_id)

    # execute plan for placing

    arm_cmd = place_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("place plan executing...")
    arm_block_until_near(arm_cmd)


    # detect if the object didn't slip, and the grasp is successful
    store_pose_based_on_gripper(global_grasp_pose, grasp_id)
 
    # open gripper
    gripper_openning()
    rospy.loginfo("openning gripper...")
    # execute plan for resetting    
 
    arm_cmd = reset_trajectory
    arm_cmd_pub.publish(arm_cmd)
    rospy.loginfo("reset plan executing...")
    arm_block_until_near(arm_cmd)


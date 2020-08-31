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
    """
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

    hello = raw_input("please input\n")
    rospy.sleep(2)
    scene.remove_world_object("master_chef_can")
    rospy.sleep(2)
    """
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

def gripper_pose_to_arm_pose():
    pass


from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_group_python_interface_tutorial',
                    anonymous=True)
    #plan_arm_state_to_state(None, None)
    #ik_generation(None)
    plan_arm_pose_to_pose_with_constraints(None, None)


if __name__ == "__main__":
    main()

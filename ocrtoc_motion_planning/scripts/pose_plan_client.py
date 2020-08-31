#!/usr/bin/env python

from __future__ import print_function

import sys
import rospy
from motion_planning.srv import PosePlan, PosePlanResponse
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
from geometry_msgs.msg import Quaternion, Pose, PoseStamped, Point

import numpy as np

def pose_plan_client(x):
    rospy.wait_for_service('pose_plan')
    try:
        pose_plan = rospy.ServiceProxy('pose_plan', PosePlan)
        resp1 = pose_plan(x)
        print('planning result: %d' % (resp1.result))
        return resp1
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)


if __name__ == "__main__":
    rospy.init_node('grasp_plan_server')
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "robot_arm"
    group = moveit_commander.MoveGroupCommander(group_name)

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
    print("Requesting pose plan...")
    result = pose_plan_client(x)
    hello = raw_input("please input\n")
    rospy.sleep(2)
    scene.remove_world_object("master_chef_can")
    rospy.sleep(2)

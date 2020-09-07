#! /usr/bin/env python

import actionlib
import rospy

import ocrtoc_task.msg
from control_msgs.msg import GripperCommandActionGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MotionPlanner(object):
    def __init__(self, name):
        self.arm_cmd_pub = rospy.Publisher(
            rospy.resolve_name('arm_controller/command'),
            JointTrajectory, queue_size=10)
        self.gripper_cmd_pub = rospy.Publisher(
            rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
            GripperCommandActionGoal, queue_size=10)
        moveit_commander.roscpp_initialize(sys.argv)

    def grasp_plan(self, goal):


if __name__ == '__main__':
    #moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_planner')
    motion_planner = MotionPlanner(sys.argv[1])
    motion_planner
    rospy.spin()

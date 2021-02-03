#!/usr/bin/env python

from __future__ import print_function


from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommandActionGoal
import tf

import rospy

rospy.init_node('motion_planning_test',
                anonymous=True)
gripper_cmd_pub = rospy.Publisher(
    rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
    GripperCommandActionGoal, queue_size=10)

gripper_cmd = GripperCommandActionGoal()
gripper_cmd.goal.command.position = -0.5
gripper_cmd.goal.command.max_effort = 0.0
gripper_cmd_pub.publish(gripper_cmd)
rospy.loginfo("Pub gripper_cmd for openning")

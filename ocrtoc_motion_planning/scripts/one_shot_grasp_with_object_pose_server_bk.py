#!/usr/bin/env python

from __future__ import print_function

from motion_planning.srv import OneShotGraspPlanWithObjectPose, OneShotGraspPlanWithObjectPoseResponse
from motion_planning_functions import one_shot_grasp_with_object_pose

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.msg import GripperCommandActionGoal
from motion_planning_execution import execute_plan_close_loop_with_pub
import tf

import rospy

arm_cmd_pub = rospy.Publisher(
    rospy.resolve_name('arm_controller/command'),
    JointTrajectory, queue_size=10)
gripper_cmd_pub = rospy.Publisher(
    rospy.resolve_name('gripper_controller/gripper_cmd/goal'),
    GripperCommandActionGoal, queue_size=10)

def handle_grasp_plan(req):
    #scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(req.object_pose1)
    #scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(req.object_pose2)
    response = OneShotGraspPlanWithObjectPoseResponse()
    try:
        plan_res = one_shot_grasp_with_object_pose(req.object_name, req.object_scale, req.object_pose1, req.object_pose2)
    except:
        rospy.logerr("Grasp plan failed.")
        # plan is unsuccessful at some point
        response.result = response.FAILURE
    else:
        # plan is successful
        rospy.loginfo("Grasp plan successfully generated.")
        response.pre_grasp_trajectory = plan_res['pre_grasp_trajectory']
        response.pre_to_grasp_trajectory = plan_res['pre_to_grasp_trajectory']
        response.post_grasp_trajectory = plan_res['post_grasp_trajectory']
        response.place_trajectory = plan_res['place_trajectory']
        response.post_place_trajectory = plan_res['post_place_trajectory']
        response.reset_trajectory = plan_res['reset_trajectory']
        
        response.result = response.SUCCESS
        rospy.loginfo("Start executing grasp plan...")
        execute_plan_close_loop_with_pub(arm_cmd_pub, gripper_cmd_pub, plan_res)
        rospy.loginfo("motion planning: end of execution.")

    return response

def motion_planning_server():
    rospy.init_node('one_shot_grasp_with_object_pose_server')
    # hot start to wait for octomap
    rospy.sleep(2.)
    s = rospy.Service('/motion_planning/one_shot_grasp_with_object_pose', OneShotGraspPlanWithObjectPose, handle_grasp_plan)
    
    print("Ready for grasp plan.")
    rospy.spin()

if __name__ == "__main__":
    motion_planning_server()

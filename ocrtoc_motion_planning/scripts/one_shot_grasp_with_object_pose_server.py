#!/usr/bin/env python

from __future__ import print_function

from motion_planning.srv import OneShotGraspPlanWithObjectPose, OneShotGraspPlanWithObjectPoseResponse
from motion_planning_functions import one_shot_grasp_with_object_pose
import tf

import rospy

def handle_grasp_plan(req):
    #scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(req.object_pose1)
    #scale, shear, angles, trans, persp = tf.transformations.decompose_matrix(req.object_pose2)
    pre_grasp_trajectory, pre_to_grasp_trajectory, place_trajectory, reset_trajectory = one_shot_grasp_with_object_pose(req.object_name, req.object_scale, req.object_pose1, req.object_pose2)
    response = OneShotGraspPlanWithObjectPoseResponse()
    response.pre_grasp_trajectory = pre_grasp_trajectory
    response.pre_to_grasp_trajectory = pre_to_grasp_trajectory
    response.place_trajectory = place_trajectory
    response.reset_trajectory = reset_trajectory
    response.result = 1
    return response

def motion_planning_server():
    rospy.init_node('one_shot_grasp_with_object_pose_server')
    s = rospy.Service('/motion_planning/one_shot_grasp_with_object_pose', OneShotGraspPlanWithObjectPose, handle_grasp_plan)
    print("Ready for grasp plan.")
    rospy.spin()

if __name__ == "__main__":
    motion_planning_server()

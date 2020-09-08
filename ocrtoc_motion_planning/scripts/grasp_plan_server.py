#!/usr/bin/env python

from __future__ import print_function

from motion_planning.srv import GraspPlan, GraspPlanResponse
from motion_planning.scripts.motion_planning import plan_arm_pose_to_pose
import rospy

def handle_grasp_plan(req):
    result = plan_grasp(req.object_name, req.object_pose1, req.object_pose2)
    response = GraspPlanResponse()
    response.result = result
    return response

def motion_planning_server():
    rospy.init_node('grasp_plan_server')
    s = rospy.Service('grasp_plan', GraspPlan, handle_grasp_plan)
    print("Ready for grasp plan.")
    rospy.spin()

if __name__ == "__main__":
    motion_planning_server()

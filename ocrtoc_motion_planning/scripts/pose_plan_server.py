#!/usr/bin/env python

from __future__ import print_function

import motion_planning
from motion_planning.srv import PosePlan, PosePlanResponse
from motion_planning import plan_arm_pose_to_pose

import rospy

def handle_pose_plan(req):
    result = plan_arm_pose_to_pose(None, req.target_pose)
    response = PosePlanResponse()
    response.result = result
    return response

def motion_planning_server():
    rospy.init_node('pose_plan_server')
    s = rospy.Service('pose_plan', PosePlan, handle_pose_plan)
    print("Ready for pose plan.")
    rospy.spin()

if __name__ == "__main__":
    motion_planning_server()

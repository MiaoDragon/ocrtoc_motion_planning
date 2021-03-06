# ocrtoc_motion_planning
motion planning components for IROS ocrtoc 2020 challenge.

**ROS Service**

- /one_shot_grasp_with_object_pose

**Message**

```
# ** Define input
# =========================================================================================================
# in this service, we assume our motion planning algorithm has access to the grasp generation service.
# Hence we are using the mesh model and transform it for grasp pose estimation.
# To do this, we require the scale of the mesh file, and the start and goal pose.
# =========================================================================================================
string object_name # object name to grasp. This is used to generate grasp pose
float64[] object_scale # vector of size 3
                       # scale of the object mesh -> real. Useful if we need to call grasp service inside.
geometry_msgs/Pose object_pose1 # initial object pose
geometry_msgs/Pose object_pose2 # target object pose
---
# ** Define output
# =========================================================================================================
# We only return the one-shot planning result as a list of planned trajectory.
# result variable indicates if the entire pipeline fails after several trials.
# =========================================================================================================
# refer to: https://github.com/ros-planning/moveit_msgs/blob/master/msg/MoveItErrorCodes.msg
int32 result # error code
trajectory_msgs/JointTrajectory pre_grasp_trajectory
trajectory_msgs/JointTrajectory pre_to_grasp_trajectory
trajectory_msgs/JointTrajectory place_trajectory
trajectory_msgs/JointTrajectory reset_trajectory
int32 SUCCESS=1
int32 FAILURE=99999
```

**Step**

```
roslaunch motion_planning motion_planning_pre_sapien.launch # this runs the prerequest ros setup
rosrun motion_planning one_shot_grasp_with_object_pose_server.py # this setups the service server  ** Note: we don't have execution server yet.
```

**Testing**
Inside folder ocrtoc_motion_planning/ocrtoc_motion_planning/scripts, run
```
roslaunch motion_planning motion_planning_pre_sapien.launch # run the setup for sapien
python motion_planning_functions.py # Individually for testing motion planning
```

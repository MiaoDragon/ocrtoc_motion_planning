# ocrtoc_motion_planning
motion planning components for IROS ocrtoc 2020 challenge.

**ROS Service**

- /one_shot_grasp_with_object_pose


**Message**

```
# input
string object_name # object name to grasp. This is used to generate grasp pose
float64[] object_scale # vector of size 3
                       # scale of the object mesh -> real. Useful if we need to call grasp service inside.
geometry_msgs/Pose object_pose1 # initial object pose
geometry_msgs/Pose object_pose2 # target object pose

---
# output
int32 result # error code
trajectory_msgs/PreGraspTrajectory pre_grasp_trajectory
trajectory_msgs/PreToGraspTrajectory pre_to_grasp_trajectory
trajectory_msgs/PlaceTrajectory place_trajectory
int32 SUCCESS=1
int32 FAILURE=99999
```

**Step**

```
roslaunch motion_planning motion_planning_pre.launch # this runs the prerequest ros setup
rosrun motion_planning one_shot_grasp_with_object_pose_server.py # this setups the service server
```

**Testing**
Inside folder ocrtoc_motion_planning/ocrtoc_motion_planning/scripts, run
```
roslaunch motion_planning motion_planning_pre.launch
python motion_planning.py # Individually for testing motion planning
```

# ocrtoc_motion_planning
motion planning components for IROS ocrtoc 2020 challenge.

* ROS Service *

- /grasp_ plan


* Input *

```
string object_name # object name to grasp. This is used to generate grasp pose
geometry_msgs/Pose object_pose1 # initial object pose
geometry_msgs/Pose object_pose2 # target object pose

---
# feedback
int32 result # error code
```

* Step *

```
rosrun ocrtoc_motion_planning grasp_plan_server.py # this setups the service server
```

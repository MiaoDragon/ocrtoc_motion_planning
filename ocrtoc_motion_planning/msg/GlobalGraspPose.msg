# define a list of grasp pose for a given object with pose
geometry_msgs/Pose[] grasp_poses
geometry_msgs/Pose[] pre_grasp_poses

# local poses related
geometry_msgs/Pose[] local_poses
float64[] scales

# grasp related
float64[] grasp_widths
string[]  model_names
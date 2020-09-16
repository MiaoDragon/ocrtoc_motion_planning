cmake_minimum_required(VERSION 2.8.3)
project(motion_planning)
#catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  trajectory_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  roscpp
  std_msgs
)
catkin_python_setup()

add_message_files(
  FILES
  PointCloudFilterAction.msg
)
add_service_files(
  FILES
  PosePlan.srv
  OneShotGraspPlanWithGraspPose.srv
  OneShotGraspPlanWithObjectPose.srv
)

#generate_messages()

generate_messages( DEPENDENCIES std_msgs geometry_msgs trajectory_msgs)


catkin_package(
  #LIBRARIES motion_planning
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs trajectory_msgs message_generation message_runtime roscpp rospy std_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(
    PROGRAMS
    scripts/motion_planning_functions.py
    scripts/depth_to_pcd_merger.py
    scripts/depth_to_pcd_kinect.py
    scripts/point_cloud_merger.py
    scripts/pcd_filter.py
    scripts/one_shot_grasp_with_object_pose_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)



#install(PROGRAMS
#  scripts/commit_solution.py
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

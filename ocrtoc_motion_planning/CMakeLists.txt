cmake_minimum_required(VERSION 3.0.2)
project(motion_planning)
#catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  sensor_msgs
  trajectory_msgs
  message_generation
  message_runtime
  roscpp
  rospy
  roscpp
  std_msgs
)
#catkin_python_setup()

add_message_files(
  FILES
  PointCloudFilterAction.msg
)
add_service_files(
  FILES
  PosePlan.srv
  OneShotGraspPlanWithGraspPose.srv
  OneShotGraspPlanWithObjectPose.srv
  UR5IKService.srv
)

#generate_messages()

generate_messages( DEPENDENCIES std_msgs geometry_msgs sensor_msgs trajectory_msgs)


catkin_package(
  #LIBRARIES motion_planning
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs sensor_msgs trajectory_msgs message_generation message_runtime roscpp rospy std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
catkin_install_python(
    PROGRAMS
    scripts/motion_planning_functions.py
    scripts/motion_planning_execution.py
    scripts/motion_planning_utils.py
    scripts/moveit_connection.py
    scripts/depth_to_pcd_merger.py
    scripts/depth_to_pcd_kinect_multithread.py
    scripts/depth_to_pcd_kinect.py
    scripts/point_cloud_merger.py
    scripts/pcd_filter.py
    scripts/one_shot_grasp_with_object_pose_server.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


add_executable(ur5_ik_service src/ur5_kinematics_service.cpp)
target_link_libraries(ur5_ik_service ${catkin_LIBRARIES})
add_dependencies(ur5_ik_service motion_planning_gencpp)


install(DIRECTORY
  launch
  data
  msg
  srv
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


file(GLOB rospkg_scripts
    "scripts/*.py"
)

install(PROGRAMS
  ${rospkg_scripts}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

cmake_minimum_required(VERSION 3.0.2)
project(alpha_control)
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  message_generation
)
add_message_files(
  DIRECTORY msgs
  FILES
  RefTrajectoryPoint.msg
  RefTrajectory.msg
  TrackingCommand.msg
)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS rospy std_msgs geometry_msgs nav_msgs visualization_msgs tf message_runtime)
catkin_install_python(PROGRAMS
  nodes/parking_planner_node.py
  nodes/path_tracker_node.py
  scripts/hybrid_astar_rviz_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config rviz DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

cmake_minimum_required(VERSION 3.0.2)
project(openmp_moveit)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  ros_tcp_endpoint
  message_generation
  moveit_msgs
  sensor_msgs
  controller_manager
  roslaunch
  trajectory_msgs
)
roslaunch_add_file_check(launch)

add_message_files(DIRECTORY msg)

add_service_files(DIRECTORY srv)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/mover.py
  scripts/trajectory_subscriber.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})



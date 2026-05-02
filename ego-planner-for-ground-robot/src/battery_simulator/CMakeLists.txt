cmake_minimum_required(VERSION 3.0.2)
project(battery_simulator)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  BatteryState.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs nav_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/battery_monitor_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

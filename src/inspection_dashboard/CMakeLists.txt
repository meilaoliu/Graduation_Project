cmake_minimum_required(VERSION 3.0.2)
project(inspection_dashboard)

find_package(catkin REQUIRED COMPONENTS rospy std_msgs nav_msgs sensor_msgs)

catkin_package()

catkin_install_python(PROGRAMS scripts/dashboard_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch templates static
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

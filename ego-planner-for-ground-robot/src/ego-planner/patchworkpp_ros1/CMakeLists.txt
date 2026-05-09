cmake_minimum_required(VERSION 3.0.2)
project(patchworkpp_ros1)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  third_party/patchworkpp/include
)

add_library(patchworkpp_core
  third_party/patchworkpp/src/patchworkpp.cpp
)
target_link_libraries(patchworkpp_core Eigen3::Eigen)

add_executable(patchworkpp_offline_own_data nodes/patchworkpp_offline_own_data.cpp)
target_link_libraries(patchworkpp_offline_own_data
  patchworkpp_core
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)


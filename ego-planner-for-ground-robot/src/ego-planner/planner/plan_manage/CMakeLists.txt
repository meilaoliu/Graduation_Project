cmake_minimum_required(VERSION 2.8.3)
project(ego_planner)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  plan_env
  path_searching
  bspline_opt
  minco_opt
  traj_utils
  message_generation
  cv_bridge
  mpc_controller
  tf
)

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Bspline.msg
  MINCOTraj.msg
  DataDisp.msg
  )
  
# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

# catkin_package(CATKIN_DEPENDS message_runtime)
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES ego_planner
 CATKIN_DEPENDS plan_env path_searching bspline_opt minco_opt traj_utils message_runtime mpc_controller
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})

add_executable(ego_planner_node
  src/ego_planner_node.cpp 
  src/ego_replan_fsm.cpp
  src/planner_manager.cpp
  )
target_link_libraries(ego_planner_node 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  )
add_dependencies(ego_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(traj_server src/traj_server.cpp)
target_link_libraries(traj_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} libqpOASES.so)
add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_ground_safety_utils test/test_ground_safety_utils.cpp)
  if(TARGET test_ground_safety_utils)
    target_link_libraries(test_ground_safety_utils ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_fsm_heading_utils test/test_fsm_heading_utils.cpp)
  if(TARGET test_fsm_heading_utils)
    target_link_libraries(test_fsm_heading_utils ${catkin_LIBRARIES})
  endif()
endif()

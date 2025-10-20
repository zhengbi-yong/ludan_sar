cmake_minimum_required(VERSION 3.0.2)
project(moveit_movej_bridge)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
  std_msgs
)

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})

add_executable(ftj_to_movej_bridge src/ftj_to_movej_bridge.cpp)
target_link_libraries(ftj_to_movej_bridge ${catkin_LIBRARIES})
add_executable(joint_states_merger src/joint_states_merger.cpp)
target_link_libraries(joint_states_merger ${catkin_LIBRARIES})
install(TARGETS joint_states_merger
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


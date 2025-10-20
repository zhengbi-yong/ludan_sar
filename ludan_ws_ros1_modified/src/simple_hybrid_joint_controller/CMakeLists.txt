cmake_minimum_required(VERSION 3.0.2)
project(simple_hybrid_joint_controller)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  controller_interface
  hardware_interface
  pluginlib
  std_msgs
  legged_common
)

catkin_package(
  CATKIN_DEPENDS roscpp controller_interface hardware_interface pluginlib std_msgs legged_common
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME}
  src/AllJointsHybridController.cpp
)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES simple_hybrid_joint_controller_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
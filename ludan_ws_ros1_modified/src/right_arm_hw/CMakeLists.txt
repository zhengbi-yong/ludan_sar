cmake_minimum_required(VERSION 3.0.2)
project(right_arm_hw)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  actionlib
  control_msgs
  trajectory_msgs
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(ftj_to_moveJ_bridge src/ftj_to_moveJ_bridge.cpp)
target_compile_features(ftj_to_moveJ_bridge PRIVATE cxx_std_11)
target_link_libraries(ftj_to_moveJ_bridge ${catkin_LIBRARIES})


# 安装（可选但推荐），方便被其它包引用/发布
install(TARGETS ftj_to_moveJ_bridge
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch/ config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.yaml")

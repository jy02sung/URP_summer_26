cmake_minimum_required(VERSION 3.0.2)
project(dual_arm)

find_package(pinocchio REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  message_generation
  tf
  geometry_msgs
  gazebo_msgs
  control_msgs
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dual_arm
 CATKIN_DEPENDS roscpp rospy roslib std_msgs tf geometry_msgs gazebo_msgs control_msgs
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(dual_arm_main src/main.cpp)
add_dependencies(dual_arm_main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_arm_main ${catkin_LIBRARIES} pinocchio::pinocchio)

add_executable(dual_arm_command src/dual_arm_command.cpp)
add_dependencies(dual_arm_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dual_arm_command ${catkin_LIBRARIES} pinocchio::pinocchio)

add_executable(mission4_ik_check src/mission4_ik_check.cpp)
add_dependencies(mission4_ik_check ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission4_ik_check ${catkin_LIBRARIES} pinocchio::pinocchio)

add_executable(gravity_pd_controller src/gravity_pd_controller.cpp)
add_dependencies(gravity_pd_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gravity_pd_controller ${catkin_LIBRARIES} pinocchio::pinocchio)

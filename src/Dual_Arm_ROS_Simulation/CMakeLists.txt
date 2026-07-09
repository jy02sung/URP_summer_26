cmake_minimum_required(VERSION 3.0.2)
project(dual_arm)

find_package(pinocchio REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  roslib
  gazebo_msgs
  tf
)


catkin_package(
 INCLUDE_DIRS include
 LIBRARIES dual_arm
 CATKIN_DEPENDS roscpp rospy std_msgs 
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
add_compile_options(-std=c++17)
cmake_minimum_required(VERSION 2.8.3)
project(aidin_arm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  aruco_ros
  gazebo_ros 
  geometry_msgs
  tf2
  tf2_ros
  )

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES aidin_arm
  CATKIN_DEPENDS roscpp std_msgs message_runtime
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ./include
    ./src
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
  DEPENDS 
    roscpp 
    gazebo_ros 
    aruco_ros
)

add_executable(main src/main.cpp)
add_dependencies(main ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(main ${catkin_LIBRARIES})

add_executable(arm_command src/arm_command.cpp)
add_dependencies(arm_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(arm_command ${catkin_LIBRARIES})

add_executable(mobile_command src/mobile_command.cpp)
add_dependencies(mobile_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mobile_command ${catkin_LIBRARIES})

add_executable(aruco_detection_command src/aruco_detection_command.cpp)
add_dependencies(aruco_detection_command ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(aruco_detection_command ${catkin_LIBRARIES})

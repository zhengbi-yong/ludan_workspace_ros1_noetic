cmake_minimum_required(VERSION 3.0.2)
project(dmbot_serial)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
link_directories(/opt/ros/noetic/lib)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  dynamic_reconfigure
)
find_package(roscpp REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/my_cfg.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dmbot_serial
  CATKIN_DEPENDS roscpp rospy std_msgs serial dynamic_reconfigure
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

########## 🚀 以下是重点 ##########

add_library(dmbot_serial
  src/robot_connect.cpp
)

add_executable(test_motor src/test_motor.cpp)
add_executable(test_motor_read src/test_motor_read.cpp)
# 👇 **关键：一定要在同一个 target_link_libraries 里按顺序链接**
target_link_libraries(dmbot_serial
  ${catkin_LIBRARIES}
  roscpp
  pthread
  rosconsole
)

target_link_libraries(test_motor
  dmbot_serial
  ${catkin_LIBRARIES}
  roscpp
  rosconsole
  pthread
)
target_link_libraries(test_motor_read
  dmbot_serial
  ${catkin_LIBRARIES}
  roscpp
  rosconsole
  pthread
)


add_dependencies(dmbot_serial ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_motor ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(test_motor_read ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
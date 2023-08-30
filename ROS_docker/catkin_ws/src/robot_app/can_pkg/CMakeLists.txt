cmake_minimum_required(VERSION 2.8.3)
project(can_pkg)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp robot_msgs)

## Declare ROS messages and services
add_message_files(
  DIRECTORY msg
  FILES
    can.msg
)



## Generate added messages and services 12generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(can scripts/can.cpp)
target_link_libraries(can ${catkin_LIBRARIES})
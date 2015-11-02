cmake_minimum_required(VERSION 2.8.3)
project(person_tracking)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs cv_bridge image_transport)

## Declare a catkin package
catkin_package()

## Build selector and tracker
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(select_target src/select_target.cpp)
target_link_libraries(select_target ${catkin_LIBRARIES})
add_dependencies(select_target select_target_generate_messages_cpp)

add_executable(track_shirt src/track_shirt.cpp)
target_link_libraries(track_shirt ${catkin_LIBRARIES})
add_dependencies(track_shirt track_shirt_generate_messages_cpp)

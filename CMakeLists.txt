cmake_minimum_required(VERSION 2.8.3)
project(person_tracking)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg sensor_msgs cv_bridge image_transport)

## Add message files
add_message_files(
  FILES
  TrackedObject.msg
)

## Generate message files
generate_messages(
   DEPENDENCIES
   std_msgs
)

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

add_executable(track_pant src/track_pant.cpp)
target_link_libraries(track_pant ${catkin_LIBRARIES})
add_dependencies(track_pant track_pant_generate_messages_cpp)

add_executable(navigate_perforo src/navigate_perforo.cpp)
target_link_libraries(navigate_perforo ${catkin_LIBRARIES})
add_dependencies(navigate_perforo navigate_perforo_generate_messages_cpp)

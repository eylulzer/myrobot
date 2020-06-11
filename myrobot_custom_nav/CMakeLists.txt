cmake_minimum_required(VERSION 3.0.2)
project(myrobot_custom_nav)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  nav_msgs
  roscpp
  actionlib
  rospy
  std_msgs
  myrobot_msgs
)

catkin_package(
    CATKIN_DEPENDS myrobot_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(send_goal src/send_goal.cpp)
target_link_libraries(send_goal ${catkin_LIBRARIES})
add_dependencies(send_goal myrobot_msgs_generate_messages_cpp)
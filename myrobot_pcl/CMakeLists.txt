cmake_minimum_required(VERSION 3.0.2)
project(myrobot_pcl)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
)

catkin_package(
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES})

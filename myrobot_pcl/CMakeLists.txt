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

find_package( PCL REQUIRED )

catkin_package(
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test src/pcl_test.cpp)
target_link_libraries(pcl_test ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(pcl_nodelet src/pcl_nodelet.cpp)
target_link_libraries(pcl_nodelet ${catkin_LIBRARIES} ${PCL_LIBRARIES})
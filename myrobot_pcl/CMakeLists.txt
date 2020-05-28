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


file(GLOB PCL_EXAMPLES src/*.cpp)
foreach (SOURCE_FILE ${PCL_EXAMPLES})
    string(REPLACE ".cpp" "" filename ${SOURCE_FILE})
    string(REPLACE "$ENV{HOME}/catkin_ws/src/myrobot/myrobot_pcl/src/" "" filename ${filename})
    add_executable(${filename} ${SOURCE_FILE})
    target_link_libraries( ${filename} ${catkin_LIBRARIES}  ${PCL_LIBRARIES})
endforeach (SOURCE_FILE ${CV_EXAMPLES})

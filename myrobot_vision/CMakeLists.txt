cmake_minimum_required(VERSION 2.8.3)
project(myrobot_vision)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    rospy
    sensor_msgs
    geometry_msgs
    std_msgs
    )

find_package(OpenCV REQUIRED)
catkin_package()

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

file(GLOB CV_EXAMPLES src/*.cpp)
foreach (SOURCE_FILE ${CV_EXAMPLES})
    string(REPLACE ".cpp" "" filename ${SOURCE_FILE})
    string(REPLACE "/home/ros/catkin_ws/src/myrobot/myrobot_vision/src/" "" filename ${filename})
    add_executable(${filename} ${SOURCE_FILE})
    target_link_libraries( ${filename} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach (SOURCE_FILE ${CV_EXAMPLES})

add_subdirectory(src/approach_bin)


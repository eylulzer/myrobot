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
    myrobot_msgs
    )

find_package(OpenCV REQUIRED)

catkin_package(
    CATKIN_DEPENDS myrobot_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_subdirectory(src/opencv_examples)
add_subdirectory(src/approach_bin)
add_subdirectory(src/opencv_road)
add_subdirectory(src/qr_barcode)
add_subdirectory(src/power_station)


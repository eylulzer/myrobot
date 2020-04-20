cmake_minimum_required(VERSION 2.8.3)
project(myrobot_vision)

find_package(catkin REQUIRED COMPONENTS
    cv_bridge
    image_transport
    roscpp
    rospy
    sensor_msgs
    std_msgs
    )

find_package(OpenCV REQUIRED)

catkin_package(
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv_test src/cv_test.cpp)
add_executable(cv_bw src/cv_bw.cpp)
add_executable(cv_sharpen src/cv_sharpen.cpp)
add_executable(cv_blend src/cv_blend.cpp)
add_executable(cv_brightness src/cv_brightness.cpp)
add_executable(cv_contrast src/cv_contrast.cpp)
add_executable(cv_drawatom src/cv_drawatom.cpp)
add_executable(cv_crop src/cv_crop.cpp)
add_executable(cv_lookuptable src/cv_lookuptable.cpp)
add_executable(cv_text src/cv_text.cpp)


target_link_libraries(
    cv_test
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_bw
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_sharpen
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_blend
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_brightness
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_contrast
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_drawatom
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_crop
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_lookuptable
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
    cv_text
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)
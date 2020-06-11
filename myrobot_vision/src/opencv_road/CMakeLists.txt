
add_library(my_line MyLine.h MyLine.cpp)
add_executable(OpenCVROAD opencv_road.cpp)
add_executable(OpenCVROAD2 opencv_road_2.cpp)


target_link_libraries(
    OpenCVROAD
    my_line
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

target_link_libraries(
        OpenCVROAD2
        my_line
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)
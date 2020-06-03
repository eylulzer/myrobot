
add_library(my_line MyLine.h MyLine.cpp)
add_executable(OpenCVROAD opencv_road.cpp)

target_link_libraries(
    OpenCVROAD
    my_line
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
)

add_library(findcircles FindCircles.h FindCircles.cpp)
add_library(pointmeasure PointMeasure.h PointMeasure.cpp)
add_library(customcircle CustomCircle.h CustomCircle.cpp)

add_executable(cv_approach_bin cv_approach_bin.cpp)

target_link_libraries(cv_approach_bin
    findcircles
    pointmeasure
    customcircle
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )

add_library(findcircles FindCircles.h FindCircles.cpp)
add_library(pointmeasure PointMeasure.h PointMeasure.cpp)
add_library(customcircle CustomCircle.h CustomCircle.cpp)
add_library(cv_approach_bin cv_approach_bin.cpp cv_approach_bin.h)

add_executable(ApproachBin main.cpp)

target_link_libraries(ApproachBin
    cv_approach_bin
    findcircles
    pointmeasure
    customcircle
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )
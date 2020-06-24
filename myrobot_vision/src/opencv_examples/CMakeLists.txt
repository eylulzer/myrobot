
file(GLOB CV_EXAMPLES *.cpp)
foreach (SOURCE_FILE ${CV_EXAMPLES})
    string(REPLACE ".cpp" "" filename ${SOURCE_FILE})
    string(REPLACE "$ENV{HOME}/catkin_ws/src/myrobot/myrobot_vision/src/opencv_examples/" "" filename ${filename})
    add_executable(${filename} ${SOURCE_FILE})
    target_link_libraries( ${filename} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
endforeach (SOURCE_FILE ${CV_EXAMPLES})
cmake_minimum_required(VERSION 2.8.3)
project(myrobot_base)

find_package(catkin REQUIRED COMPONENTS
  controller_manager
  hardware_interface
  roscpp
  std_msgs
)

catkin_package(

)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(myrobot_hardware_interface src/myrobot_hardware_interface.cpp)
add_dependencies (myrobot_hardware_interface ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(myrobot_hardware_interface ${catkin_LIBRARIES})
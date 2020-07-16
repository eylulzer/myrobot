
add_executable(powerStation
  power_station.cpp
  Line.h Line.cpp
  Docking.h Docking.cpp
  PointMeasure.h PointMeasure.cpp
)

target_link_libraries(
  powerStation
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED IMPORTED_TARGET zbar)

target_link_libraries(powerStation PkgConfig::ZBar)

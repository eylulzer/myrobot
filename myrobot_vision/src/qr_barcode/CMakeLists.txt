
add_executable(readQRBarcode qr_code.cpp)

target_link_libraries(
        readQRBarcode
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(ZBar REQUIRED IMPORTED_TARGET zbar)

target_link_libraries(readQRBarcode PkgConfig::ZBar)
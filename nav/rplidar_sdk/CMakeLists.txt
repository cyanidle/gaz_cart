# Vendored Slamtec RPLidar SDK (from cyanidle/bigbang submodule/lidar-sdk),
# built as a static lib for the gaz_nav Lidar worker. Linux-only build — the
# arch/{macOS,win32} sources are left in the tree but not compiled.
cmake_minimum_required(VERSION 3.16)
project(rplidar_sdk C CXX)

set(CMAKE_CXX_STANDARD 17)

file(GLOB RPLIDAR_SDK_SRC
    sdk/src/arch/linux/*.cpp
    sdk/src/hal/*.cpp
    sdk/src/*.cpp
)

add_library(rplidar_sdk STATIC ${RPLIDAR_SDK_SRC})
target_include_directories(rplidar_sdk PUBLIC
    sdk/include
    sdk/src
)
set_target_properties(rplidar_sdk PROPERTIES POSITION_INDEPENDENT_CODE ON)

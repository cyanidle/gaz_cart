cmake_minimum_required(VERSION 3.16)
project(gaz_cart_nav CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

add_subdirectory(rplidar_sdk)

create_radapter_plugin(gaz_nav
    FILES
        plugin.cpp
        costmap_server.cpp
        global_planner.cpp
        local_planner.cpp
        lidar.cpp
)

find_package(Qt6 REQUIRED COMPONENTS Gui)
target_link_libraries(gaz_nav PRIVATE Qt6::Gui rplidar_sdk)

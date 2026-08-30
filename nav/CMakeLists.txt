cmake_minimum_required(VERSION 3.16)
project(gaz_cart_nav CXX)

find_package(Qt6 REQUIRED COMPONENTS Gui)

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

target_link_libraries(gaz_nav PRIVATE Qt6::Gui rplidar_sdk)

set_target_properties(gaz_nav PROPERTIES
    BUILD_RPATH   "$ORIGIN/../bin"
    INSTALL_RPATH "$ORIGIN/.."
)

# Canonical location for .deb-installed radapter plugins (see cmake/Packaging.cmake).
# Component-scoped so cpack packages only the plugin, not radapter's own
# install rules (radapter-sdk lib + headers) that share this build tree.
install(TARGETS gaz_nav LIBRARY DESTINATION lib/radapter/plugins COMPONENT gaz_nav)

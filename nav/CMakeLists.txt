cmake_minimum_required(VERSION 3.16)
project(gaz_nav CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Build the radapter SDK from the sibling submodule (no app, no tests).
# CPM_SOURCE_CACHE makes the dependency fetch a no-op after the first configure.
set(RADAPTER_SDK_ONLY ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../radapter radapter)

create_radapter_plugin(gaz_nav
    FILES
        plugin.cpp
        costmap_server.cpp
        global_planner.cpp
        local_planner.cpp
)

find_package(Qt6 REQUIRED COMPONENTS Gui)
target_link_libraries(gaz_nav PRIVATE Qt6::Gui)

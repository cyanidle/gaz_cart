cmake_minimum_required(VERSION 3.16)
project(gaz_cart_frames CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

create_radapter_plugin(gaz_frames
    FILES
        plugin.cpp
        frames.cpp
)

set_target_properties(gaz_frames PROPERTIES
    BUILD_RPATH   "$ORIGIN/../bin"
    INSTALL_RPATH "$ORIGIN/../.."
)

install(TARGETS gaz_frames
    LIBRARY DESTINATION lib/radapter/plugins
    COMPONENT gaz_frames
)

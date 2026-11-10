cmake_minimum_required(VERSION 3.16)
project(gaz_cart_joystick CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

create_radapter_plugin(gaz_joystick
    FILES
        plugin.cpp
        joystick.cpp
)

set_target_properties(gaz_joystick PROPERTIES
    BUILD_RPATH   "$ORIGIN/../bin"
    INSTALL_RPATH "$ORIGIN/../.."
)

install(TARGETS gaz_joystick
    LIBRARY DESTINATION lib/radapter/plugins
    COMPONENT gaz_joystick
)

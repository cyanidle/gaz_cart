cmake_minimum_required(VERSION 3.16)
project(gaz_cart VERSION 0.1.0 LANGUAGES CXX C)

option(GAZ_BUILD_FRAMES "Build the gaz_frames plugin" ON)
option(GAZ_BUILD_NAV "Build the gaz_nav plugin" ON)
option(GAZ_BUILD_SLAM "Build the gaz_slam plugin" ON)
option(RADAPTER_GUI "Enable GUI capabilities" OFF)

# Unified packaging target:
#   cart — headless radapter + all three plugins + Lua cart runtime (the RPi)
#   gui  — GUI radapter + gui.lua + QML UI (the controller PC)
#   none — no unified package; plugins are packaged separately (development)
#   auto — pick cart for a clean headless all-plugin configure, gui when
#          RADAPTER_GUI=ON, none otherwise
set(GAZ_PACKAGE_TARGET "auto" CACHE STRING "Unified package target: cart, gui, none, or auto")
set_property(CACHE GAZ_PACKAGE_TARGET PROPERTY STRINGS auto cart gui none)

if(GAZ_PACKAGE_TARGET STREQUAL "auto")
    set(GAZ_PACKAGE_TARGET "")
    if(RADAPTER_GUI)
        set(GAZ_PACKAGE_TARGET gui)
    elseif(NOT RADAPTER_SDK_ONLY AND NOT RADAPTER_STATIC AND
           GAZ_BUILD_FRAMES AND GAZ_BUILD_NAV AND GAZ_BUILD_SLAM)
        set(GAZ_PACKAGE_TARGET cart)
    endif()
elseif(GAZ_PACKAGE_TARGET STREQUAL "none")
    set(GAZ_PACKAGE_TARGET "")
endif()

if(GAZ_PACKAGE_TARGET STREQUAL "cart" AND (RADAPTER_GUI OR RADAPTER_SDK_ONLY OR RADAPTER_STATIC OR
   NOT GAZ_BUILD_FRAMES OR NOT GAZ_BUILD_NAV OR NOT GAZ_BUILD_SLAM))
    message(FATAL_ERROR "GAZ_PACKAGE_TARGET=cart requires headless shared radapter and all three plugins. Set RADAPTER_GUI/RADAPTER_SDK_ONLY/RADAPTER_STATIC=OFF and GAZ_BUILD_FRAMES/GAZ_BUILD_NAV/GAZ_BUILD_SLAM=ON, or GAZ_PACKAGE_TARGET=none for development.")
endif()
if(GAZ_PACKAGE_TARGET STREQUAL "gui" AND (NOT RADAPTER_GUI OR RADAPTER_SDK_ONLY OR RADAPTER_STATIC))
    message(FATAL_ERROR "GAZ_PACKAGE_TARGET=gui requires a GUI shared build. Set RADAPTER_GUI=ON and RADAPTER_SDK_ONLY/RADAPTER_STATIC=OFF, or GAZ_PACKAGE_TARGET=none for development.")
endif()

add_subdirectory(radapter)
if(GAZ_BUILD_FRAMES)
    add_subdirectory(frames)
endif()
if(GAZ_BUILD_NAV)
    add_subdirectory(nav)
endif()
if(GAZ_BUILD_SLAM)
    add_subdirectory(slam)
endif()

include(cmake/Packaging.cmake)

if(NOT CMAKE_CROSSCOMPILING AND GAZ_PACKAGE_TARGET)
    enable_testing()
    add_test(NAME packaging_smoke_${GAZ_PACKAGE_TARGET}
        COMMAND ${CMAKE_COMMAND}
            -DBUILD_DIR=${CMAKE_BINARY_DIR}
            -DSTAGE_DIR=${CMAKE_BINARY_DIR}/packaging-smoke
            -DPKG_TARGET=${GAZ_PACKAGE_TARGET}
            -DCONFIG=$<CONFIG>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/packaging/stage.cmake)
    set_tests_properties(packaging_smoke_${GAZ_PACKAGE_TARGET} PROPERTIES TIMEOUT 60)
endif()

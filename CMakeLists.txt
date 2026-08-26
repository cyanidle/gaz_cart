cmake_minimum_required(VERSION 3.16)
project(gaz_cart CXX C)

option(GAZ_BUILD_FRAMES "Build the gaz_frames plugin" ON)
option(GAZ_BUILD_NAV "Build the gaz_nav plugin" ON)
option(GAZ_BUILD_SLAM "Build the gaz_slam plugin" ON)

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

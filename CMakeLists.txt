cmake_minimum_required(VERSION 3.16)
project(gaz_cart VERSION 0.1.0 LANGUAGES CXX C)

option(GAZ_BUILD_FRAMES "Build the gaz_frames plugin" ON)
option(GAZ_BUILD_NAV "Build the gaz_nav plugin" ON)
option(GAZ_BUILD_SLAM "Build the gaz_slam plugin" ON)
option(RADAPTER_GUI "Enable GUI capabilities" OFF)

set(gaz_runtime_default ON)
if(RADAPTER_GUI OR RADAPTER_SDK_ONLY OR RADAPTER_STATIC OR
   NOT GAZ_BUILD_FRAMES OR NOT GAZ_BUILD_NAV OR NOT GAZ_BUILD_SLAM)
    set(gaz_runtime_default OFF)
endif()
option(GAZ_PACKAGE_RUNTIME "Package headless radapter and the complete cart runtime" ${gaz_runtime_default})
if(GAZ_PACKAGE_RUNTIME AND (RADAPTER_GUI OR RADAPTER_SDK_ONLY OR RADAPTER_STATIC OR
   NOT GAZ_BUILD_FRAMES OR NOT GAZ_BUILD_NAV OR NOT GAZ_BUILD_SLAM))
    message(FATAL_ERROR "GAZ_PACKAGE_RUNTIME requires headless shared radapter and all three plugins. Set RADAPTER_GUI/RADAPTER_SDK_ONLY/RADAPTER_STATIC=OFF and GAZ_BUILD_FRAMES/GAZ_BUILD_NAV/GAZ_BUILD_SLAM=ON, or GAZ_PACKAGE_RUNTIME=OFF for development.")
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

if(GAZ_PACKAGE_RUNTIME AND NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_test(NAME packaging_smoke
        COMMAND ${CMAKE_COMMAND}
            -DBUILD_DIR=${CMAKE_BINARY_DIR}
            -DSTAGE_DIR=${CMAKE_BINARY_DIR}/packaging-smoke
            -DCONFIG=$<CONFIG>
            -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/packaging/stage.cmake)
    set_tests_properties(packaging_smoke PROPERTIES TIMEOUT 60)
endif()

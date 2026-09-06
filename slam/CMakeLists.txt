cmake_minimum_required(VERSION 3.16)
project(gaz_cart_slam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Debian's boost runtime packages carry the boost version in their names
# (libboost-serialization1.74.0 / 1.83.0 / 1.90.0 ...), which breaks DEB
# dependencies across distro releases, and the distro's static archives are
# built without -fPIC, so they cannot be linked into a shared library. Build
# boost serialization from source as a static PIC library instead: it is
# embedded into libgaz_slam.so and leaves no runtime package dependency.
# CPM_SOURCE_CACHE (set for the Docker cross build) caches the tarball.
include(${CMAKE_SOURCE_DIR}/radapter/cmake/CPM.cmake)
set(BOOST_INCLUDE_LIBRARIES serialization)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
CPMAddPackage(
    NAME Boost
    URL https://github.com/boostorg/boost/releases/download/boost-1.84.0/boost-1.84.0.tar.gz
    URL_HASH SHA256=4d27e9efed0f6f152dc28db6430b9d3dfb40c0345da7342eaa5a987dde57bd95
    EXCLUDE_FROM_ALL ON
)

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(TBB REQUIRED)

add_library(gaz_karto STATIC
    third_party/karto_sdk/src/Karto.cpp
    third_party/karto_sdk/src/Mapper.cpp
    third_party/ceres_solver/ceres_solver.cpp
)
set_target_properties(gaz_karto PROPERTIES
    AUTOMOC OFF
    POSITION_INDEPENDENT_CODE ON
)
target_include_directories(gaz_karto PUBLIC
    third_party/karto_sdk/include
    third_party/ceres_solver
)
target_link_libraries(gaz_karto PUBLIC
    Boost::serialization
    Ceres::ceres
    Eigen3::Eigen
    TBB::tbb
)

create_radapter_plugin(gaz_slam
    FILES
        plugin.cpp
        slam.cpp
)
target_include_directories(gaz_slam PRIVATE ${CMAKE_SOURCE_DIR})
target_link_libraries(gaz_slam PRIVATE gaz_karto)

set_target_properties(gaz_slam PROPERTIES
    BUILD_RPATH   "$ORIGIN/../bin"
    INSTALL_RPATH "$ORIGIN/../.."
)

install(TARGETS gaz_slam
    LIBRARY DESTINATION lib/radapter/plugins
    COMPONENT gaz_slam
)

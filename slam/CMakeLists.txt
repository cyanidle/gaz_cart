cmake_minimum_required(VERSION 3.16)
project(gaz_cart_slam CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

# Debian's boost runtime packages carry the boost version in their names
# (libboost-serialization1.74.0 / 1.83.0 / 1.90.0 ...), which breaks DEB
# dependencies across distro releases, and the distro's static archives are
# built without -fPIC, so they cannot be linked into a shared library. Boost is
# instead built from source as a static PIC library and embedded into
# libgaz_slam.so, leaving no runtime package dependency.
#
# radapter already fetches Boost through CPM and CPM ignores a second add under
# the same name, so the top-level CMakeLists requests the serialization
# component there, before add_subdirectory(radapter).

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

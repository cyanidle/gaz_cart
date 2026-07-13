cmake_minimum_required(VERSION 3.16)
project(gaz_cart_slam CXX)

if(POLICY CMP0167)
    cmake_policy(SET CMP0167 NEW)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)

find_package(Boost REQUIRED COMPONENTS serialization)
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

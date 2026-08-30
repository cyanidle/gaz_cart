# ============================================================================
# CPack DEB packaging for the gaz_cart radapter plugins.
# Included from the top-level CMakeLists — after all add_subdirectory() calls
# so CPACK_* vars leaked by CPM deps are overridden here.
#
#   cmake --build build -j $(nproc)
#   (cd build && cpack -G DEB)
#
# Installs the plugins into /usr/lib/radapter/plugins; reference them from
# cart.lua, e.g. plugins = { nav = "/usr/lib/radapter/plugins/libgaz_nav" }.
# ============================================================================

if(NOT TARGET gaz_nav AND NOT TARGET gaz_frames AND NOT TARGET gaz_slam)
    return()
endif()

set(CPACK_PACKAGE_VENDOR        "cyanidle")
set(CPACK_PACKAGE_CONTACT       "lyosha.doronin@gmail.com")
set(CPACK_PACKAGE_HOMEPAGE_URL  "https://github.com/cyanidle/gaz_cart")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Radapter plugins for the gaz_cart robot cart")
set(CPACK_PACKAGE_DESCRIPTION   "Native radapter plugins for the gaz_cart differential-drive
robot: transform frame support (gaz_frames), navigation stack (gaz_nav),
and SLAM (gaz_slam).")

# Package only the plugin components — radapter's own install rules in this
# build tree belong to the radapter-headless/radapter-gui debs, not this one.
# Component install is ON: monolithic DEB mode ignores CPACK_COMPONENTS_ALL
# and would bundle radapter-sdk + headers into this package.
set(CPACK_COMPONENTS_ALL             "gaz_nav;gaz_frames;gaz_slam")
set(CPACK_DEB_COMPONENT_INSTALL      ON)
if(NOT DEFINED CPACK_PACKAGE_VERSION)
    set(CPACK_PACKAGE_VERSION    "0.1.0")
endif()
set(CPACK_GENERATOR              "DEB")
set(CPACK_DEBIAN_FILE_NAME       DEB-DEFAULT)
# Runtime deps are declared manually: shlibdeps breaks cross-builds where it
# can't resolve target-arch libs (same reasoning as radapter's Packaging.cmake).
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS OFF)
set(CPACK_DEBIAN_PACKAGE_SECTION "utils")
set(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")

# Architecture mapping
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "arm64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7")
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "armhf")
else()
    set(CPACK_DEBIAN_PACKAGE_ARCHITECTURE "${CMAKE_SYSTEM_PROCESSOR}")
endif()

# Per-component package names and dependencies. These must be set before
# include(CPack) so that CPack reads them when configuring the generators.
set(CPACK_DEBIAN_GAZ_NAV_PACKAGE_NAME    "gaz-nav")
set(CPACK_DEBIAN_GAZ_NAV_PACKAGE_DEPENDS
    "radapter-headless | radapter-gui, libqt6core6, libqt6gui6")

set(CPACK_DEBIAN_GAZ_FRAMES_PACKAGE_NAME "gaz-frames")
set(CPACK_DEBIAN_GAZ_FRAMES_PACKAGE_DEPENDS
    "radapter-headless | radapter-gui, libqt6core6")

set(CPACK_DEBIAN_GAZ_SLAM_PACKAGE_NAME   "gaz-slam")
set(CPACK_DEBIAN_GAZ_SLAM_PACKAGE_DEPENDS
    "radapter-headless | radapter-gui, libqt6core6, libboost-serialization1.83.0, libceres4t64, libgoogle-glog0v6t64, libtbb12")

include(CPack)

# Declaring the component (not just CPACK_COMPONENTS_ALL) is what makes CPack
# actually do a component-filtered install.
cpack_add_component(gaz_nav
    DISPLAY_NAME "gaz_nav plugin"
    DESCRIPTION "Radapter navigation plugin: costmap, planners, lidar"
)
cpack_add_component(gaz_frames
    DISPLAY_NAME "gaz_frames plugin"
    DESCRIPTION "Radapter transform frame support"
)
cpack_add_component(gaz_slam
    DISPLAY_NAME "gaz_slam plugin"
    DESCRIPTION "Radapter SLAM"
)

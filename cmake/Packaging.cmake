# ============================================================================
# CPack DEB packaging for the gaz_nav radapter plugin.
# Included from the top-level CMakeLists — after all add_subdirectory() calls
# so CPACK_* vars leaked by CPM deps are overridden here.
#
#   cmake --build /build --target gaz_nav
#   (cd /build && cpack -G DEB)
#
# Installs libgaz_nav.so into /usr/lib/radapter/plugins; reference it from
# cart.lua's NAV_PLUGINS, e.g. plugins = { nav = "/usr/lib/radapter/plugins/libgaz_nav" }.
# ============================================================================

if(NOT TARGET gaz_nav)
    return()
endif()

set(CPACK_PACKAGE_VENDOR        "cyanidle")
set(CPACK_PACKAGE_CONTACT       "lyosha.doronin@gmail.com")
set(CPACK_PACKAGE_HOMEPAGE_URL  "https://github.com/cyanidle/gaz_cart")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Navigation plugin for the gaz_cart robot cart")
set(CPACK_PACKAGE_DESCRIPTION   "Radapter native plugin (gaz_nav): costmap server, A* global
planner, differential-drive local planner and RPLidar lidar worker
for the gaz_cart differential-drive robot.")

set(CPACK_PACKAGE_NAME           "gaz-nav")
set(CPACK_DEBIAN_PACKAGE_NAME    "${CPACK_PACKAGE_NAME}")
# Package only the plugin component — radapter's own install rules in this
# build tree belong to the radapter-headless/radapter-gui debs, not this one.
set(CPACK_COMPONENTS_ALL         "gaz_nav")
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

# The plugin is dlopen'ed by the radapter runtime and links Qt6 Gui/Core.
# Everything else (rplidar_sdk) is statically linked in.
set(CPACK_DEBIAN_PACKAGE_DEPENDS "radapter-headless | radapter-gui, libqt6core6, libqt6gui6")

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

include(CPack)

# Declaring the component (not just CPACK_COMPONENTS_ALL) is what makes CPack
# actually do a component-filtered install.
cpack_add_component(gaz_nav
    DISPLAY_NAME "gaz_nav plugin"
    DESCRIPTION "Radapter navigation plugin: costmap, planners, lidar"
)

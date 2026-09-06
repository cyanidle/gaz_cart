cmake_minimum_required(VERSION 3.16)

if(NOT BUILD_DIR OR NOT STAGE_DIR OR NOT PKG_TARGET MATCHES "^(cart|gui)$")
    message(FATAL_ERROR "Use the packaging_smoke_<target> CTest target")
endif()
if(NOT STAGE_DIR STREQUAL "${BUILD_DIR}/packaging-smoke")
    message(FATAL_ERROR "Use the packaging_smoke_<target> CTest target")
endif()
file(REMOVE_RECURSE "${STAGE_DIR}")
file(MAKE_DIRECTORY "${STAGE_DIR}/cwd")

if(PKG_TARGET STREQUAL "cart")
    set(components gaz_cart_runtime gaz_frames gaz_nav gaz_slam)
    set(required
        bin/radapter bin/gaz-cart lib/libradapter-sdk.so
        lib/radapter/plugins/libgaz_frames.so lib/radapter/plugins/libgaz_nav.so
        lib/radapter/plugins/libgaz_slam.so share/gaz-cart/cart.lua)
else()
    set(components gaz_gui_runtime)
    set(required
        bin/radapter bin/gaz-gui lib/libradapter-sdk.so
        share/gaz-gui/gui.lua share/gaz-gui/qml/WheelConfigWindow.qml
        share/gaz-gui/mods/config_defs.lua)
endif()

foreach(component IN LISTS components)
    execute_process(
        COMMAND "${CMAKE_COMMAND}" -E env --unset=DESTDIR
            "${CMAKE_COMMAND}" --install "${BUILD_DIR}" --prefix "${STAGE_DIR}/usr"
            --component "${component}" --config "${CONFIG}"
        RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE err)
    if(NOT result STREQUAL "0")
        message(FATAL_ERROR "Install ${component}: ${result}\n${out}\n${err}")
    endif()
endforeach()
foreach(path IN LISTS required)
    if(NOT EXISTS "${STAGE_DIR}/usr/${path}")
        message(FATAL_ERROR "Missing runtime file: ${path}")
    endif()
endforeach()
file(GLOB_RECURSE forbidden "${STAGE_DIR}/usr/include/*" "${STAGE_DIR}/usr/*.a"
    "${STAGE_DIR}/usr/*radapter_test_plugin*" "${STAGE_DIR}/usr/*.service")
if(forbidden)
    message(FATAL_ERROR "Unexpected development/autostart files: ${forbidden}")
endif()
if(PKG_TARGET STREQUAL "gui" AND EXISTS "${STAGE_DIR}/usr/lib/radapter/plugins")
    message(FATAL_ERROR "The gui package must not contain cart plugins")
endif()

if(PKG_TARGET STREQUAL "cart")
    execute_process(
        # Only the staged plugin directory needs relocation; the installed package
        # uses radapter's /usr/lib/radapter/plugins fallback with no environment setup.
        COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
            "QT_PLUGIN_PATH=${STAGE_DIR}/usr/lib/radapter/plugins"
            --unset=LUA_PATH --unset=LUA_CPATH
            "${STAGE_DIR}/usr/bin/radapter" "${STAGE_DIR}/usr/share/gaz-cart/packaging-smoke.lua"
        WORKING_DIRECTORY "${STAGE_DIR}/cwd"
        RESULT_VARIABLE result OUTPUT_VARIABLE out ERROR_VARIABLE err TIMEOUT 15)
    if(NOT result STREQUAL "0" OR NOT "${out}${err}" MATCHES "gaz-cart packaging smoke OK")
        message(FATAL_ERROR "Staged runtime: ${result}\n${out}\n${err}")
    endif()
    message(STATUS "Staged runtime smoke passed: ${STAGE_DIR}\n${out}${err}")
else()
    # The gui runtime needs a display for QML; execution is verified on the
    # target, here we only check that the staged layout is complete.
    message(STATUS "Staged gui layout verified: ${STAGE_DIR}")
endif()

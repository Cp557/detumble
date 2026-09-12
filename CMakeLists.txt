cmake_minimum_required(VERSION 3.25...4.4)

project(
    detumble
    VERSION 0.1.0
    DESCRIPTION "CubeSat attitude-control simulation"
    LANGUAGES CXX
)

option(DETUMBLE_BUILD_VIEWER "Build the desktop viewer target" ON)

set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

set(EIGEN_BUILD_DOC OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(EIGEN_BUILD_PKGCONFIG OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    Eigen
    SYSTEM
    URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
    URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
)
FetchContent_MakeAvailable(Eigen)

include(cmake/CompilerWarnings.cmake)

add_library(
    detumble_core
    STATIC
        src/attitude.cpp
        src/attitude_estimator.cpp
        src/bdot.cpp
        src/coarse_sun_sensor.cpp
        src/dynamics.cpp
        src/environment.cpp
        src/flight_software.cpp
        src/frames.cpp
        src/gyroscope.cpp
        src/initial_conditions.cpp
        src/magnetic_control.cpp
        src/magnetic_field.cpp
        src/magnetometer.cpp
        src/magnetorquer.cpp
        src/mission.cpp
        src/orbit.cpp
        src/reaction_wheel.cpp
        src/sensor_error.cpp
        src/sensor_suite.cpp
        src/simulation.cpp
        src/spacecraft_visual.cpp
        src/sun_pointing.cpp
        src/version.cpp
)
add_library(detumble::core ALIAS detumble_core)

target_compile_features(detumble_core PUBLIC cxx_std_20)
target_include_directories(
    detumble_core
    PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
)
target_link_libraries(detumble_core PUBLIC Eigen3::Eigen)
target_compile_definitions(
    detumble_core
    PRIVATE DETUMBLE_VERSION="${PROJECT_VERSION}"
)
detumble_enable_warnings(detumble_core)

add_executable(detumble_cli apps/cli_main.cpp)
set_target_properties(detumble_cli PROPERTIES OUTPUT_NAME detumble)
target_link_libraries(detumble_cli PRIVATE detumble::core)
detumble_enable_warnings(detumble_cli)

add_executable(detumble_monte_carlo apps/monte_carlo_main.cpp)
target_link_libraries(detumble_monte_carlo PRIVATE detumble::core)
detumble_enable_warnings(detumble_monte_carlo)

if(DETUMBLE_BUILD_VIEWER)
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        raylib
        SYSTEM
        URL https://github.com/raysan5/raylib/archive/refs/tags/6.0.tar.gz
        URL_HASH SHA256=2b3ee1e2120c7a0796b33062c7e9a694dd8a8caa56a96319ac8c8ecf54a90d0b
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_Declare(
        imgui
        SYSTEM
        URL https://github.com/ocornut/imgui/archive/refs/tags/v1.92.7.tar.gz
        URL_HASH SHA256=b21b14ce1ef6dd4d85fa54f68f8449a9be94f3b453aba7fe4ea2a9764e43a5ef
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_Declare(
        rlimgui
        SYSTEM
        URL https://github.com/raylib-extras/rlImGui/archive/3bc5731c4216bb8caa67fbea24aa85ce80d57ccb.tar.gz
        URL_HASH SHA256=2198c4eb4c0a2b8efc2204da046ef014a2c7b736e1483039badee6d19a7ed35a
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(raylib imgui rlimgui)

    add_library(
        detumble_imgui
        STATIC
            ${imgui_SOURCE_DIR}/imgui.cpp
            ${imgui_SOURCE_DIR}/imgui_draw.cpp
            ${imgui_SOURCE_DIR}/imgui_tables.cpp
            ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    )
    target_include_directories(
        detumble_imgui
        SYSTEM PUBLIC ${imgui_SOURCE_DIR}
    )
    target_compile_definitions(
        detumble_imgui
        PUBLIC "ImDrawIdx=unsigned int"
    )

    add_library(detumble_rlimgui STATIC ${rlimgui_SOURCE_DIR}/rlImGui.cpp)
    target_include_directories(
        detumble_rlimgui
        SYSTEM PUBLIC ${rlimgui_SOURCE_DIR}
    )
    target_link_libraries(detumble_rlimgui PUBLIC detumble_imgui raylib)

    add_executable(detumble_viewer apps/viewer_main.cpp)
    target_link_libraries(
        detumble_viewer
        PRIVATE
            detumble::core
            detumble_rlimgui
            raylib
    )
    detumble_enable_warnings(detumble_viewer)

    add_custom_command(
        TARGET detumble_viewer POST_BUILD
        COMMAND
            ${CMAKE_COMMAND} -E make_directory
            $<TARGET_FILE_DIR:detumble_viewer>/assets/models
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${PROJECT_SOURCE_DIR}/assets/models/detumble_3u.glb
            $<TARGET_FILE_DIR:detumble_viewer>/assets/models/detumble_3u.glb
        VERBATIM
    )
endif()

include(CTest)

if(BUILD_TESTING)
    set(CATCH_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
    set(CATCH_INSTALL_EXTRAS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        Catch2
        SYSTEM
        URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.15.3.tar.gz
        URL_HASH SHA256=b0299ae552918220a7a6e21e7de5b714777f4e8c883fb70c4bb23fe01df8c6e3
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(
        detumble_tests
        tests/attitude_test.cpp
        tests/attitude_estimator_test.cpp
        tests/autonomous_flight_software_test.cpp
        tests/bdot_test.cpp
        tests/coarse_sun_sensor_test.cpp
        tests/dynamics_test.cpp
        tests/detumble_validation_test.cpp
        tests/flight_software_test.cpp
        tests/frames_test.cpp
        tests/gyroscope_test.cpp
        tests/initial_conditions_test.cpp
        tests/magnetic_control_test.cpp
        tests/magnetic_field_test.cpp
        tests/magnetometer_test.cpp
        tests/magnetorquer_test.cpp
        tests/mission_validation_test.cpp
        tests/orbit_test.cpp
        tests/reaction_wheel_test.cpp
        tests/robustness_validation_test.cpp
        tests/sensor_error_test.cpp
        tests/simulation_test.cpp
        tests/spacecraft_visual_test.cpp
        tests/smoke_test.cpp
        tests/sun_environment_test.cpp
        tests/sun_pointing_test.cpp
    )
    target_link_libraries(
        detumble_tests
        PRIVATE
            detumble::core
            Catch2::Catch2WithMain
    )
    if(DETUMBLE_BUILD_VIEWER)
        target_sources(
            detumble_tests
            PRIVATE tests/viewer_conventions_test.cpp
        )
        target_link_libraries(detumble_tests PRIVATE raylib)
    endif()
    detumble_enable_warnings(detumble_tests)

    include(Catch)
    catch_discover_tests(detumble_tests)
endif()

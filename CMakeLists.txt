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
    URL https://gitlab.com/libeigen/eigen/-/archive/5.0.1/eigen-5.0.1.tar.gz
    URL_HASH SHA256=e9c326dc8c05cd1e044c71f30f1b2e34a6161a3b6ecf445d56b53ff1669e3dec
    DOWNLOAD_EXTRACT_TIMESTAMP OFF
)
FetchContent_MakeAvailable(Eigen)

include(cmake/CompilerWarnings.cmake)

add_library(detumble_core STATIC src/version.cpp)
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

if(DETUMBLE_BUILD_VIEWER)
    add_executable(detumble_viewer apps/viewer_main.cpp)
    target_link_libraries(detumble_viewer PRIVATE detumble::core)
    detumble_enable_warnings(detumble_viewer)
endif()

include(CTest)

if(BUILD_TESTING)
    set(CATCH_INSTALL_DOCS OFF CACHE BOOL "" FORCE)
    set(CATCH_INSTALL_EXTRAS OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(
        Catch2
        URL https://github.com/catchorg/Catch2/archive/refs/tags/v3.15.3.tar.gz
        URL_HASH SHA256=b0299ae552918220a7a6e21e7de5b714777f4e8c883fb70c4bb23fe01df8c6e3
        DOWNLOAD_EXTRACT_TIMESTAMP OFF
    )
    FetchContent_MakeAvailable(Catch2)

    add_executable(detumble_tests tests/smoke_test.cpp)
    target_link_libraries(
        detumble_tests
        PRIVATE
            detumble::core
            Catch2::Catch2WithMain
    )
    detumble_enable_warnings(detumble_tests)

    include(Catch)
    catch_discover_tests(detumble_tests)
endif()

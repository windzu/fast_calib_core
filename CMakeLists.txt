cmake_minimum_required(VERSION 3.14)

# Project definition
project(fast_calib_core
    VERSION 0.1.0
    DESCRIPTION "ROS-agnostic header-only library for camera-LiDAR calibration"
    HOMEPAGE_URL "https://github.com/hku-mars/FAST-Calib"
    LANGUAGES CXX
)

# Require C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Options
option(FAST_CALIB_CORE_BUILD_TESTS "Build unit tests" ON)
option(FAST_CALIB_CORE_BUILD_EXAMPLES "Build example programs" ON)

# ============================================================================
# Dependencies
# ============================================================================

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io filters search kdtree registration)

# Workaround for FLANN serialization issue with C++17 and newer compilers
# This is a known issue with FLANN headers in some distributions
add_compile_definitions(FLANN_USE_UNORDERED=0)

# Find OpenCV (with aruco module)
find_package(OpenCV REQUIRED)

# Find Eigen3
find_package(Eigen3 REQUIRED)

# ============================================================================
# Library Target (Header-only)
# ============================================================================

add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME}
    INTERFACE
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

target_compile_definitions(${PROJECT_NAME}
    INTERFACE
        ${PCL_DEFINITIONS}
)

# ============================================================================
# Tests
# ============================================================================

if(FAST_CALIB_CORE_BUILD_TESTS)
    enable_testing()
    
    # Try to find GTest
    find_package(GTest QUIET)
    
    if(GTest_FOUND)
        add_subdirectory(test)
    else()
        message(STATUS "GTest not found, tests will not be built")
    endif()
endif()

# ============================================================================
# Examples
# ============================================================================

if(FAST_CALIB_CORE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()

# ============================================================================
# Installation
# ============================================================================

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# Install headers
install(
    DIRECTORY include/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Install targets
install(
    TARGETS ${PROJECT_NAME}
    EXPORT ${PROJECT_NAME}Targets
    INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Generate and install package configuration
install(
    EXPORT ${PROJECT_NAME}Targets
    FILE ${PROJECT_NAME}Targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}Config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

install(
    FILES
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake
        ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

# ============================================================================
# Package Information
# ============================================================================

message(STATUS "")
message(STATUS "fast_calib_core v${PROJECT_VERSION}")
message(STATUS "")
message(STATUS "  Dependencies:")
message(STATUS "    PCL:     ${PCL_VERSION}")
message(STATUS "    OpenCV:  ${OpenCV_VERSION}")
message(STATUS "    Eigen3:  ${EIGEN3_VERSION_STRING}")
message(STATUS "")
message(STATUS "  Build Options:")
message(STATUS "    Tests:    ${FAST_CALIB_CORE_BUILD_TESTS}")
message(STATUS "    Examples: ${FAST_CALIB_CORE_BUILD_EXAMPLES}")
message(STATUS "")

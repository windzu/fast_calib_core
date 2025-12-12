# Tests for fast_calib_core

# Find GTest
find_package(GTest REQUIRED)
include(GoogleTest)

# Test executable
add_executable(test_fast_calib_core
    test_types.cpp
    test_calibration.cpp
    test_detectors.cpp
)

target_link_libraries(test_fast_calib_core
    PRIVATE
        fast_calib_core::fast_calib_core
        GTest::gtest
        GTest::gtest_main
)

# Register tests
gtest_discover_tests(test_fast_calib_core)

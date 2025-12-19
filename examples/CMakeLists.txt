# Examples for fast_calib_core

# Basic calibration example
add_executable(basic_calibration
    basic_calibration.cpp
)

target_link_libraries(basic_calibration
    PRIVATE
        fast_calib_core::fast_calib_core
)

# PCD file calibration example
add_executable(calibrate_from_files
    calibrate_from_files.cpp
)

target_link_libraries(calibrate_from_files
    PRIVATE
        fast_calib_core::fast_calib_core
)

# Config file based calibration (single and multi-scene)
add_executable(calibrate
    calibrate.cpp
)

target_link_libraries(calibrate
    PRIVATE
        fast_calib_core::fast_calib_core
)

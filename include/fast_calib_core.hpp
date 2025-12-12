/*
 * FAST-Calib Core Library
 *
 * Main header file - includes all library components.
 * Include this single header for complete library access.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_HPP
#define FAST_CALIB_CORE_HPP

#include "fast_calib_core/calibration.hpp"
#include "fast_calib_core/lidar_detector.hpp"
#include "fast_calib_core/qr_detector.hpp"
#include "fast_calib_core/types.hpp"

/**
 * @mainpage FAST-Calib Core Library
 *
 * @section intro_sec Introduction
 *
 * FAST-Calib Core is a ROS-agnostic, header-only C++ library for camera-LiDAR
 * extrinsic calibration. It provides the core algorithms for detecting
 * calibration targets in both sensor modalities and computing the rigid
 * transformation between them.
 *
 * @section features_sec Features
 *
 * - Support for both mechanical (ring-based) and solid-state LiDARs
 * - ArUco marker-based calibration target detection
 * - SVD-based extrinsic parameter estimation
 * - Point cloud projection and colorization utilities
 *
 * @section usage_sec Basic Usage
 *
 * @code
 * #include <fast_calib_core.hpp>
 *
 * // Setup parameters
 * fast_calib::CalibParams params;
 * params.target.marker_edge_size = 0.05;
 * params.target.marker_distance = 0.3;
 * params.lidar_type = fast_calib::LiDARType::SolidState;
 *
 * // Detect circles in LiDAR
 * fast_calib::LiDARDetector lidar_det;
 * auto lidar_result = lidar_det.detect(cloud, params);
 *
 * // Detect circles from camera
 * fast_calib::QRDetector qr_det(params.camera);
 * auto qr_result = qr_det.detect(image, params.target);
 *
 * // Compute calibration
 * fast_calib::CalibrationCalculator calc;
 * auto calib_result = calc.compute(lidar_result.centers, qr_result.centers);
 *
 * if (calib_result.success) {
 *     std::cout << "Calibration RMSE: " << calib_result.rmse << " m" <<
 * std::endl;
 *     // Use calib_result.transformation
 * }
 * @endcode
 *
 * @section deps_sec Dependencies
 *
 * - PCL >= 1.10 (Point Cloud Library)
 * - OpenCV >= 4.0 with ArUco module
 * - Eigen3
 *
 * @section license_sec License
 *
 * See LICENSE file in the project root.
 */

#endif  // FAST_CALIB_CORE_HPP

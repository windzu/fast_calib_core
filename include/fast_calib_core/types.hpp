/*
 * FAST-Calib Core Library
 *
 * Core type definitions for camera-lidar extrinsic calibration.
 * This is a header-only, ROS-independent library.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_TYPES_HPP
#define FAST_CALIB_CORE_TYPES_HPP

#define PCL_NO_PRECOMPILE

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <functional>
#include <iostream>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>

namespace fast_calib {

// ============================================================================
// Version Information
// ============================================================================
constexpr int VERSION_MAJOR = 0;
constexpr int VERSION_MINOR = 1;
constexpr int VERSION_PATCH = 0;

inline std::string getVersionString() {
  return std::to_string(VERSION_MAJOR) + "." + std::to_string(VERSION_MINOR) +
         "." + std::to_string(VERSION_PATCH);
}

// ============================================================================
// Constants
// ============================================================================
constexpr int TARGET_NUM_CIRCLES = 4;
constexpr double GEOMETRY_TOLERANCE = 0.08;

// ============================================================================
// Custom Point Type with Ring Information
// ============================================================================
struct PointXYZRing {
  PCL_ADD_POINT4D;         // quad-word XYZ + padding
  std::uint16_t ring = 0;  // ring number for mechanical/multi-line LiDAR
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

}  // namespace fast_calib

// Register custom point type with PCL
POINT_CLOUD_REGISTER_POINT_STRUCT(fast_calib::PointXYZRing,
                                  (float, x, x)(float, y, y)(float, z,
                                                             z)(std::uint16_t,
                                                                ring, ring))

namespace fast_calib {

// ============================================================================
// Enumerations
// ============================================================================

/**
 * @brief LiDAR sensor type
 */
enum class LiDARType : int {
  Unknown = 0,
  Solid = 1,  ///< Solid-state LiDAR (e.g., Livox)
  Mech = 2    ///< Mechanical multi-line LiDAR (e.g., Velodyne, Ouster)
};

/**
 * @brief Camera distortion model type
 */
enum class DistortionModel : int {
  PlumbBob = 0,  ///< plumb_bob (RadTan): k1, k2, p1, p2, k3 (5 coefficients)
  Rational = 1   ///< rational: k1, k2, p1, p2, k3, k4, k5, k6 (8 coefficients)
};

/**
 * @brief Log level for internal logging
 */
enum class LogLevel { Debug, Info, Warning, Error };

// ============================================================================
// Logger Interface (for dependency injection)
// ============================================================================

/**
 * @brief Callback type for logging
 * @param level Log level
 * @param message Log message
 */
using LogCallback =
    std::function<void(LogLevel level, const std::string& message)>;

/**
 * @brief Default logger that prints to stdout
 */
inline void defaultLogger(LogLevel level, const std::string& message) {
  const char* level_str = "";
  switch (level) {
    case LogLevel::Debug:
      level_str = "[DEBUG] ";
      break;
    case LogLevel::Info:
      level_str = "[INFO]  ";
      break;
    case LogLevel::Warning:
      level_str = "[WARN]  ";
      break;
    case LogLevel::Error:
      level_str = "[ERROR] ";
      break;
  }
  std::cout << level_str << message << std::endl;
}

/**
 * @brief Silent logger (no output)
 */
inline void silentLogger(LogLevel /*level*/, const std::string& /*message*/) {
  // Do nothing
}

// ============================================================================
// Camera Intrinsic Parameters
// ============================================================================

/**
 * @brief Camera intrinsic parameters with support for multiple distortion models
 * 
 * Supports:
 * - plumb_bob (RadTan): k1, k2, p1, p2, k3 (5 coefficients)
 * - rational: k1, k2, p1, p2, k3, k4, k5, k6 (8 coefficients)
 */
struct CameraIntrinsics {
  double fx = 1215.31801774424;      ///< Focal length x
  double fy = 1214.72961288138;      ///< Focal length y
  double cx = 1047.86571859677;      ///< Principal point x
  double cy = 745.068353101898;      ///< Principal point y
  
  // Distortion model
  DistortionModel distortion_model = DistortionModel::PlumbBob;
  
  // Distortion coefficients (common to both models)
  double k1 = -0.33574781188503;     ///< Radial distortion k1
  double k2 = 0.10996870793601;      ///< Radial distortion k2
  double p1 = 0.000157303079833973;  ///< Tangential distortion p1
  double p2 = 0.000544930726278493;  ///< Tangential distortion p2
  double k3 = 0.0;                   ///< Radial distortion k3
  
  // Additional coefficients for rational model
  double k4 = 0.0;                   ///< Radial distortion k4 (rational model)
  double k5 = 0.0;                   ///< Radial distortion k5 (rational model)
  double k6 = 0.0;                   ///< Radial distortion k6 (rational model)

  /**
   * @brief Get OpenCV camera matrix (3x3)
   */
  cv::Mat getCameraMatrix() const {
    return (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
  }

  /**
   * @brief Get OpenCV distortion coefficients
   * @return 5-element vector for plumb_bob, 8-element for rational
   */
  cv::Mat getDistCoeffs() const {
    if (distortion_model == DistortionModel::Rational) {
      // rational model: k1, k2, p1, p2, k3, k4, k5, k6
      return (cv::Mat_<double>(1, 8) << k1, k2, p1, p2, k3, k4, k5, k6);
    } else {
      // plumb_bob (RadTan) model: k1, k2, p1, p2, k3
      return (cv::Mat_<double>(1, 5) << k1, k2, p1, p2, k3);
    }
  }
  
  /**
   * @brief Get distortion model name as string
   */
  std::string getDistortionModelName() const {
    return (distortion_model == DistortionModel::Rational) ? "rational" : "plumb_bob";
  }
};

// ============================================================================
// Calibration Target Parameters
// ============================================================================

/**
 * @brief Parameters for the calibration target (ArUco board with circular
 * holes)
 */
struct TargetParams {
  double marker_size = 0.2;  ///< ArUco marker size in meters
  double delta_width_qr_center =
      0.55;  ///< Horizontal distance between QR centers
  double delta_height_qr_center =
      0.35;  ///< Vertical distance between QR centers
  double delta_width_circles =
      0.5;  ///< Horizontal distance between circle centers
  double delta_height_circles =
      0.4;                       ///< Vertical distance between circle centers
  double circle_radius = 0.12;   ///< Circle hole radius
  int min_detected_markers = 3;  ///< Minimum markers for valid detection
};

// ============================================================================
// LiDAR Filter Parameters
// ============================================================================

/**
 * @brief ROI (Region of Interest) filter parameters for LiDAR point cloud
 */
struct LiDARFilterParams {
  double x_min = 1.5;
  double x_max = 3.0;
  double y_min = -1.5;
  double y_max = 2.0;
  double z_min = -0.5;
  double z_max = 2.0;
};

// ============================================================================
// Complete Calibration Parameters
// ============================================================================

/**
 * @brief Complete calibration configuration
 */
struct CalibParams {
  CameraIntrinsics camera;
  TargetParams target;
  LiDARFilterParams lidar_filter;
  std::string output_path = "./output";
};

// ============================================================================
// Calibration Result
// ============================================================================

/**
 * @brief Result of extrinsic calibration
 */
struct CalibrationResult {
  Eigen::Matrix4f transformation =
      Eigen::Matrix4f::Identity();  ///< T_camera_lidar
  double rmse = -1.0;               ///< Root mean square error
  bool success = false;             ///< Whether calibration succeeded
  std::string error_message;        ///< Error message if failed

  /**
   * @brief Get rotation matrix (3x3)
   */
  Eigen::Matrix3f getRotation() const {
    return transformation.block<3, 3>(0, 0);
  }

  /**
   * @brief Get translation vector (3x1)
   */
  Eigen::Vector3f getTranslation() const {
    return transformation.block<3, 1>(0, 3);
  }
};

// ============================================================================
// Detection Results
// ============================================================================

/**
 * @brief Result of QR/ArUco marker detection
 */
struct QRDetectionResult {
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      centers;              ///< Detected circle centers in camera frame
  cv::Mat annotated_image;  ///< Image with detection visualization
  bool success = false;
  std::string error_message;

  QRDetectionResult() : centers(new pcl::PointCloud<pcl::PointXYZ>) {}
};

/**
 * @brief Result of LiDAR circle detection
 */
struct LiDARDetectionResult {
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      centers;  ///< Detected circle centers in LiDAR frame

  // Intermediate results for debugging/visualization
  pcl::PointCloud<PointXYZRing>::Ptr filtered_cloud;  ///< ROI filtered cloud
  pcl::PointCloud<PointXYZRing>::Ptr plane_cloud;     ///< Plane inliers
  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud;  ///< Cloud aligned to Z=0
  pcl::PointCloud<pcl::PointXYZ>::Ptr edge_cloud;     ///< Extracted edge points
  pcl::PointCloud<pcl::PointXYZ>::Ptr
      center_z0_cloud;  ///< Circle centers at Z=0

  bool success = false;
  std::string error_message;

  LiDARDetectionResult()
      : centers(new pcl::PointCloud<pcl::PointXYZ>),
        filtered_cloud(new pcl::PointCloud<PointXYZRing>),
        plane_cloud(new pcl::PointCloud<PointXYZRing>),
        aligned_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        edge_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        center_z0_cloud(new pcl::PointCloud<pcl::PointXYZ>) {}
};

// ============================================================================
// Type Aliases for Convenience
// ============================================================================
using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudXYZPtr = pcl::PointCloud<pcl::PointXYZ>::Ptr;
using PointCloudRing = pcl::PointCloud<PointXYZRing>;
using PointCloudRingPtr = pcl::PointCloud<PointXYZRing>::Ptr;
using PointCloudXYZRGB = pcl::PointCloud<pcl::PointXYZRGB>;
using PointCloudXYZRGBPtr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr;

}  // namespace fast_calib

// ============================================================================
// PCL Template Explicit Instantiations
// Required for custom point type with PCL algorithms
// These must come AFTER the POINT_CLOUD_REGISTER_POINT_STRUCT
// ============================================================================

#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>

// Include PCL implementation files for template instantiation
#include <pcl/impl/pcl_base.hpp>
#include <pcl/filters/impl/passthrough.hpp>
#include <pcl/filters/impl/voxel_grid.hpp>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/impl/filter_indices.hpp>
#include <pcl/filters/impl/extract_indices.hpp>
#include <pcl/segmentation/impl/sac_segmentation.hpp>

#endif  // FAST_CALIB_CORE_TYPES_HPP

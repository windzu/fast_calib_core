/*
 * FAST-Calib Core Library
 *
 * Calibration computation and utility functions.
 * Includes SVD-based extrinsic estimation, point cloud projection, and result
 * saving.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_CALIBRATION_HPP
#define FAST_CALIB_CORE_CALIBRATION_HPP

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>

#include "types.hpp"

namespace fast_calib {

// ============================================================================
// Point Cloud Sorting Utilities
// ============================================================================

/**
 * @brief Sort pattern centers to ensure consistent ordering across scenes
 *
 * This implementation follows the original FAST-Calib approach:
 * 1. For LiDAR points, first transform to camera-like coordinate system
 * 2. Use atan2 angle-based sorting around centroid
 * 3. Ensure consistent counter-clockwise ordering
 * 4. For LiDAR, transform back to original coordinate system
 *
 * The key insight is that both camera and LiDAR points are sorted in a
 * unified coordinate system to ensure consistent point correspondence.
 *
 * @param input Input point cloud (4 points)
 * @param output Sorted output point cloud
 * @param sensor_type "camera" or "lidar"
 */
inline void sortPatternCenters(const PointCloudXYZPtr& input,
                               PointCloudXYZPtr& output,
                               const std::string& sensor_type) {
  output->clear();
  output->resize(4);

  if (input->size() != TARGET_NUM_CIRCLES) {
    std::cerr << "[sortPatternCenters] Number of " << sensor_type
              << " center points to be sorted is not 4." << std::endl;
    *output = *input;
    return;
  }

  // Create working point cloud - transform LiDAR to camera-like coordinates
  PointCloudXYZPtr work_pc(new PointCloudXYZ);

  if (sensor_type == "lidar") {
    // LiDAR to Camera-like coordinate transformation:
    // LiDAR: X-forward, Y-left, Z-up
    // Camera: X-right, Y-down, Z-forward
    // Transform: cam_x = -lidar_y, cam_y = -lidar_z, cam_z = lidar_x
    for (const auto& p : *input) {
      pcl::PointXYZ pt;
      pt.x = -p.y;  // LiDAR Y -> Camera -X
      pt.y = -p.z;  // LiDAR Z -> Camera -Y
      pt.z = p.x;   // LiDAR X -> Camera Z
      work_pc->push_back(pt);
    }
  } else {
    *work_pc = *input;
  }

  // Calculate centroid
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(*work_pc, centroid);
  pcl::PointXYZ ref_origin(centroid[0], centroid[1], centroid[2]);

  // Calculate angles using atan2 for each point relative to centroid
  // Project to XY plane (camera image plane)
  std::vector<std::pair<float, int>> angle_idx_pairs;
  for (size_t i = 0; i < work_pc->size(); ++i) {
    const auto& p = work_pc->points[i];
    float rel_x = p.x - ref_origin.x;
    float rel_y = p.y - ref_origin.y;
    float angle = std::atan2(rel_y, rel_x);
    angle_idx_pairs.emplace_back(angle, static_cast<int>(i));
  }

  // Sort by angle (this gives counter-clockwise or clockwise ordering)
  std::sort(angle_idx_pairs.begin(), angle_idx_pairs.end());

  // Copy sorted points to temporary vector
  std::vector<pcl::PointXYZ> sorted_work(4);
  for (int i = 0; i < 4; ++i) {
    sorted_work[i] = work_pc->points[angle_idx_pairs[i].second];
  }

  // Verify counter-clockwise ordering and fix if necessary
  // Check cross product of consecutive edge vectors
  const auto& p0 = sorted_work[0];
  const auto& p1 = sorted_work[1];
  const auto& p2 = sorted_work[2];

  Eigen::Vector3f v01(p1.x - p0.x, p1.y - p0.y, 0);
  Eigen::Vector3f v12(p2.x - p1.x, p2.y - p1.y, 0);

  // If cross product Z > 0, it's counter-clockwise, swap to make clockwise
  // (or vice versa - the key is consistency)
  if (v01.cross(v12).z() > 0) {
    std::swap(sorted_work[1], sorted_work[3]);
  }

  // Transform back to original LiDAR coordinates if needed
  if (sensor_type == "lidar") {
    for (int i = 0; i < 4; ++i) {
      const auto& pt = sorted_work[i];
      // Inverse transform: lidar_x = cam_z, lidar_y = -cam_x, lidar_z = -cam_y
      output->points[i].x = pt.z;   // Camera Z -> LiDAR X
      output->points[i].y = -pt.x;  // Camera -X -> LiDAR Y
      output->points[i].z = -pt.y;  // Camera -Y -> LiDAR Z
    }
  } else {
    for (int i = 0; i < 4; ++i) {
      output->points[i] = sorted_work[i];
    }
  }
}

// ============================================================================
// Calibration Calculator
// ============================================================================

/**
 * @brief Scene data for multi-scene calibration
 *
 * Contains the detected circle centers from both LiDAR and camera for one
 * scene. Each scene should have exactly 4 points (TARGET_NUM_CIRCLES).
 */
struct SceneData {
  PointCloudXYZPtr lidar_centers;  ///< Circle centers detected in LiDAR frame
  PointCloudXYZPtr qr_centers;     ///< Circle centers detected in camera frame

  SceneData()
      : lidar_centers(new PointCloudXYZ), qr_centers(new PointCloudXYZ) {}

  SceneData(const PointCloudXYZPtr& lidar, const PointCloudXYZPtr& qr)
      : lidar_centers(lidar), qr_centers(qr) {}

  /**
   * @brief Check if scene data is valid
   * @return true if both point clouds have exactly TARGET_NUM_CIRCLES points
   */
  bool isValid() const {
    return lidar_centers && qr_centers &&
           lidar_centers->size() == TARGET_NUM_CIRCLES &&
           qr_centers->size() == TARGET_NUM_CIRCLES;
  }
};

/**
 * @brief Computes extrinsic calibration from LiDAR to camera
 *
 * Supports both single-scene and multi-scene calibration.
 * For multi-scene calibration, point correspondences from all scenes are
 * merged and jointly optimized using weighted SVD.
 */
class CalibrationCalculator {
 public:
  /**
   * @brief Construct a calibration calculator
   * @param logger Optional logging callback
   */
  explicit CalibrationCalculator(LogCallback logger = defaultLogger)
      : logger_(logger) {}

  /**
   * @brief Compute extrinsic transformation from LiDAR to camera (single scene)
   * @param lidar_centers Circle centers detected in LiDAR frame
   * @param qr_centers Circle centers detected in camera frame
   * @return Calibration result with transformation matrix and RMSE
   */
  CalibrationResult compute(const PointCloudXYZPtr& lidar_centers,
                            const PointCloudXYZPtr& qr_centers) {
    CalibrationResult result;

    if (lidar_centers->size() != TARGET_NUM_CIRCLES ||
        qr_centers->size() != TARGET_NUM_CIRCLES) {
      result.error_message = "Point cloud sizes don't match expected count (" +
                             std::to_string(TARGET_NUM_CIRCLES) + ")";
      return result;
    }

    // Sort centers for consistent pairing
    PointCloudXYZPtr sorted_lidar(new PointCloudXYZ);
    PointCloudXYZPtr sorted_qr(new PointCloudXYZ);
    sortPatternCenters(lidar_centers, sorted_lidar, "lidar");
    sortPatternCenters(qr_centers, sorted_qr, "camera");

    // Estimate rigid transformation using SVD
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>
        svd;
    svd.estimateRigidTransformation(*sorted_lidar, *sorted_qr,
                                    result.transformation);

    // Compute RMSE
    PointCloudXYZPtr aligned_lidar(new PointCloudXYZ);
    alignPointCloud(sorted_lidar, aligned_lidar, result.transformation);
    result.rmse = computeRMSE(sorted_qr, aligned_lidar);

    if (result.rmse >= 0) {
      log(LogLevel::Info,
          "Calibration RMSE: " + std::to_string(result.rmse) + " m");
      result.success = true;
    } else {
      result.error_message = "RMSE computation failed";
    }

    return result;
  }

  /**
   * @brief Compute extrinsic transformation from multiple scenes (joint
   * optimization)
   *
   * This method merges point correspondences from all scenes and performs
   * a joint weighted SVD optimization to find the best rigid transformation.
   *
   * @param scenes Vector of scene data, each containing lidar and camera
   * centers
   * @param weights Optional weights for each scene (nullptr for uniform
   * weights)
   * @param sort_points If true, sort points for consistent pairing (default).
   *                    Set to false if points are already correctly paired.
   * @return Calibration result with transformation matrix and RMSE
   */
  CalibrationResult computeMultiScene(
      const std::vector<SceneData>& scenes,
      const std::vector<double>* weights = nullptr, bool sort_points = true) {
    CalibrationResult result;

    // Validate input
    if (scenes.empty()) {
      result.error_message = "No scenes provided";
      return result;
    }

    if (scenes.size() < 2) {
      result.error_message =
          "Multi-scene calibration requires at least 2 scenes";
      return result;
    }

    // Validate weights if provided
    if (weights && weights->size() != scenes.size()) {
      result.error_message = "Weights size doesn't match number of scenes";
      return result;
    }

    // Collect all point correspondences
    std::vector<Eigen::Vector3d> all_lidar_pts;
    std::vector<Eigen::Vector3d> all_camera_pts;
    std::vector<double> all_weights;

    for (size_t scene_idx = 0; scene_idx < scenes.size(); ++scene_idx) {
      const auto& scene = scenes[scene_idx];

      if (!scene.isValid()) {
        result.error_message = "Scene " + std::to_string(scene_idx) +
                               " has invalid data (expected " +
                               std::to_string(TARGET_NUM_CIRCLES) + " points)";
        return result;
      }

      PointCloudXYZPtr lidar_pts = scene.lidar_centers;
      PointCloudXYZPtr camera_pts = scene.qr_centers;

      // Optionally sort centers for consistent pairing
      if (sort_points) {
        PointCloudXYZPtr sorted_lidar(new PointCloudXYZ);
        PointCloudXYZPtr sorted_qr(new PointCloudXYZ);
        sortPatternCenters(scene.lidar_centers, sorted_lidar, "lidar");
        sortPatternCenters(scene.qr_centers, sorted_qr, "camera");
        lidar_pts = sorted_lidar;
        camera_pts = sorted_qr;
      }

      // Get scene weight
      double scene_weight = weights ? (*weights)[scene_idx] : 1.0;

      // Add points from this scene
      for (size_t i = 0; i < lidar_pts->size(); ++i) {
        const auto& lidar_pt = lidar_pts->points[i];
        const auto& camera_pt = camera_pts->points[i];

        all_lidar_pts.emplace_back(lidar_pt.x, lidar_pt.y, lidar_pt.z);
        all_camera_pts.emplace_back(camera_pt.x, camera_pt.y, camera_pt.z);
        all_weights.push_back(scene_weight);
      }
    }

    log(LogLevel::Info, "Multi-scene calibration with " +
                            std::to_string(scenes.size()) + " scenes, " +
                            std::to_string(all_lidar_pts.size()) +
                            " point correspondences");

    // Compute weighted rigid transformation using SVD
    result =
        solveRigidTransformWeighted(all_lidar_pts, all_camera_pts, all_weights);

    if (result.success) {
      log(LogLevel::Info, "Multi-scene calibration RMSE: " +
                              std::to_string(result.rmse) + " m");
    }

    return result;
  }

  /**
   * @brief Solve rigid transformation using weighted SVD
   *
   * Computes the optimal rigid transformation (R, t) that minimizes:
   *   sum_i w_i * ||R * lidar_i + t - camera_i||^2
   *
   * Uses the weighted Procrustes/Wahba algorithm with SVD decomposition.
   *
   * @param lidar_pts Source points in LiDAR frame
   * @param camera_pts Target points in camera frame
   * @param weights Weights for each point correspondence
   * @return Calibration result with transformation matrix and weighted RMSE
   */
  static CalibrationResult solveRigidTransformWeighted(
      const std::vector<Eigen::Vector3d>& lidar_pts,
      const std::vector<Eigen::Vector3d>& camera_pts,
      const std::vector<double>& weights) {
    CalibrationResult result;

    const size_t N = lidar_pts.size();
    if (N < 3 || camera_pts.size() != N || weights.size() != N) {
      result.error_message =
          "Invalid input: need at least 3 points with matching sizes";
      return result;
    }

    // Compute total weight
    double weight_sum = 0.0;
    for (const auto& w : weights) {
      weight_sum += w;
    }
    if (weight_sum <= 0) {
      result.error_message = "Total weight must be positive";
      return result;
    }

    // Compute weighted centroids
    Eigen::Vector3d centroid_lidar = Eigen::Vector3d::Zero();
    Eigen::Vector3d centroid_camera = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      centroid_lidar += weights[i] * lidar_pts[i];
      centroid_camera += weights[i] * camera_pts[i];
    }
    centroid_lidar /= weight_sum;
    centroid_camera /= weight_sum;

    // Build weighted cross-covariance matrix
    Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d p = lidar_pts[i] - centroid_lidar;
      Eigen::Vector3d q = camera_pts[i] - centroid_camera;
      H += weights[i] * (p * q.transpose());
    }

    // SVD decomposition: H = U * S * V^T
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(
        H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Rotation: R = V * U^T
    Eigen::Matrix3d R = V * U.transpose();

    // Handle reflection case (det(R) = -1)
    if (R.determinant() < 0) {
      Eigen::Matrix3d D = Eigen::Matrix3d::Identity();
      D(2, 2) = -1;
      R = V * D * U.transpose();
    }

    // Translation: t = centroid_camera - R * centroid_lidar
    Eigen::Vector3d t = centroid_camera - R * centroid_lidar;

    // Build 4x4 transformation matrix
    result.transformation = Eigen::Matrix4f::Identity();
    result.transformation.block<3, 3>(0, 0) = R.cast<float>();
    result.transformation.block<3, 1>(0, 3) = t.cast<float>();

    // Compute weighted RMSE
    double weighted_rss = 0.0;
    for (size_t i = 0; i < N; ++i) {
      Eigen::Vector3d transformed = R * lidar_pts[i] + t;
      Eigen::Vector3d residual = transformed - camera_pts[i];
      weighted_rss += weights[i] * residual.squaredNorm();
    }
    result.rmse = std::sqrt(weighted_rss / weight_sum);
    result.success = true;

    return result;
  }

  /**
   * @brief Compute RMSE between two point clouds
   * @return RMSE value, or -1.0 if sizes don't match
   */
  static double computeRMSE(const PointCloudXYZPtr& cloud1,
                            const PointCloudXYZPtr& cloud2) {
    if (cloud1->size() != cloud2->size()) {
      return -1.0;
    }

    double sum = 0.0;
    for (size_t i = 0; i < cloud1->size(); ++i) {
      double dx = cloud1->points[i].x - cloud2->points[i].x;
      double dy = cloud1->points[i].y - cloud2->points[i].y;
      double dz = cloud1->points[i].z - cloud2->points[i].z;
      sum += dx * dx + dy * dy + dz * dz;
    }

    return std::sqrt(sum / cloud1->size());
  }

  /**
   * @brief Transform point cloud by given transformation matrix
   */
  static void alignPointCloud(const PointCloudXYZPtr& input,
                              PointCloudXYZPtr& output,
                              const Eigen::Matrix4f& transformation) {
    output->clear();
    output->reserve(input->size());

    for (const auto& pt : input->points) {
      Eigen::Vector4f pt_homogeneous(pt.x, pt.y, pt.z, 1.0f);
      Eigen::Vector4f transformed_pt = transformation * pt_homogeneous;
      output->push_back(pcl::PointXYZ(transformed_pt(0), transformed_pt(1),
                                      transformed_pt(2)));
    }
  }

 private:
  LogCallback logger_;

  void log(LogLevel level, const std::string& message) {
    if (logger_) {
      logger_(level, "[Calibration] " + message);
    }
  }
};

// ============================================================================
// Point Cloud Projection to Image
// ============================================================================

/**
 * @brief Project LiDAR point cloud to image and colorize
 * @param cloud Input point cloud (with ring info)
 * @param transformation T_camera_lidar transformation
 * @param cameraMatrix Camera intrinsic matrix
 * @param distCoeffs Distortion coefficients
 * @param image Color image for texturing
 * @param colored_cloud Output colored point cloud
 */
inline void projectPointCloudToImage(const PointCloudRingPtr& cloud,
                                     const Eigen::Matrix4f& transformation,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs,
                                     const cv::Mat& image,
                                     PointCloudXYZRGBPtr& colored_cloud) {
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);

  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) {
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    // Skip points behind the camera
    if (transformed_point(2) < 0) continue;

    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1),
                                  transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs,
                      imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    if (u >= 0 && u < undistortedImage.cols && v >= 0 &&
        v < undistortedImage.rows) {
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

/**
 * @brief Project point cloud to image (overload for PointXYZ)
 */
inline void projectPointCloudToImage(const PointCloudXYZPtr& cloud,
                                     const Eigen::Matrix4f& transformation,
                                     const cv::Mat& cameraMatrix,
                                     const cv::Mat& distCoeffs,
                                     const cv::Mat& image,
                                     PointCloudXYZRGBPtr& colored_cloud) {
  colored_cloud->clear();
  colored_cloud->reserve(cloud->size());

  cv::Mat undistortedImage;
  cv::undistort(image, undistortedImage, cameraMatrix, distCoeffs);

  cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
  cv::Mat zeroDistCoeffs = cv::Mat::zeros(5, 1, CV_64F);

  std::vector<cv::Point3f> objectPoints(1);
  std::vector<cv::Point2f> imagePoints(1);

  for (const auto& point : *cloud) {
    Eigen::Vector4f homogeneous_point(point.x, point.y, point.z, 1.0f);
    Eigen::Vector4f transformed_point = transformation * homogeneous_point;

    if (transformed_point(2) < 0) continue;

    objectPoints[0] = cv::Point3f(transformed_point(0), transformed_point(1),
                                  transformed_point(2));
    cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, zeroDistCoeffs,
                      imagePoints);

    int u = static_cast<int>(imagePoints[0].x);
    int v = static_cast<int>(imagePoints[0].y);

    if (u >= 0 && u < undistortedImage.cols && v >= 0 &&
        v < undistortedImage.rows) {
      cv::Vec3b color = undistortedImage.at<cv::Vec3b>(v, u);

      pcl::PointXYZRGB colored_point;
      colored_point.x = transformed_point(0);
      colored_point.y = transformed_point(1);
      colored_point.z = transformed_point(2);
      colored_point.r = color[2];
      colored_point.g = color[1];
      colored_point.b = color[0];
      colored_cloud->push_back(colored_point);
    }
  }
}

// ============================================================================
// Result Saving Utilities
// ============================================================================

/**
 * @brief Utility class for saving calibration results
 */
class ResultSaver {
 public:
  /**
   * @brief Construct a result saver
   * @param output_path Directory to save results
   */
  explicit ResultSaver(const std::string& output_path)
      : output_path_(output_path) {
    if (!output_path_.empty() && output_path_.back() != '/') {
      output_path_ += '/';
    }
  }

  /**
   * @brief Save circle center records for multi-scene calibration
   */
  bool saveCircleCenters(const PointCloudXYZPtr& lidar_centers,
                         const PointCloudXYZPtr& qr_centers) {
    if (lidar_centers->size() != 4 || qr_centers->size() != 4) {
      return false;
    }

    std::ofstream file(output_path_ + "circle_center_record.txt",
                       std::ios::app);
    if (!file.is_open()) {
      return false;
    }

    // Write timestamp
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    file << "time: "
         << std::put_time(std::localtime(&now_time), "%Y-%m-%d %H:%M:%S")
         << std::endl;

    file << "lidar_centers:";
    for (const auto& pt : lidar_centers->points) {
      file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    file << std::endl;

    file << "qr_centers:";
    for (const auto& pt : qr_centers->points) {
      file << " {" << pt.x << "," << pt.y << "," << pt.z << "}";
    }
    file << std::endl;

    return true;
  }

  /**
   * @brief Save complete calibration results
   */
  bool saveCalibrationResults(const CalibParams& params,
                              const CalibrationResult& result,
                              const PointCloudXYZRGBPtr& colored_cloud,
                              const cv::Mat& annotated_image) {
    // Save transformation to text file
    std::ofstream outFile(output_path_ + "single_calib_result.txt");
    if (!outFile.is_open()) {
      return false;
    }

    outFile << "# FAST-Calib result\n";
    outFile << "# Generated by fast_calib_core v" << getVersionString()
            << "\n\n";

    outFile << "cam_model: Pinhole\n";
    outFile << "cam_width: " << annotated_image.cols << "\n";
    outFile << "cam_height: " << annotated_image.rows << "\n";
    outFile << "scale: 1.0\n";
    outFile << "cam_fx: " << params.camera.fx << "\n";
    outFile << "cam_fy: " << params.camera.fy << "\n";
    outFile << "cam_cx: " << params.camera.cx << "\n";
    outFile << "cam_cy: " << params.camera.cy << "\n";
    outFile << "cam_d0: " << params.camera.k1 << "\n";
    outFile << "cam_d1: " << params.camera.k2 << "\n";
    outFile << "cam_d2: " << params.camera.p1 << "\n";
    outFile << "cam_d3: " << params.camera.p2 << "\n";

    const auto& T = result.transformation;
    outFile << "\nRcl: [" << std::fixed << std::setprecision(9);
    outFile << std::setw(12) << T(0, 0) << ", " << std::setw(12) << T(0, 1)
            << ", " << std::setw(12) << T(0, 2) << ",\n";
    outFile << "      " << std::setw(12) << T(1, 0) << ", " << std::setw(12)
            << T(1, 1) << ", " << std::setw(12) << T(1, 2) << ",\n";
    outFile << "      " << std::setw(12) << T(2, 0) << ", " << std::setw(12)
            << T(2, 1) << ", " << std::setw(12) << T(2, 2) << "]\n";

    outFile << "Pcl: [";
    outFile << std::setw(12) << T(0, 3) << ", " << std::setw(12) << T(1, 3)
            << ", " << std::setw(12) << T(2, 3) << "]\n";

    outFile << "\nRMSE: " << result.rmse << " m\n";
    outFile.close();

    // Save annotated image
    if (!annotated_image.empty()) {
      cv::imwrite(output_path_ + "annotated_image.png", annotated_image);
    }

    // Save colored point cloud
    if (colored_cloud && !colored_cloud->empty()) {
      pcl::io::savePCDFileBinary(output_path_ + "colored_cloud.pcd",
                                 *colored_cloud);
    }

    return true;
  }

  /**
   * @brief Save transformation matrix to file
   */
  bool saveTransformationMatrix(
      const Eigen::Matrix4f& T,
      const std::string& filename = "transformation.txt") {
    std::ofstream file(output_path_ + filename);
    if (!file.is_open()) {
      return false;
    }

    file << std::fixed << std::setprecision(9);
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        file << T(i, j);
        if (j < 3) file << " ";
      }
      file << "\n";
    }

    return true;
  }

 private:
  std::string output_path_;
};

}  // namespace fast_calib

#endif  // FAST_CALIB_CORE_CALIBRATION_HPP

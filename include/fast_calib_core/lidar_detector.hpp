/*
 * FAST-Calib Core Library
 *
 * LiDAR circle detection for calibration target.
 * Supports both mechanical (multi-line) and solid-state LiDAR.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_LIDAR_DETECTOR_HPP
#define FAST_CALIB_CORE_LIDAR_DETECTOR_HPP

// Workaround for FLANN serialization issue with C++17 and newer compilers
// This must be included before any PCL headers that use kdtree
#include <unordered_map>
#include <vector>
namespace flann {
namespace serialization {
template <class Archive, class Key, class Value>
void serialize(Archive& ar, std::unordered_map<Key, std::vector<Value>>& map) {
  (void)ar;
  (void)map;
}
}  // namespace serialization
}  // namespace flann

#include <pcl/features/boundary.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <algorithm>
#include <cmath>

#include "types.hpp"

namespace fast_calib {

// ============================================================================
// Square Geometry Validator
// ============================================================================

/**
 * @brief Validates if a set of 4 points forms a rectangle with expected
 * dimensions
 */
class SquareValidator {
 public:
  SquareValidator(const std::vector<pcl::PointXYZ>& candidates,
                  double delta_width, double delta_height)
      : candidates_(candidates),
        delta_width_(delta_width),
        delta_height_(delta_height) {}

  /**
   * @brief Check if the candidates form a valid rectangle
   * @return true if geometry matches expected dimensions within tolerance
   */
  bool isValid() const {
    if (candidates_.size() != TARGET_NUM_CIRCLES) {
      return false;
    }

    // Calculate all pairwise distances (C(4,2) = 6)
    std::vector<double> distances;
    distances.reserve(6);

    for (size_t i = 0; i < candidates_.size(); ++i) {
      for (size_t j = i + 1; j < candidates_.size(); ++j) {
        double dx = candidates_[i].x - candidates_[j].x;
        double dy = candidates_[i].y - candidates_[j].y;
        double dz = candidates_[i].z - candidates_[j].z;
        distances.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
      }
    }

    std::sort(distances.begin(), distances.end());

    // Expected: 2 widths, 2 heights, 2 diagonals
    double expected_width = delta_width_;
    double expected_height = delta_height_;
    double expected_diag =
        std::sqrt(delta_width_ * delta_width_ + delta_height_ * delta_height_);

    std::vector<double> expected = {expected_height, expected_height,
                                    expected_width,  expected_width,
                                    expected_diag,   expected_diag};
    std::sort(expected.begin(), expected.end());

    for (size_t i = 0; i < distances.size(); ++i) {
      if (std::abs(distances[i] - expected[i]) > GEOMETRY_TOLERANCE) {
        return false;
      }
    }

    return true;
  }

 private:
  std::vector<pcl::PointXYZ> candidates_;
  double delta_width_;
  double delta_height_;
};

// ============================================================================
// LiDAR Detector Class
// ============================================================================

/**
 * @brief Detects circular holes in calibration target from LiDAR point cloud
 *
 * Supports two detection modes:
 * - Mechanical LiDAR: Uses ring information for edge detection
 * - Solid-state LiDAR: Uses boundary estimation for edge detection
 */
class LiDARDetector {
 public:
  /**
   * @brief Construct a LiDAR detector
   * @param params Calibration parameters
   * @param logger Optional logging callback
   */
  explicit LiDARDetector(const CalibParams& params,
                         LogCallback logger = defaultLogger)
      : params_(params), logger_(logger) {}

  /**
   * @brief Detect circle centers for mechanical LiDAR (with ring information)
   * @param cloud Input point cloud with ring field
   * @return Detection result with circle centers
   */
  LiDARDetectionResult detectMechanical(const PointCloudRingPtr& cloud) {
    LiDARDetectionResult result;

    if (!cloud || cloud->empty()) {
      result.error_message = "Input cloud is empty";
      return result;
    }

    log(LogLevel::Info, "Starting mechanical LiDAR detection...");

    // 1. Apply passthrough filters
    filterCloud(cloud, result.filtered_cloud);
    log(LogLevel::Info, "Depth filtered cloud size: " +
                            std::to_string(result.filtered_cloud->size()));

    if (result.filtered_cloud->empty()) {
      result.error_message = "Filtered cloud is empty";
      return result;
    }

    // 2. Fit plane and extract normal
    Eigen::Vector4f plane_coeffs;
    if (!fitPlane(result.filtered_cloud, result.plane_cloud, plane_coeffs)) {
      result.error_message = "Failed to fit plane";
      return result;
    }
    log(LogLevel::Info,
        "Plane cloud size: " + std::to_string(result.plane_cloud->size()));

    // 3. Extract edge points using ring-based neighbor distance
    Eigen::Matrix3d R_align;
    float average_z;
    extractEdgePointsMech(result.filtered_cloud, plane_coeffs,
                          result.edge_cloud, result.aligned_cloud, R_align,
                          average_z);
    log(LogLevel::Info,
        "Extracted " + std::to_string(result.edge_cloud->size()) +
            " edge points (mechanical LiDAR by neighbor distance).");

    // 4. Detect circles using RANSAC
    detectCirclesRANSAC(result.aligned_cloud, result.center_z0_cloud);

    // 5. Geometric consistency check and transform back
    if (!selectAndTransformCenters(result.center_z0_cloud, R_align, average_z,
                                   result.centers)) {
      result.error_message = "Failed to find valid circle pattern";
      return result;
    }

    result.success = (result.centers->size() == TARGET_NUM_CIRCLES);
    if (!result.success) {
      result.error_message = "Did not find exactly 4 circles";
    }

    return result;
  }

  /**
   * @brief Detect circle centers for solid-state LiDAR (without ring
   * information)
   * @param cloud Input point cloud
   * @return Detection result with circle centers
   */
  LiDARDetectionResult detectSolidState(const PointCloudRingPtr& cloud) {
    LiDARDetectionResult result;

    if (!cloud || cloud->empty()) {
      result.error_message = "Input cloud is empty";
      return result;
    }

    log(LogLevel::Info, "Starting solid-state LiDAR detection...");

    // 1. Apply passthrough filters
    filterCloud(cloud, result.filtered_cloud);
    log(LogLevel::Info, "Filtered cloud size: " +
                            std::to_string(result.filtered_cloud->size()));

    // 1.5 Voxel grid downsampling
    pcl::VoxelGrid<PointXYZRing> voxel_filter;
    voxel_filter.setInputCloud(result.filtered_cloud);
    voxel_filter.setLeafSize(0.005f, 0.005f, 0.005f);
    voxel_filter.filter(*result.filtered_cloud);
    log(LogLevel::Info,
        "After voxel filter: " + std::to_string(result.filtered_cloud->size()));

    if (result.filtered_cloud->empty()) {
      result.error_message = "Filtered cloud is empty";
      return result;
    }

    // 2. Fit plane
    Eigen::Vector4f plane_coeffs;
    if (!fitPlane(result.filtered_cloud, result.plane_cloud, plane_coeffs)) {
      result.error_message = "Failed to fit plane";
      return result;
    }
    log(LogLevel::Info,
        "Plane cloud size: " + std::to_string(result.plane_cloud->size()));

    // 3. Align plane to Z=0 and extract boundary points
    Eigen::Matrix3d R_align;
    float average_z;
    alignPlaneToZ0(result.plane_cloud, plane_coeffs, result.aligned_cloud,
                   R_align, average_z);

    // 4. Extract edge points using boundary estimation
    extractEdgePointsBoundary(result.aligned_cloud, result.edge_cloud);
    log(LogLevel::Info, "Extracted " +
                            std::to_string(result.edge_cloud->size()) +
                            " edge points.");

    // 5. Cluster edge points
    std::vector<pcl::PointIndices> cluster_indices;
    clusterEdgePoints(result.edge_cloud, cluster_indices);
    log(LogLevel::Info,
        "Number of edge clusters: " + std::to_string(cluster_indices.size()));

    // 6. Fit circles to each cluster
    Eigen::Matrix3d R_inv = R_align.inverse();
    fitCirclesToClusters(result.edge_cloud, cluster_indices, R_inv, average_z,
                         result.center_z0_cloud, result.centers);

    result.success = (result.centers->size() == TARGET_NUM_CIRCLES);
    if (!result.success) {
      result.error_message = "Did not find exactly 4 circles, found: " +
                             std::to_string(result.centers->size());
    }

    return result;
  }

  /**
   * @brief Auto-detect LiDAR type and process accordingly
   * @param cloud Input point cloud
   * @param type LiDAR type
   * @return Detection result with circle centers
   */
  LiDARDetectionResult detect(const PointCloudRingPtr& cloud, LiDARType type) {
    switch (type) {
      case LiDARType::Mech:
        return detectMechanical(cloud);
      case LiDARType::Solid:
        return detectSolidState(cloud);
      default:
        LiDARDetectionResult result;
        result.error_message = "Unknown LiDAR type";
        return result;
    }
  }

 private:
  CalibParams params_;
  LogCallback logger_;

  void log(LogLevel level, const std::string& message) {
    if (logger_) {
      logger_(level, "[LiDARDetector] " + message);
    }
  }

  // Filter point cloud by XYZ bounds
  void filterCloud(const PointCloudRingPtr& input, PointCloudRingPtr& output) {
    output->clear();
    output->reserve(input->size());

    pcl::PassThrough<PointXYZRing> pass_x;
    pass_x.setInputCloud(input);
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(params_.lidar_filter.x_min,
                           params_.lidar_filter.x_max);
    pass_x.filter(*output);

    pcl::PassThrough<PointXYZRing> pass_y;
    pass_y.setInputCloud(output);
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(params_.lidar_filter.y_min,
                           params_.lidar_filter.y_max);
    pass_y.filter(*output);

    pcl::PassThrough<PointXYZRing> pass_z;
    pass_z.setInputCloud(output);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(params_.lidar_filter.z_min,
                           params_.lidar_filter.z_max);
    pass_z.filter(*output);
  }

  // Fit plane using RANSAC
  bool fitPlane(const PointCloudRingPtr& input, PointCloudRingPtr& plane_cloud,
                Eigen::Vector4f& coefficients) {
    pcl::ModelCoefficients::Ptr model_coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointXYZRing> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01);
    seg.setInputCloud(input);
    seg.segment(*inliers, *model_coeffs);

    if (inliers->indices.empty()) {
      return false;
    }

    coefficients << model_coeffs->values[0], model_coeffs->values[1],
        model_coeffs->values[2], model_coeffs->values[3];

    pcl::ExtractIndices<PointXYZRing> extract;
    extract.setInputCloud(input);
    extract.setIndices(inliers);
    extract.filter(*plane_cloud);

    return true;
  }

  // Extract edge points for mechanical LiDAR using ring neighbor distance
  void extractEdgePointsMech(const PointCloudRingPtr& filtered_cloud,
                             const Eigen::Vector4f& plane_coeffs,
                             PointCloudXYZPtr& edge_cloud,
                             PointCloudXYZPtr& aligned_cloud,
                             Eigen::Matrix3d& R_align, float& average_z) {
    edge_cloud->clear();
    aligned_cloud->clear();

    // Group points by ring
    std::unordered_map<unsigned int, std::vector<int>> ring2indices;
    ring2indices.reserve(64);

    for (int i = 0; i < static_cast<int>(filtered_cloud->size()); ++i) {
      const auto& pt = filtered_cloud->points[i];
      ring2indices[pt.ring].push_back(i);
    }

    Eigen::Vector3d n(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]);
    double norm_n = n.norm();
    Eigen::Vector3d normal = n / norm_n;

    // Detect edge points by neighbor distance jump
    const double neighbor_gap_threshold = 0.10;
    const int min_points_per_ring = 10;

    for (auto& kv : ring2indices) {
      auto& idx_vec = kv.second;
      if (static_cast<int>(idx_vec.size()) < min_points_per_ring) continue;

      for (size_t k = 1; k + 1 < idx_vec.size(); ++k) {
        const auto& p_prev = filtered_cloud->points[idx_vec[k - 1]];
        const auto& p_cur = filtered_cloud->points[idx_vec[k]];
        const auto& p_next = filtered_cloud->points[idx_vec[k + 1]];

        // Only keep points near the fitted plane
        double dist_plane =
            std::fabs(plane_coeffs[0] * p_cur.x + plane_coeffs[1] * p_cur.y +
                      plane_coeffs[2] * p_cur.z + plane_coeffs[3]) /
            norm_n;
        if (dist_plane >= 0.03) continue;

        // Distance to prev
        double dx1 = p_cur.x - p_prev.x;
        double dy1 = p_cur.y - p_prev.y;
        double dz1 = p_cur.z - p_prev.z;
        double dist_prev = std::sqrt(dx1 * dx1 + dy1 * dy1 + dz1 * dz1);

        // Distance to next
        double dx2 = p_cur.x - p_next.x;
        double dy2 = p_cur.y - p_next.y;
        double dz2 = p_cur.z - p_next.z;
        double dist_next = std::sqrt(dx2 * dx2 + dy2 * dy2 + dz2 * dz2);

        if (dist_prev > neighbor_gap_threshold ||
            dist_next > neighbor_gap_threshold) {
          edge_cloud->push_back(pcl::PointXYZ(p_cur.x, p_cur.y, p_cur.z));
        }
      }
    }

    // Align edge points to Z=0 plane
    Eigen::Vector3d z_axis(0, 0, 1);
    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = acos(normal.dot(z_axis));

    Eigen::AngleAxisd rotation(angle, axis.normalized());
    R_align = rotation.toRotationMatrix();

    average_z = 0.0;
    int cnt = 0;
    aligned_cloud->reserve(edge_cloud->size());

    for (const auto& pt : *edge_cloud) {
      Eigen::Vector3d point(pt.x, pt.y, pt.z);
      Eigen::Vector3d aligned_point = R_align * point;
      aligned_cloud->push_back(
          pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
      average_z += aligned_point.z();
      cnt++;
    }
    if (cnt > 0) average_z /= cnt;
  }

  // Align plane cloud to Z=0
  void alignPlaneToZ0(const PointCloudRingPtr& plane_cloud,
                      const Eigen::Vector4f& plane_coeffs,
                      PointCloudXYZPtr& aligned_cloud, Eigen::Matrix3d& R_align,
                      float& average_z) {
    Eigen::Vector3d normal(plane_coeffs[0], plane_coeffs[1], plane_coeffs[2]);
    normal.normalize();
    Eigen::Vector3d z_axis(0, 0, 1);

    Eigen::Vector3d axis = normal.cross(z_axis);
    double angle = acos(normal.dot(z_axis));

    Eigen::AngleAxisd rotation(angle, axis.normalized());
    R_align = rotation.toRotationMatrix();

    average_z = 0.0;
    int cnt = 0;
    aligned_cloud->reserve(plane_cloud->size());

    for (const auto& pt : *plane_cloud) {
      Eigen::Vector3d point(pt.x, pt.y, pt.z);
      Eigen::Vector3d aligned_point = R_align * point;
      aligned_cloud->push_back(
          pcl::PointXYZ(aligned_point.x(), aligned_point.y(), 0.0));
      average_z += aligned_point.z();
      cnt++;
    }
    if (cnt > 0) average_z /= cnt;
  }

  // Extract edge points using boundary estimation
  void extractEdgePointsBoundary(const PointCloudXYZPtr& aligned_cloud,
                                 PointCloudXYZPtr& edge_cloud) {
    edge_cloud->clear();
    edge_cloud->reserve(aligned_cloud->size());

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
    normal_estimator.setInputCloud(aligned_cloud);
    normal_estimator.setRadiusSearch(0.03);
    normal_estimator.compute(*normals);

    pcl::PointCloud<pcl::Boundary> boundaries;
    pcl::BoundaryEstimation<pcl::PointXYZ, pcl::Normal, pcl::Boundary>
        boundary_estimator;
    boundary_estimator.setInputCloud(aligned_cloud);
    boundary_estimator.setInputNormals(normals);
    boundary_estimator.setRadiusSearch(0.03);
    boundary_estimator.setAngleThreshold(M_PI / 4);
    boundary_estimator.compute(boundaries);

    for (size_t i = 0; i < aligned_cloud->size(); ++i) {
      if (boundaries.points[i].boundary_point > 0) {
        edge_cloud->push_back(aligned_cloud->points[i]);
      }
    }
  }

  // Cluster edge points
  void clusterEdgePoints(const PointCloudXYZPtr& edge_cloud,
                         std::vector<pcl::PointIndices>& cluster_indices) {
    if (edge_cloud->empty()) return;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
        new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(edge_cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.05);
    ec.setMinClusterSize(50);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(edge_cloud);
    ec.extract(cluster_indices);
  }

  // Detect circles using RANSAC on aligned cloud
  void detectCirclesRANSAC(const PointCloudXYZPtr& aligned_cloud,
                           PointCloudXYZPtr& center_z0_cloud) {
    center_z0_cloud->clear();

    PointCloudXYZPtr xy_cloud(new PointCloudXYZ(*aligned_cloud));

    pcl::SACSegmentation<pcl::PointXYZ> circle_seg;
    circle_seg.setModelType(pcl::SACMODEL_CIRCLE2D);
    circle_seg.setMethodType(pcl::SAC_RANSAC);
    circle_seg.setDistanceThreshold(0.02);
    circle_seg.setOptimizeCoefficients(true);
    circle_seg.setMaxIterations(1000);
    circle_seg.setRadiusLimits(params_.target.circle_radius - 0.03,
                               params_.target.circle_radius + 0.03);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;

    while (xy_cloud->points.size() > 3) {
      circle_seg.setInputCloud(xy_cloud);
      circle_seg.segment(*inliers, *coefficients);

      if (inliers->indices.empty()) break;
      if (inliers->indices.size() < 5) break;

      pcl::PointXYZ center;
      center.x = coefficients->values[0];
      center.y = coefficients->values[1];
      center.z = 0;
      center_z0_cloud->push_back(center);

      // Remove inliers from cloud
      extract.setInputCloud(xy_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      PointCloudXYZPtr remaining(new PointCloudXYZ);
      extract.filter(*remaining);
      xy_cloud.swap(remaining);

      inliers->indices.clear();
    }
  }

  // Fit circles to clustered edge points
  void fitCirclesToClusters(
      const PointCloudXYZPtr& edge_cloud,
      const std::vector<pcl::PointIndices>& cluster_indices,
      const Eigen::Matrix3d& R_inv, float average_z,
      PointCloudXYZPtr& center_z0_cloud, PointCloudXYZPtr& center_cloud) {
    center_z0_cloud->clear();
    center_cloud->clear();

    for (const auto& cluster : cluster_indices) {
      PointCloudXYZPtr cluster_cloud(new PointCloudXYZ);
      for (const auto& idx : cluster.indices) {
        cluster_cloud->push_back(edge_cloud->points[idx]);
      }

      pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

      pcl::SACSegmentation<pcl::PointXYZ> seg;
      seg.setOptimizeCoefficients(true);
      seg.setModelType(pcl::SACMODEL_CIRCLE2D);
      seg.setMethodType(pcl::SAC_RANSAC);
      seg.setDistanceThreshold(0.01);
      seg.setMaxIterations(1000);
      seg.setInputCloud(cluster_cloud);
      seg.segment(*inliers, *coefficients);

      if (inliers->indices.size() > 0) {
        // Calculate fitting error
        double error = 0.0;
        for (const auto& idx : inliers->indices) {
          double dx = cluster_cloud->points[idx].x - coefficients->values[0];
          double dy = cluster_cloud->points[idx].y - coefficients->values[1];
          double distance =
              sqrt(dx * dx + dy * dy) - params_.target.circle_radius;
          error += std::abs(distance);
        }
        error /= inliers->indices.size();

        if (error < 0.025) {
          pcl::PointXYZ center_z0;
          center_z0.x = coefficients->values[0];
          center_z0.y = coefficients->values[1];
          center_z0.z = 0.0;
          center_z0_cloud->push_back(center_z0);

          // Transform back to original coordinate
          Eigen::Vector3d aligned_point(center_z0.x, center_z0.y, average_z);
          Eigen::Vector3d original_point = R_inv * aligned_point;

          pcl::PointXYZ center;
          center.x = original_point.x();
          center.y = original_point.y();
          center.z = original_point.z();
          center_cloud->push_back(center);
        }
      }
    }
  }

  // Generate combinations C(N, K)
  void generateCombinations(int N, int K,
                            std::vector<std::vector<int>>& groups) {
    groups.clear();
    if (N < K || K <= 0) return;

    std::string bitmask(K, 1);
    bitmask.resize(N, 0);

    do {
      std::vector<int> group;
      for (int i = 0; i < N; ++i) {
        if (bitmask[i]) {
          group.push_back(i);
        }
      }
      groups.push_back(group);
    } while (std::prev_permutation(bitmask.begin(), bitmask.end()));
  }

  // Select valid circle pattern and transform centers back
  bool selectAndTransformCenters(const PointCloudXYZPtr& center_z0_cloud,
                                 const Eigen::Matrix3d& R_align,
                                 float average_z,
                                 PointCloudXYZPtr& center_cloud) {
    center_cloud->clear();

    if (center_z0_cloud->size() < TARGET_NUM_CIRCLES) {
      return false;
    }

    std::vector<std::vector<int>> groups;
    generateCombinations(center_z0_cloud->size(), TARGET_NUM_CIRCLES, groups);

    std::vector<double> group_scores(groups.size(), -1.0);

    for (size_t i = 0; i < groups.size(); ++i) {
      std::vector<pcl::PointXYZ> candidates;
      for (int j : groups[i]) {
        candidates.push_back(center_z0_cloud->at(j));
      }

      SquareValidator validator(candidates, params_.target.delta_width_circles,
                                params_.target.delta_height_circles);
      group_scores[i] = validator.isValid() ? 1.0 : -1.0;
    }

    int best_idx = -1;
    double best_score = -1;
    int valid_count = 0;

    for (size_t i = 0; i < groups.size(); ++i) {
      if (group_scores[i] > 0) {
        valid_count++;
        if (group_scores[i] > best_score) {
          best_score = group_scores[i];
          best_idx = i;
        }
      }
    }

    if (valid_count > 1) {
      log(LogLevel::Error,
          "More than one set of candidates fit target's geometry.");
      return false;
    }

    if (best_idx == -1) {
      log(LogLevel::Warning,
          "Unable to find a candidate set that matches target's geometry");
      return false;
    }

    // Transform selected centers back to original coordinate
    Eigen::Matrix3d R_inv = R_align.inverse();
    for (int j : groups[best_idx]) {
      const auto& center_z0 = center_z0_cloud->at(j);

      Eigen::Vector3d aligned_point(center_z0.x, center_z0.y,
                                    center_z0.z + average_z);
      Eigen::Vector3d original_point = R_inv * aligned_point;

      pcl::PointXYZ center;
      center.x = original_point.x();
      center.y = original_point.y();
      center.z = original_point.z();
      center_cloud->push_back(center);
    }

    return true;
  }
};

}  // namespace fast_calib

#endif  // FAST_CALIB_CORE_LIDAR_DETECTOR_HPP

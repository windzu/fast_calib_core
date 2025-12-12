/*
 * FAST-Calib Core Library
 *
 * QR/ArUco marker detection for calibration target.
 * Detects ArUco markers and computes circle center positions in camera frame.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_QR_DETECTOR_HPP
#define FAST_CALIB_CORE_QR_DETECTOR_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <opencv2/aruco.hpp>
#include <vector>

#include "types.hpp"

namespace fast_calib {

/**
 * @brief Detects ArUco markers and computes circle centers in camera frame
 *
 * The calibration target has 4 ArUco markers arranged in a rectangle,
 * with circular holes near each marker.
 */
class QRDetector {
 public:
  /**
   * @brief Construct a QR detector
   * @param params Calibration parameters
   * @param logger Optional logging callback
   */
  explicit QRDetector(const CalibParams& params,
                      LogCallback logger = defaultLogger)
      : params_(params),
        logger_(logger),
        dictionary_(
            cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250)) {
    // Initialize camera matrix and distortion coefficients
    cameraMatrix_ = params_.camera.getCameraMatrix();
    distCoeffs_ = params_.camera.getDistCoeffs();
  }

  /**
   * @brief Detect QR codes and compute circle centers in camera frame
   * @param image Input image
   * @return Detection result with circle centers
   */
  QRDetectionResult detect(const cv::Mat& image) {
    QRDetectionResult result;

    if (image.empty()) {
      result.error_message = "Input image is empty";
      return result;
    }

    image.copyTo(result.annotated_image);

    // Setup board geometry
    std::vector<std::vector<cv::Point3f>> boardCorners;
    std::vector<cv::Point3f> boardCircleCenters;
    setupBoardGeometry(boardCorners, boardCircleCenters);

    std::vector<int> boardIds{1, 2, 4, 3};  // IDs order as per design
    cv::Ptr<cv::aruco::Board> board =
        cv::aruco::Board::create(boardCorners, dictionary_, boardIds);

    // Setup detector parameters
    cv::Ptr<cv::aruco::DetectorParameters> parameters =
        cv::aruco::DetectorParameters::create();

#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    parameters->doCornerRefinement = true;
#else
    parameters->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
#endif

    // Detect markers
    std::vector<int> ids;
    std::vector<std::vector<cv::Point2f>> corners;
    cv::aruco::detectMarkers(image, dictionary_, corners, ids, parameters);

    if (ids.size() > 0) {
      cv::aruco::drawDetectedMarkers(result.annotated_image, corners, ids);
    }

    log(LogLevel::Info, "Detected " + std::to_string(ids.size()) + " markers");

    // Check if we have enough markers
    if (ids.size() < static_cast<size_t>(params_.target.min_detected_markers) ||
        ids.size() > TARGET_NUM_CIRCLES) {
      result.error_message = std::to_string(ids.size()) + " marker(s) found, " +
                             std::to_string(TARGET_NUM_CIRCLES) + " expected";
      log(LogLevel::Warning, result.error_message);
      return result;
    }

    cv::Vec3d rvec(0, 0, 0), tvec(0, 0, 0);

    // Estimate poses of individual markers
    std::vector<cv::Vec3d> rvecs, tvecs;
    cv::Vec3f rvec_sin(0, 0, 0), rvec_cos(0, 0, 0);

    cv::aruco::estimatePoseSingleMarkers(corners, params_.target.marker_size,
                                         cameraMatrix_, distCoeffs_, rvecs,
                                         tvecs);

    // Draw markers' axis and accumulate pose
    for (size_t i = 0; i < ids.size(); i++) {
      cv::aruco::drawAxis(result.annotated_image, cameraMatrix_, distCoeffs_,
                          rvecs[i], tvecs[i], 0.1);

      tvec[0] += tvecs[i][0];
      tvec[1] += tvecs[i][1];
      tvec[2] += tvecs[i][2];
      rvec_sin[0] += sin(rvecs[i][0]);
      rvec_sin[1] += sin(rvecs[i][1]);
      rvec_sin[2] += sin(rvecs[i][2]);
      rvec_cos[0] += cos(rvecs[i][0]);
      rvec_cos[1] += cos(rvecs[i][1]);
      rvec_cos[2] += cos(rvecs[i][2]);
    }

    // Compute average pose
    tvec = tvec / static_cast<int>(ids.size());
    rvec_sin = rvec_sin / static_cast<int>(ids.size());
    rvec_cos = rvec_cos / static_cast<int>(ids.size());
    rvec[0] = atan2(rvec_sin[0], rvec_cos[0]);
    rvec[1] = atan2(rvec_sin[1], rvec_cos[1]);
    rvec[2] = atan2(rvec_sin[2], rvec_cos[2]);

    // Estimate board pose using all detected markers
#if (CV_MAJOR_VERSION == 3 && CV_MINOR_VERSION <= 2) || CV_MAJOR_VERSION < 3
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                             distCoeffs_, rvec, tvec);
#else
    int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cameraMatrix_,
                                             distCoeffs_, rvec, tvec, true);
#endif

    if (valid == 0) {
      result.error_message = "Failed to estimate board pose";
      return result;
    }

    cv::aruco::drawAxis(result.annotated_image, cameraMatrix_, distCoeffs_,
                        rvec, tvec, 0.2);

    // Build transformation matrix
    cv::Mat R(3, 3, CV_64F);
    cv::Rodrigues(rvec, R);

    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    t.at<double>(0) = tvec[0];
    t.at<double>(1) = tvec[1];
    t.at<double>(2) = tvec[2];

    cv::Mat board_transform = cv::Mat::eye(3, 4, CV_64F);
    R.copyTo(board_transform.rowRange(0, 3).colRange(0, 3));
    t.copyTo(board_transform.rowRange(0, 3).col(3));

    // Compute circle centers in camera frame
    PointCloudXYZPtr candidates_cloud(new PointCloudXYZ);

    for (size_t i = 0; i < boardCircleCenters.size(); ++i) {
      cv::Mat mat = cv::Mat::zeros(4, 1, CV_64F);
      mat.at<double>(0, 0) = boardCircleCenters[i].x;
      mat.at<double>(1, 0) = boardCircleCenters[i].y;
      mat.at<double>(2, 0) = boardCircleCenters[i].z;
      mat.at<double>(3, 0) = 1.0;

      cv::Mat mat_qr = board_transform * mat;
      cv::Point3f center3d;
      center3d.x = mat_qr.at<double>(0, 0);
      center3d.y = mat_qr.at<double>(1, 0);
      center3d.z = mat_qr.at<double>(2, 0);

      // Draw center on image
      cv::Point2f uv = projectPointDist(center3d, cameraMatrix_, distCoeffs_);
      cv::circle(result.annotated_image, uv, 5, cv::Scalar(0, 255, 0), -1);

      pcl::PointXYZ qr_center;
      qr_center.x = center3d.x;
      qr_center.y = center3d.y;
      qr_center.z = center3d.z;
      candidates_cloud->push_back(qr_center);
    }

    // Geometric consistency check
    if (!validateAndSelectCenters(candidates_cloud, result.centers)) {
      result.error_message = "Failed geometric consistency check";
      return result;
    }

    // Draw final centers
    for (size_t i = 0; i < result.centers->size(); i++) {
      cv::Point3f pt(result.centers->at(i).x, result.centers->at(i).y,
                     result.centers->at(i).z);
      cv::Point2f uv = projectPointDist(pt, cameraMatrix_, distCoeffs_);
      cv::circle(result.annotated_image, uv, 2, cv::Scalar(255, 0, 255), -1);
    }

    result.success = (result.centers->size() == TARGET_NUM_CIRCLES);
    return result;
  }

  /** @brief Get camera matrix */
  const cv::Mat& getCameraMatrix() const { return cameraMatrix_; }

  /** @brief Get distortion coefficients */
  const cv::Mat& getDistCoeffs() const { return distCoeffs_; }

 private:
  CalibParams params_;
  LogCallback logger_;
  cv::Ptr<cv::aruco::Dictionary> dictionary_;
  cv::Mat cameraMatrix_;
  cv::Mat distCoeffs_;

  void log(LogLevel level, const std::string& message) {
    if (logger_) {
      logger_(level, "[QRDetector] " + message);
    }
  }

  /**
   * @brief Setup board corner and circle center geometry
   *
   * Markers layout:
   *   0-------1
   *   |       |
   *   |   C   |
   *   |       |
   *   3-------2
   *
   * IDs: Marker 0 -> ID 1, Marker 1 -> ID 2, Marker 2 -> ID 4, Marker 3 -> ID 3
   */
  void setupBoardGeometry(std::vector<std::vector<cv::Point3f>>& boardCorners,
                          std::vector<cv::Point3f>& boardCircleCenters) {
    float width = params_.target.delta_width_qr_center;
    float height = params_.target.delta_height_qr_center;
    float circle_width = params_.target.delta_width_circles / 2.0f;
    float circle_height = params_.target.delta_height_circles / 2.0f;
    float marker_size = params_.target.marker_size;

    boardCorners.resize(4);
    boardCircleCenters.clear();

    for (int i = 0; i < 4; ++i) {
      int x_qr_center = (i % 3) == 0 ? -1 : 1;
      int y_qr_center = (i < 2) ? 1 : -1;
      float x_center = x_qr_center * width;
      float y_center = y_qr_center * height;

      cv::Point3f circleCenter3d(x_qr_center * circle_width,
                                 y_qr_center * circle_height, 0);
      boardCircleCenters.push_back(circleCenter3d);

      for (int j = 0; j < 4; ++j) {
        int x_qr = (j % 3) == 0 ? -1 : 1;
        int y_qr = (j < 2) ? 1 : -1;
        cv::Point3f pt3d(x_center + x_qr * marker_size / 2.0f,
                         y_center + y_qr * marker_size / 2.0f, 0);
        boardCorners[i].push_back(pt3d);
      }
    }
  }

  // Project 3D point to 2D image considering distortion
  cv::Point2f projectPointDist(const cv::Point3f& pt_cv,
                               const cv::Mat& intrinsics,
                               const cv::Mat& distCoeffs) {
    std::vector<cv::Point3f> input{pt_cv};
    std::vector<cv::Point2f> projectedPoints;
    projectedPoints.resize(1);
    cv::projectPoints(input, cv::Mat::zeros(3, 1, CV_64F),
                      cv::Mat::zeros(3, 1, CV_64F), intrinsics, distCoeffs,
                      projectedPoints);
    return projectedPoints[0];
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

  // Validate candidates and select best set
  bool validateAndSelectCenters(const PointCloudXYZPtr& candidates_cloud,
                                PointCloudXYZPtr& centers_cloud) {
    centers_cloud->clear();

    if (candidates_cloud->size() < TARGET_NUM_CIRCLES) {
      return false;
    }

    std::vector<std::vector<int>> groups;
    generateCombinations(candidates_cloud->size(), TARGET_NUM_CIRCLES, groups);

    std::vector<double> group_scores(groups.size(), -1.0);

    for (size_t i = 0; i < groups.size(); ++i) {
      std::vector<pcl::PointXYZ> candidates;
      for (int j : groups[i]) {
        candidates.push_back(candidates_cloud->at(j));
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
          "More than one set of candidates fit target's geometry");
      return false;
    }

    if (best_idx == -1) {
      log(LogLevel::Warning,
          "Unable to find a candidate set that matches target's geometry");
      return false;
    }

    for (int j : groups[best_idx]) {
      centers_cloud->push_back(candidates_cloud->at(j));
    }

    return true;
  }

  // SquareValidator class for geometry validation (reuse from lidar_detector)
  class SquareValidator {
   public:
    SquareValidator(const std::vector<pcl::PointXYZ>& candidates,
                    double delta_width, double delta_height)
        : candidates_(candidates),
          delta_width_(delta_width),
          delta_height_(delta_height) {}

    bool isValid() const {
      if (candidates_.size() != TARGET_NUM_CIRCLES) {
        return false;
      }

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

      double expected_width = delta_width_;
      double expected_height = delta_height_;
      double expected_diag = std::sqrt(delta_width_ * delta_width_ +
                                       delta_height_ * delta_height_);

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
};

}  // namespace fast_calib

#endif  // FAST_CALIB_CORE_QR_DETECTOR_HPP

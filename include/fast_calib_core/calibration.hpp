/*
 * FAST-Calib Core Library
 * 
 * Calibration computation and utility functions.
 * Includes SVD-based extrinsic estimation, point cloud projection, and result saving.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 * Developer: Chunran Zheng <zhengcr@connect.hku.hk>
 * 
 * This file is subject to the terms and conditions outlined in the 'LICENSE' file.
 */

#ifndef FAST_CALIB_CORE_CALIBRATION_HPP
#define FAST_CALIB_CORE_CALIBRATION_HPP

#include "types.hpp"
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <chrono>
#include <ctime>

namespace fast_calib {

// ============================================================================
// Point Cloud Sorting Utilities
// ============================================================================

/**
 * @brief Sort pattern centers to ensure consistent ordering
 * @param input Input point cloud
 * @param output Sorted output point cloud
 * @param sensor_type "camera" or "lidar"
 */
inline void sortPatternCenters(const PointCloudXYZPtr& input,
                               PointCloudXYZPtr& output,
                               const std::string& sensor_type) {
    output->clear();
    
    if (input->size() != TARGET_NUM_CIRCLES) {
        *output = *input;
        return;
    }

    // Copy points for sorting
    std::vector<pcl::PointXYZ> points(input->begin(), input->end());

    // Sort strategy based on sensor type
    if (sensor_type == "camera") {
        // For camera: sort by Y (top to bottom), then by X (left to right)
        std::sort(points.begin(), points.end(), 
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                if (std::abs(a.y - b.y) > 0.05) return a.y < b.y;
                return a.x < b.x;
            });
    } else {
        // For LiDAR: sort by Z (top to bottom), then by Y (left to right)
        std::sort(points.begin(), points.end(), 
            [](const pcl::PointXYZ& a, const pcl::PointXYZ& b) {
                if (std::abs(a.z - b.z) > 0.05) return a.z > b.z;
                return a.y < b.y;
            });
    }

    for (const auto& pt : points) {
        output->push_back(pt);
    }
}

// ============================================================================
// Calibration Calculator
// ============================================================================

/**
 * @brief Computes extrinsic calibration from LiDAR to camera
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
     * @brief Compute extrinsic transformation from LiDAR to camera
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
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> svd;
        svd.estimateRigidTransformation(*sorted_lidar, *sorted_qr, result.transformation);

        // Compute RMSE
        PointCloudXYZPtr aligned_lidar(new PointCloudXYZ);
        alignPointCloud(sorted_lidar, aligned_lidar, result.transformation);
        result.rmse = computeRMSE(sorted_qr, aligned_lidar);

        if (result.rmse >= 0) {
            log(LogLevel::Info, "Calibration RMSE: " + std::to_string(result.rmse) + " m");
            result.success = true;
        } else {
            result.error_message = "RMSE computation failed";
        }

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
            output->push_back(pcl::PointXYZ(transformed_pt(0), 
                                            transformed_pt(1), 
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

        objectPoints[0] = cv::Point3f(transformed_point(0), 
                                      transformed_point(1), 
                                      transformed_point(2));
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, 
                         zeroDistCoeffs, imagePoints);

        int u = static_cast<int>(imagePoints[0].x);
        int v = static_cast<int>(imagePoints[0].y);

        if (u >= 0 && u < undistortedImage.cols && 
            v >= 0 && v < undistortedImage.rows) {
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

        objectPoints[0] = cv::Point3f(transformed_point(0), 
                                      transformed_point(1), 
                                      transformed_point(2));
        cv::projectPoints(objectPoints, rvec, tvec, cameraMatrix, 
                         zeroDistCoeffs, imagePoints);

        int u = static_cast<int>(imagePoints[0].x);
        int v = static_cast<int>(imagePoints[0].y);

        if (u >= 0 && u < undistortedImage.cols && 
            v >= 0 && v < undistortedImage.rows) {
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

        std::ofstream file(output_path_ + "circle_center_record.txt", std::ios::app);
        if (!file.is_open()) {
            return false;
        }

        // Write timestamp
        auto now = std::chrono::system_clock::now();
        std::time_t now_time = std::chrono::system_clock::to_time_t(now);
        file << "time: " << std::put_time(std::localtime(&now_time), 
                                          "%Y-%m-%d %H:%M:%S") << std::endl;

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
        outFile << "# Generated by fast_calib_core v" << getVersionString() << "\n\n";
        
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
        outFile << "      " << std::setw(12) << T(1, 0) << ", " << std::setw(12) << T(1, 1) 
                << ", " << std::setw(12) << T(1, 2) << ",\n";
        outFile << "      " << std::setw(12) << T(2, 0) << ", " << std::setw(12) << T(2, 1) 
                << ", " << std::setw(12) << T(2, 2) << "]\n";

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
            pcl::io::savePCDFileBinary(output_path_ + "colored_cloud.pcd", *colored_cloud);
        }

        return true;
    }

    /**
     * @brief Save transformation matrix to file
     */
    bool saveTransformationMatrix(const Eigen::Matrix4f& T,
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

} // namespace fast_calib

#endif // FAST_CALIB_CORE_CALIBRATION_HPP

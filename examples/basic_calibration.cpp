/*
 * Basic Calibration Example
 * 
 * Demonstrates basic usage of the fast_calib_core library
 * with synthetic data.
 */

#include <fast_calib_core.hpp>
#include <iostream>
#include <iomanip>

using namespace fast_calib;

// Custom logger that prints to stdout
void customLogger(LogLevel level, const std::string& message) {
    const char* prefix;
    switch (level) {
        case LogLevel::Debug: prefix = "[DEBUG]"; break;
        case LogLevel::Info:  prefix = "[INFO] "; break;
        case LogLevel::Warning: prefix = "[WARN] "; break;
        case LogLevel::Error: prefix = "[ERROR]"; break;
        default: prefix = "[?????]";
    }
    std::cout << prefix << " " << message << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "FAST-Calib Core Library v" << getVersionString() << std::endl;
    std::cout << "==========================================" << std::endl << std::endl;

    // Create synthetic calibration data
    // LiDAR sees 4 circle centers at these positions (in LiDAR frame)
    PointCloudXYZPtr lidar_centers(new PointCloudXYZ);
    lidar_centers->push_back(pcl::PointXYZ(2.0f, -0.15f,  0.65f));  // Top-left
    lidar_centers->push_back(pcl::PointXYZ(2.0f,  0.15f,  0.65f));  // Top-right
    lidar_centers->push_back(pcl::PointXYZ(2.0f, -0.15f,  0.35f));  // Bottom-left
    lidar_centers->push_back(pcl::PointXYZ(2.0f,  0.15f,  0.35f));  // Bottom-right

    // Camera sees the same points (in camera frame, after transformation)
    // Simulate a transformation: rotation around Z + translation
    Eigen::Matrix4f T_ground_truth = Eigen::Matrix4f::Identity();
    float angle = 0.1f;  // ~5.7 degrees
    T_ground_truth(0, 0) = cos(angle);
    T_ground_truth(0, 1) = -sin(angle);
    T_ground_truth(1, 0) = sin(angle);
    T_ground_truth(1, 1) = cos(angle);
    T_ground_truth(0, 3) = 0.05f;   // 5cm in X
    T_ground_truth(1, 3) = -0.02f;  // -2cm in Y
    T_ground_truth(2, 3) = 0.1f;    // 10cm in Z

    // Transform lidar points to get camera frame points
    PointCloudXYZPtr camera_centers(new PointCloudXYZ);
    for (const auto& pt : lidar_centers->points) {
        Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
        Eigen::Vector4f transformed = T_ground_truth * p;
        camera_centers->push_back(pcl::PointXYZ(
            transformed(0), transformed(1), transformed(2)));
    }

    // Perform calibration
    std::cout << "Running calibration..." << std::endl;
    CalibrationCalculator calculator(customLogger);
    CalibrationResult result = calculator.compute(lidar_centers, camera_centers);

    if (result.success) {
        std::cout << std::endl << "Calibration successful!" << std::endl;
        std::cout << "RMSE: " << std::fixed << std::setprecision(6) 
                  << result.rmse << " m" << std::endl;

        std::cout << std::endl << "Estimated transformation (T_camera_lidar):" << std::endl;
        std::cout << std::fixed << std::setprecision(6);
        for (int i = 0; i < 4; ++i) {
            std::cout << "  [";
            for (int j = 0; j < 4; ++j) {
                std::cout << std::setw(12) << result.transformation(i, j);
                if (j < 3) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        std::cout << std::endl << "Ground truth transformation:" << std::endl;
        for (int i = 0; i < 4; ++i) {
            std::cout << "  [";
            for (int j = 0; j < 4; ++j) {
                std::cout << std::setw(12) << T_ground_truth(i, j);
                if (j < 3) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }

        // Check accuracy
        float max_error = 0.0f;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                float err = std::abs(result.transformation(i, j) - T_ground_truth(i, j));
                if (err > max_error) max_error = err;
            }
        }
        std::cout << std::endl << "Max element-wise error: " << max_error << std::endl;

    } else {
        std::cerr << "Calibration failed: " << result.error_message << std::endl;
        return 1;
    }

    return 0;
}

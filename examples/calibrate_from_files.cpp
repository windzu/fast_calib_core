/*
 * Calibration from Files Example
 * 
 * Demonstrates loading point cloud and image from files,
 * then performing calibration.
 * 
 * Usage: calibrate_from_files <pcd_file> <image_file> <camera_yaml> [options]
 */

#include <fast_calib_core.hpp>
#include <iostream>
#include <fstream>
#include <iomanip>

using namespace fast_calib;

// Simple YAML parser for camera intrinsics
bool loadCameraIntrinsics(const std::string& filename, CameraIntrinsics& cam) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Cannot open camera file: " << filename << std::endl;
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        // Parse simple key: value format
        size_t pos = line.find(':');
        if (pos == std::string::npos) continue;
        
        std::string key = line.substr(0, pos);
        std::string value = line.substr(pos + 1);
        
        // Remove whitespace
        key.erase(0, key.find_first_not_of(" \t"));
        key.erase(key.find_last_not_of(" \t") + 1);
        value.erase(0, value.find_first_not_of(" \t"));
        value.erase(value.find_last_not_of(" \t") + 1);
        
        try {
            if (key == "fx" || key == "cam_fx") cam.fx = std::stod(value);
            else if (key == "fy" || key == "cam_fy") cam.fy = std::stod(value);
            else if (key == "cx" || key == "cam_cx") cam.cx = std::stod(value);
            else if (key == "cy" || key == "cam_cy") cam.cy = std::stod(value);
            else if (key == "k1" || key == "cam_d0") cam.k1 = std::stod(value);
            else if (key == "k2" || key == "cam_d1") cam.k2 = std::stod(value);
            else if (key == "p1" || key == "cam_d2") cam.p1 = std::stod(value);
            else if (key == "p2" || key == "cam_d3") cam.p2 = std::stod(value);
        } catch (const std::exception& e) {
            // Skip invalid values
        }
    }

    return (cam.fx > 0 && cam.fy > 0);
}

/**
 * @brief Load point cloud from PCD file, auto-detect format
 * 
 * For solid-state LiDAR: loads XYZ and converts to XYZRing (ring=0)
 * For mechanical LiDAR: loads XYZRing directly (requires ring field)
 */
bool loadPointCloud(const std::string& pcd_file, LiDARType lidar_type, 
                    PointCloudRingPtr& cloud) {
    cloud->clear();
    
    if (lidar_type == LiDARType::Solid) {
        // Solid-state LiDAR: load XYZ format and convert to XYZRing
        PointCloudXYZPtr xyz_cloud(new PointCloudXYZ);
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *xyz_cloud) == -1) {
            std::cerr << "Failed to load point cloud as XYZ: " << pcd_file << std::endl;
            return false;
        }
        
        // Convert to XYZRing format (ring = 0 for solid-state)
        cloud->reserve(xyz_cloud->size());
        for (const auto& pt : xyz_cloud->points) {
            PointXYZRing pt_ring;
            pt_ring.x = pt.x;
            pt_ring.y = pt.y;
            pt_ring.z = pt.z;
            pt_ring.ring = 0;
            cloud->push_back(pt_ring);
        }
        std::cout << "Loaded XYZ point cloud (solid-state mode): " 
                  << cloud->size() << " points" << std::endl;
    } else {
        // Mechanical LiDAR: load XYZRing format directly
        if (pcl::io::loadPCDFile<PointXYZRing>(pcd_file, *cloud) == -1) {
            std::cerr << "Failed to load point cloud as XYZRing: " << pcd_file << std::endl;
            std::cerr << "Note: Mechanical LiDAR requires PCD with 'ring' field" << std::endl;
            return false;
        }
        std::cout << "Loaded XYZRing point cloud (mechanical mode): " 
                  << cloud->size() << " points" << std::endl;
    }
    
    return true;
}

void printUsage(const char* progname) {
    std::cout << "Usage: " << progname << " <pcd_file> <image_file> <camera_yaml> [options]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  pcd_file     Path to LiDAR point cloud (.pcd)" << std::endl;
    std::cout << "  image_file   Path to camera image (.png, .jpg)" << std::endl;
    std::cout << "  camera_yaml  Path to camera intrinsics file" << std::endl;
    std::cout << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --lidar-type <type>  LiDAR type: 'solid' (default) or 'mech'" << std::endl;
    std::cout << "                       solid: Solid-state LiDAR (e.g., Livox Mid360)" << std::endl;
    std::cout << "                       mech:  Mechanical LiDAR (e.g., Velodyne, Ouster)" << std::endl;
    std::cout << "  --output <dir>       Output directory for results (default: .)" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << progname << " scan.pcd image.png camera.yaml" << std::endl;
    std::cout << "  " << progname << " scan.pcd image.png camera.yaml --lidar-type solid" << std::endl;
    std::cout << "  " << progname << " scan.pcd image.png camera.yaml --lidar-type mech --output ./results" << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 4) {
        printUsage(argv[0]);
        return 1;
    }

    std::string pcd_file = argv[1];
    std::string image_file = argv[2];
    std::string camera_file = argv[3];
    std::string output_dir = ".";
    LiDARType lidar_type = LiDARType::Solid;  // Default to solid-state
    
    // Parse optional arguments
    for (int i = 4; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--lidar-type" && i + 1 < argc) {
            std::string type_str = argv[++i];
            if (type_str == "solid") {
                lidar_type = LiDARType::Solid;
            } else if (type_str == "mech" || type_str == "mechanical") {
                lidar_type = LiDARType::Mech;
            } else {
                std::cerr << "Unknown LiDAR type: " << type_str << std::endl;
                std::cerr << "Use 'solid' or 'mech'" << std::endl;
                return 1;
            }
        } else if (arg == "--output" && i + 1 < argc) {
            output_dir = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            // For backward compatibility, treat unknown arg as output_dir
            output_dir = arg;
        }
    }

    std::cout << "FAST-Calib Core Library v" << getVersionString() << std::endl;
    std::cout << "==========================================" << std::endl;
    std::cout << "LiDAR type: " << (lidar_type == LiDARType::Solid ? "Solid-state" : "Mechanical") << std::endl;
    std::cout << std::endl;

    // Load camera intrinsics
    CameraIntrinsics camera;
    if (!loadCameraIntrinsics(camera_file, camera)) {
        std::cerr << "Failed to load camera intrinsics" << std::endl;
        return 1;
    }
    std::cout << "Loaded camera intrinsics:" << std::endl;
    std::cout << "  fx=" << camera.fx << ", fy=" << camera.fy << std::endl;
    std::cout << "  cx=" << camera.cx << ", cy=" << camera.cy << std::endl;
    std::cout << "  k1=" << camera.k1 << ", k2=" << camera.k2 << std::endl << std::endl;

    // Load point cloud based on LiDAR type
    PointCloudRingPtr cloud(new PointCloudRing);
    if (!loadPointCloud(pcd_file, lidar_type, cloud)) {
        return 1;
    }

    // Load image
    cv::Mat image = cv::imread(image_file);
    if (image.empty()) {
        std::cerr << "Failed to load image: " << image_file << std::endl;
        return 1;
    }
    std::cout << "Loaded image: " << image.cols << "x" << image.rows << std::endl << std::endl;

    // Setup calibration parameters
    CalibParams params;
    params.camera = camera;
    params.target.marker_size = 0.05;   // 5cm ArUco markers
    params.target.delta_width_qr_center = 0.55;
    params.target.delta_height_qr_center = 0.35;
    params.target.delta_width_circles = 0.50;
    params.target.delta_height_circles = 0.40;
    params.target.circle_radius = 0.10; // 10cm circle radius
    
    // Filter params (adjust based on target distance)
    params.lidar_filter.x_min = 0.5;
    params.lidar_filter.x_max = 5.0;
    params.lidar_filter.y_min = -2.0;
    params.lidar_filter.y_max = 2.0;
    params.lidar_filter.z_min = -1.0;
    params.lidar_filter.z_max = 2.0;

    // Detect circles in LiDAR
    std::cout << "Detecting circles in LiDAR point cloud..." << std::endl;
    LiDARDetector lidar_detector(params);
    LiDARDetectionResult lidar_result = lidar_detector.detect(cloud, lidar_type);
    
    if (!lidar_result.success) {
        std::cerr << "LiDAR detection failed: " << lidar_result.error_message << std::endl;
        return 1;
    }
    std::cout << "  Found " << lidar_result.centers->size() << " circle centers" << std::endl;

    // Detect markers and compute circle centers in camera
    std::cout << "Detecting ArUco markers in image..." << std::endl;
    QRDetector qr_detector(params);
    QRDetectionResult qr_result = qr_detector.detect(image);
    
    if (!qr_result.success) {
        std::cerr << "QR detection failed: " << qr_result.error_message << std::endl;
        return 1;
    }
    std::cout << "  Found " << qr_result.centers->size() << " circle centers" << std::endl;

    // Compute calibration
    std::cout << std::endl << "Computing extrinsic calibration..." << std::endl;
    CalibrationCalculator calculator;
    CalibrationResult calib_result = calculator.compute(
        lidar_result.centers, qr_result.centers);

    if (!calib_result.success) {
        std::cerr << "Calibration failed: " << calib_result.error_message << std::endl;
        return 1;
    }

    // Print results
    std::cout << std::endl << "Calibration successful!" << std::endl;
    std::cout << "RMSE: " << std::fixed << std::setprecision(6) 
              << calib_result.rmse << " m" << std::endl;

    std::cout << std::endl << "Transformation matrix (T_camera_lidar):" << std::endl;
    std::cout << std::fixed << std::setprecision(6);
    for (int i = 0; i < 4; ++i) {
        std::cout << "  [";
        for (int j = 0; j < 4; ++j) {
            std::cout << std::setw(12) << calib_result.transformation(i, j);
            if (j < 3) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }

    // Project point cloud to image for visualization
    std::cout << std::endl << "Creating colored point cloud..." << std::endl;
    PointCloudXYZRGBPtr colored_cloud(new PointCloudXYZRGB);
    projectPointCloudToImage(
        cloud,
        calib_result.transformation,
        camera.getCameraMatrix(),
        camera.getDistCoeffs(),
        image,
        colored_cloud);
    std::cout << "  Colored " << colored_cloud->size() << " points" << std::endl;

    // Save results
    std::cout << std::endl << "Saving results to " << output_dir << "/" << std::endl;
    ResultSaver saver(output_dir);
    
    if (saver.saveCalibrationResults(params, calib_result, colored_cloud, qr_result.annotated_image)) {
        std::cout << "  Results saved successfully" << std::endl;
    } else {
        std::cerr << "  Failed to save some results" << std::endl;
    }

    saver.saveTransformationMatrix(calib_result.transformation);
    
    std::cout << std::endl << "Done!" << std::endl;
    return 0;
}

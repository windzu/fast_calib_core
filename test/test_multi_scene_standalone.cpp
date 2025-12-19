/*
 * Standalone test for multi-scene calibration
 * Compile: g++ -std=c++17 -I../include $(pkg-config --cflags eigen3 opencv4) \
 *          -I/usr/include/pcl-1.12 test_multi_scene_standalone.cpp \
 *          -L/usr/lib/x86_64-linux-gnu -lpcl_common -lopencv_core -o
 * test_multi_scene
 */

#include <cmath>
#include <fast_calib_core.hpp>
#include <iostream>
#include <random>

using namespace fast_calib;

// Create a transformation matrix with rotation and translation
Eigen::Matrix4f createTransformation(float rx, float ry, float rz, float tx,
                                     float ty, float tz) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  Eigen::Matrix3f Rx, Ry, Rz;
  Rx << 1, 0, 0, 0, cos(rx), -sin(rx), 0, sin(rx), cos(rx);
  Ry << cos(ry), 0, sin(ry), 0, 1, 0, -sin(ry), 0, cos(ry);
  Rz << cos(rz), -sin(rz), 0, sin(rz), cos(rz), 0, 0, 0, 1;
  T.block<3, 3>(0, 0) = Rz * Ry * Rx;
  T(0, 3) = tx;
  T(1, 3) = ty;
  T(2, 3) = tz;
  return T;
}

// Transform point cloud
PointCloudXYZPtr transformCloud(const PointCloudXYZPtr& input,
                                const Eigen::Matrix4f& T) {
  PointCloudXYZPtr output(new PointCloudXYZ);
  for (const auto& pt : input->points) {
    Eigen::Vector4f p(pt.x, pt.y, pt.z, 1.0f);
    Eigen::Vector4f transformed = T * p;
    output->push_back(
        pcl::PointXYZ(transformed(0), transformed(1), transformed(2)));
  }
  return output;
}

int main() {
  std::cout << "=== Multi-Scene Calibration Test ===" << std::endl;

  // Create known transformation (what we want to recover)
  Eigen::Matrix4f known_T =
      createTransformation(0.1f, 0.05f, 0.15f,  // Small rotations (radians)
                           0.5f, -0.3f, 3.5f    // Translation
      );

  std::cout << "\nKnown transformation T_camera_lidar:\n"
            << known_T << std::endl;

  // Create multiple scenes with different board positions
  // Using proper square pattern (4 corners at same plane, LiDAR coordinate)
  // Pattern: board is ~0.3m wide, ~0.3m tall, oriented facing the LiDAR
  std::vector<SceneData> scenes;

  // Scene 1: Board at position 1 (x=2m distance)
  // LiDAR frame: x=forward, y=left, z=up
  PointCloudXYZPtr scene1_lidar(new PointCloudXYZ);
  scene1_lidar->push_back(pcl::PointXYZ(2.0f, 0.15f, 0.15f));    // Top-left
  scene1_lidar->push_back(pcl::PointXYZ(2.0f, -0.15f, 0.15f));   // Top-right
  scene1_lidar->push_back(pcl::PointXYZ(2.0f, 0.15f, -0.15f));   // Bottom-left
  scene1_lidar->push_back(pcl::PointXYZ(2.0f, -0.15f, -0.15f));  // Bottom-right
  scenes.emplace_back(scene1_lidar, transformCloud(scene1_lidar, known_T));

  // Scene 2: Board at position 2 (x=2.5m, slightly offset)
  PointCloudXYZPtr scene2_lidar(new PointCloudXYZ);
  scene2_lidar->push_back(pcl::PointXYZ(2.5f, 0.35f, 0.15f));   // Top-left
  scene2_lidar->push_back(pcl::PointXYZ(2.5f, 0.05f, 0.15f));   // Top-right
  scene2_lidar->push_back(pcl::PointXYZ(2.5f, 0.35f, -0.15f));  // Bottom-left
  scene2_lidar->push_back(pcl::PointXYZ(2.5f, 0.05f, -0.15f));  // Bottom-right
  scenes.emplace_back(scene2_lidar, transformCloud(scene2_lidar, known_T));

  // Scene 3: Board at position 3 (x=1.8m, different offset)
  PointCloudXYZPtr scene3_lidar(new PointCloudXYZ);
  scene3_lidar->push_back(pcl::PointXYZ(1.8f, -0.05f, 0.25f));   // Top-left
  scene3_lidar->push_back(pcl::PointXYZ(1.8f, -0.35f, 0.25f));   // Top-right
  scene3_lidar->push_back(pcl::PointXYZ(1.8f, -0.05f, -0.05f));  // Bottom-left
  scene3_lidar->push_back(pcl::PointXYZ(1.8f, -0.35f, -0.05f));  // Bottom-right
  scenes.emplace_back(scene3_lidar, transformCloud(scene3_lidar, known_T));

  std::cout << "\nCreated " << scenes.size() << " scenes with "
            << scenes.size() * 4 << " total point correspondences" << std::endl;

  // Test 1: Multi-scene calibration (perfect data)
  std::cout << "\n--- Test 1: Multi-scene calibration (perfect data) ---"
            << std::endl;
  CalibrationCalculator calc;
  // Use sort_points=false since our test data is already correctly paired
  auto result = calc.computeMultiScene(scenes, nullptr, false);

  if (result.success) {
    std::cout << "SUCCESS!" << std::endl;
    std::cout << "RMSE: " << result.rmse << " m" << std::endl;
    std::cout << "\nRecovered transformation:\n"
              << result.transformation << std::endl;

    // Compare with known transform
    float max_error = 0.0f;
    for (int i = 0; i < 4; ++i) {
      for (int j = 0; j < 4; ++j) {
        float error = std::abs(result.transformation(i, j) - known_T(i, j));
        max_error = std::max(max_error, error);
      }
    }
    std::cout << "Max element error: " << max_error << std::endl;

    if (result.rmse < 0.001 && max_error < 0.01) {
      std::cout << "✓ Test 1 PASSED" << std::endl;
    } else {
      std::cout << "✗ Test 1 FAILED" << std::endl;
    }
  } else {
    std::cout << "FAILED: " << result.error_message << std::endl;
    std::cout << "✗ Test 1 FAILED" << std::endl;
  }

  // Test 2: Multi-scene with noise
  std::cout << "\n--- Test 2: Multi-scene calibration (with 1mm noise) ---"
            << std::endl;

  std::mt19937 gen(42);
  std::normal_distribution<float> noise(0.0f, 0.001f);  // 1mm noise

  std::vector<SceneData> noisy_scenes;
  for (const auto& scene : scenes) {
    PointCloudXYZPtr noisy_cam(new PointCloudXYZ);
    for (const auto& pt : scene.qr_centers->points) {
      noisy_cam->push_back(pcl::PointXYZ(pt.x + noise(gen), pt.y + noise(gen),
                                         pt.z + noise(gen)));
    }
    noisy_scenes.emplace_back(scene.lidar_centers, noisy_cam);
  }

  auto noisy_result = calc.computeMultiScene(noisy_scenes, nullptr, false);

  if (noisy_result.success) {
    std::cout << "SUCCESS!" << std::endl;
    std::cout << "RMSE: " << noisy_result.rmse << " m" << std::endl;

    float max_error = 0.0f;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        float error =
            std::abs(noisy_result.transformation(i, j) - known_T(i, j));
        max_error = std::max(max_error, error);
      }
    }
    std::cout << "Max rotation error: " << max_error << std::endl;

    if (noisy_result.rmse < 0.01 && max_error < 0.05) {
      std::cout << "✓ Test 2 PASSED" << std::endl;
    } else {
      std::cout << "✗ Test 2 FAILED" << std::endl;
    }
  } else {
    std::cout << "FAILED: " << noisy_result.error_message << std::endl;
    std::cout << "✗ Test 2 FAILED" << std::endl;
  }

  // Test 3: Weighted multi-scene
  std::cout << "\n--- Test 3: Weighted multi-scene calibration ---"
            << std::endl;
  std::vector<double> weights = {2.0, 1.0, 1.0};  // Higher weight for scene 1

  auto weighted_result = calc.computeMultiScene(scenes, &weights, false);

  if (weighted_result.success && weighted_result.rmse < 0.001) {
    std::cout << "SUCCESS!" << std::endl;
    std::cout << "RMSE: " << weighted_result.rmse << " m" << std::endl;
    std::cout << "✓ Test 3 PASSED" << std::endl;
  } else {
    std::cout << "FAILED" << std::endl;
    std::cout << "✗ Test 3 FAILED" << std::endl;
  }

  // Test 4: Error handling - single scene
  std::cout << "\n--- Test 4: Error handling (single scene) ---" << std::endl;
  std::vector<SceneData> single_scene = {scenes[0]};
  auto single_result = calc.computeMultiScene(single_scene);

  if (!single_result.success && !single_result.error_message.empty()) {
    std::cout << "Correctly rejected single scene: "
              << single_result.error_message << std::endl;
    std::cout << "✓ Test 4 PASSED" << std::endl;
  } else {
    std::cout << "✗ Test 4 FAILED" << std::endl;
  }

  // Test 5: Compare with single-scene compute()
  std::cout << "\n--- Test 5: Single-scene compute() ---" << std::endl;
  auto single_scene_result =
      calc.compute(scene1_lidar, transformCloud(scene1_lidar, known_T));

  if (single_scene_result.success && single_scene_result.rmse < 0.001) {
    std::cout << "SUCCESS!" << std::endl;
    std::cout << "RMSE: " << single_scene_result.rmse << " m" << std::endl;
    std::cout << "✓ Test 5 PASSED" << std::endl;
  } else {
    std::cout << "FAILED" << std::endl;
    std::cout << "✗ Test 5 FAILED" << std::endl;
  }

  std::cout << "\n=== All Tests Completed ===" << std::endl;
  return 0;
}

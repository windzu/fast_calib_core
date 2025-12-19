/*
 * Unit tests for fast_calib_core calibration module
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fast_calib_core.hpp>
#include <random>

namespace fast_calib {
namespace test {

// ============================================================================
// Test Utilities
// ============================================================================

/**
 * @brief Generate test point cloud with 4 points
 */
inline PointCloudXYZPtr createTestCloud(
    const std::vector<std::array<float, 3>>& points) {
  PointCloudXYZPtr cloud(new PointCloudXYZ);
  for (const auto& pt : points) {
    cloud->push_back(pcl::PointXYZ(pt[0], pt[1], pt[2]));
  }
  return cloud;
}

/**
 * @brief Apply transformation to point cloud
 */
inline PointCloudXYZPtr transformCloud(const PointCloudXYZPtr& input,
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

/**
 * @brief Create a known transformation matrix
 */
inline Eigen::Matrix4f createTransformation(float rx, float ry, float rz,
                                            float tx, float ty, float tz) {
  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

  // Rotation matrices
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

// ============================================================================
// CalibrationCalculator Single-Scene Tests
// ============================================================================

class CalibrationCalculatorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create known correspondence between points
    // Source points (LiDAR frame)
    source_cloud.reset(new PointCloudXYZ);
    source_cloud->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));  // Top-left
    source_cloud->push_back(pcl::PointXYZ(0.3, 0.0, 1.0));  // Top-right
    source_cloud->push_back(pcl::PointXYZ(0.0, 0.0, 0.7));  // Bottom-left
    source_cloud->push_back(pcl::PointXYZ(0.3, 0.0, 0.7));  // Bottom-right

    // Target points (Camera frame) - same points, identity transform
    target_cloud.reset(new PointCloudXYZ);
    *target_cloud = *source_cloud;
  }

  PointCloudXYZPtr source_cloud;
  PointCloudXYZPtr target_cloud;
};

TEST_F(CalibrationCalculatorTest, IdentityTransformation) {
  CalibrationCalculator calc;
  auto result = calc.compute(source_cloud, target_cloud);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001);

  // Check approximate identity
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      float expected = (i == j) ? 1.0f : 0.0f;
      EXPECT_NEAR(result.transformation(i, j), expected, 0.001);
    }
  }
}

TEST_F(CalibrationCalculatorTest, TranslationOnly) {
  // Apply translation to target
  Eigen::Vector3f translation(0.5, 0.3, 0.1);
  for (auto& pt : target_cloud->points) {
    pt.x += translation(0);
    pt.y += translation(1);
    pt.z += translation(2);
  }

  CalibrationCalculator calc;
  auto result = calc.compute(source_cloud, target_cloud);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001);

  // Check translation
  EXPECT_NEAR(result.transformation(0, 3), translation(0), 0.001);
  EXPECT_NEAR(result.transformation(1, 3), translation(1), 0.001);
  EXPECT_NEAR(result.transformation(2, 3), translation(2), 0.001);
}

TEST_F(CalibrationCalculatorTest, RotationOnly) {
  // Apply 90-degree rotation around Z axis
  float angle = M_PI / 2;
  Eigen::Matrix3f R;
  R << cos(angle), -sin(angle), 0, sin(angle), cos(angle), 0, 0, 0, 1;

  target_cloud->clear();
  for (const auto& pt : source_cloud->points) {
    Eigen::Vector3f p(pt.x, pt.y, pt.z);
    Eigen::Vector3f rotated = R * p;
    target_cloud->push_back(pcl::PointXYZ(rotated(0), rotated(1), rotated(2)));
  }

  CalibrationCalculator calc;
  auto result = calc.compute(source_cloud, target_cloud);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.01);
}

TEST_F(CalibrationCalculatorTest, IncorrectPointCount) {
  // Remove one point from source
  PointCloudXYZPtr incorrect_source(new PointCloudXYZ);
  incorrect_source->push_back(source_cloud->points[0]);
  incorrect_source->push_back(source_cloud->points[1]);
  incorrect_source->push_back(source_cloud->points[2]);
  // Missing 4th point

  CalibrationCalculator calc;
  auto result = calc.compute(incorrect_source, target_cloud);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

// ============================================================================
// CalibrationCalculator Multi-Scene Tests
// ============================================================================

class MultiSceneCalibrationTest : public ::testing::Test {
 protected:
  void SetUp() override {
    // Create a known transformation to recover
    known_transform =
        createTransformation(0.1f, 0.05f, 0.15f,  // Small rotations (rad)
                             0.5f, -0.3f, 3.5f    // Translation
        );

    // Create multiple scenes with different LiDAR point positions
    // Scene 1: Board at position 1
    scene1_lidar = createTestCloud({
        {{0.0f, 0.0f, 1.0f}},
        {{0.3f, 0.0f, 1.0f}},
        {{0.0f, 0.0f, 0.7f}},
        {{0.3f, 0.0f, 0.7f}},
    });

    // Scene 2: Board at position 2 (rotated and translated)
    scene2_lidar = createTestCloud({
        {{0.5f, 0.2f, 1.2f}},
        {{0.8f, 0.2f, 1.2f}},
        {{0.5f, 0.2f, 0.9f}},
        {{0.8f, 0.2f, 0.9f}},
    });

    // Scene 3: Board at position 3
    scene3_lidar = createTestCloud({
        {{-0.3f, 0.1f, 0.8f}},
        {{0.0f, 0.1f, 0.8f}},
        {{-0.3f, 0.1f, 0.5f}},
        {{0.0f, 0.1f, 0.5f}},
    });

    // Transform LiDAR points to camera frame using known transform
    scene1_camera = transformCloud(scene1_lidar, known_transform);
    scene2_camera = transformCloud(scene2_lidar, known_transform);
    scene3_camera = transformCloud(scene3_lidar, known_transform);
  }

  Eigen::Matrix4f known_transform;
  PointCloudXYZPtr scene1_lidar, scene1_camera;
  PointCloudXYZPtr scene2_lidar, scene2_camera;
  PointCloudXYZPtr scene3_lidar, scene3_camera;
};

TEST_F(MultiSceneCalibrationTest, TwoScenes_PerfectData) {
  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, scene1_camera);
  scenes.emplace_back(scene2_lidar, scene2_camera);

  CalibrationCalculator calc;
  // Use sort_points=false since test data is already correctly paired
  auto result = calc.computeMultiScene(scenes, nullptr, false);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001) << "RMSE too high: " << result.rmse;

  // Check transformation is recovered
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(result.transformation(i, j), known_transform(i, j), 0.01)
          << "Mismatch at (" << i << "," << j << ")";
    }
  }
}

TEST_F(MultiSceneCalibrationTest, ThreeScenes_PerfectData) {
  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, scene1_camera);
  scenes.emplace_back(scene2_lidar, scene2_camera);
  scenes.emplace_back(scene3_lidar, scene3_camera);

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes, nullptr, false);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001) << "RMSE too high: " << result.rmse;

  // Check transformation is recovered
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(result.transformation(i, j), known_transform(i, j), 0.01)
          << "Mismatch at (" << i << "," << j << ")";
    }
  }
}

TEST_F(MultiSceneCalibrationTest, WeightedMultiScene) {
  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, scene1_camera);
  scenes.emplace_back(scene2_lidar, scene2_camera);
  scenes.emplace_back(scene3_lidar, scene3_camera);

  // Give higher weight to scene 1
  std::vector<double> weights = {2.0, 1.0, 1.0};

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes, &weights, false);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001) << "RMSE too high: " << result.rmse;
}

TEST_F(MultiSceneCalibrationTest, WithNoise_StillAccurate) {
  // Add small noise to camera points
  std::random_device rd;
  std::mt19937 gen(42);  // Fixed seed for reproducibility
  std::normal_distribution<float> noise(0.0f, 0.001f);  // 1mm noise

  auto addNoise = [&](PointCloudXYZPtr cloud) {
    for (auto& pt : cloud->points) {
      pt.x += noise(gen);
      pt.y += noise(gen);
      pt.z += noise(gen);
    }
  };

  PointCloudXYZPtr noisy_cam1(new PointCloudXYZ(*scene1_camera));
  PointCloudXYZPtr noisy_cam2(new PointCloudXYZ(*scene2_camera));
  PointCloudXYZPtr noisy_cam3(new PointCloudXYZ(*scene3_camera));

  addNoise(noisy_cam1);
  addNoise(noisy_cam2);
  addNoise(noisy_cam3);

  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, noisy_cam1);
  scenes.emplace_back(scene2_lidar, noisy_cam2);
  scenes.emplace_back(scene3_lidar, noisy_cam3);

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes, nullptr, false);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.01) << "RMSE too high with noise: " << result.rmse;

  // Should still recover approximate transformation
  for (int i = 0; i < 3; ++i) {
    for (int j = 0; j < 3; ++j) {
      EXPECT_NEAR(result.transformation(i, j), known_transform(i, j), 0.05)
          << "Rotation mismatch at (" << i << "," << j << ")";
    }
  }
}

TEST_F(MultiSceneCalibrationTest, SingleScene_Error) {
  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, scene1_camera);

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(MultiSceneCalibrationTest, EmptyScenes_Error) {
  std::vector<SceneData> scenes;

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

TEST_F(MultiSceneCalibrationTest, InvalidScene_Error) {
  // Create invalid scene with wrong number of points
  PointCloudXYZPtr invalid_lidar(new PointCloudXYZ);
  invalid_lidar->push_back(pcl::PointXYZ(0, 0, 0));  // Only 1 point

  std::vector<SceneData> scenes;
  scenes.emplace_back(scene1_lidar, scene1_camera);
  scenes.emplace_back(invalid_lidar, scene2_camera);

  CalibrationCalculator calc;
  auto result = calc.computeMultiScene(scenes);

  EXPECT_FALSE(result.success);
  EXPECT_FALSE(result.error_message.empty());
}

// ============================================================================
// solveRigidTransformWeighted Tests
// ============================================================================

TEST(WeightedSVDTest, UniformWeights_MatchesPCL) {
  // Create random points
  std::vector<Eigen::Vector3d> lidar_pts = {
      {0.0, 0.0, 1.0}, {0.3, 0.0, 1.0}, {0.0, 0.0, 0.7}, {0.3, 0.0, 0.7}};

  // Known transformation
  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitZ());
  Eigen::Vector3d t(0.5, 0.3, 3.0);

  // Transform points
  std::vector<Eigen::Vector3d> camera_pts;
  for (const auto& pt : lidar_pts) {
    camera_pts.push_back(R * pt + t);
  }

  // Uniform weights
  std::vector<double> weights(4, 1.0);

  auto result = CalibrationCalculator::solveRigidTransformWeighted(
      lidar_pts, camera_pts, weights);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 1e-10);
}

TEST(WeightedSVDTest, DifferentWeights) {
  std::vector<Eigen::Vector3d> lidar_pts = {
      {0.0, 0.0, 1.0}, {0.3, 0.0, 1.0}, {0.0, 0.0, 0.7}, {0.3, 0.0, 0.7}};

  Eigen::Matrix3d R;
  R = Eigen::AngleAxisd(0.05, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxisd(0.1, Eigen::Vector3d::UnitY());
  Eigen::Vector3d t(0.2, -0.1, 2.5);

  std::vector<Eigen::Vector3d> camera_pts;
  for (const auto& pt : lidar_pts) {
    camera_pts.push_back(R * pt + t);
  }

  // Non-uniform weights
  std::vector<double> weights = {1.0, 2.0, 0.5, 1.5};

  auto result = CalibrationCalculator::solveRigidTransformWeighted(
      lidar_pts, camera_pts, weights);

  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 1e-10);
}

TEST(WeightedSVDTest, TooFewPoints_Error) {
  std::vector<Eigen::Vector3d> lidar_pts = {{0.0, 0.0, 1.0}, {0.3, 0.0, 1.0}};
  std::vector<Eigen::Vector3d> camera_pts = {{0.0, 0.0, 1.0}, {0.3, 0.0, 1.0}};
  std::vector<double> weights = {1.0, 1.0};

  auto result = CalibrationCalculator::solveRigidTransformWeighted(
      lidar_pts, camera_pts, weights);

  EXPECT_FALSE(result.success);
}

// ============================================================================
// SceneData Tests
// ============================================================================

TEST(SceneDataTest, DefaultConstruction) {
  SceneData scene;
  EXPECT_NE(scene.lidar_centers, nullptr);
  EXPECT_NE(scene.qr_centers, nullptr);
  EXPECT_FALSE(scene.isValid());  // Empty clouds are not valid
}

TEST(SceneDataTest, ValidScene) {
  PointCloudXYZPtr lidar(new PointCloudXYZ);
  PointCloudXYZPtr camera(new PointCloudXYZ);

  for (int i = 0; i < 4; ++i) {
    lidar->push_back(pcl::PointXYZ(i, 0, 0));
    camera->push_back(pcl::PointXYZ(i, 0, 0));
  }

  SceneData scene(lidar, camera);
  EXPECT_TRUE(scene.isValid());
}

TEST(SceneDataTest, InvalidScene_WrongCount) {
  PointCloudXYZPtr lidar(new PointCloudXYZ);
  PointCloudXYZPtr camera(new PointCloudXYZ);

  for (int i = 0; i < 3; ++i) {  // Only 3 points
    lidar->push_back(pcl::PointXYZ(i, 0, 0));
    camera->push_back(pcl::PointXYZ(i, 0, 0));
  }

  SceneData scene(lidar, camera);
  EXPECT_FALSE(scene.isValid());
}

// ============================================================================
// RMSE Computation Tests
// ============================================================================

TEST(CalibrationUtilsTest, ComputeRMSE_IdenticalClouds) {
  PointCloudXYZPtr cloud1(new PointCloudXYZ);
  cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud1->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));
  cloud1->push_back(pcl::PointXYZ(0.0, 1.0, 0.0));
  cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));

  PointCloudXYZPtr cloud2(new PointCloudXYZ);
  *cloud2 = *cloud1;

  double rmse = CalibrationCalculator::computeRMSE(cloud1, cloud2);
  EXPECT_DOUBLE_EQ(rmse, 0.0);
}

TEST(CalibrationUtilsTest, ComputeRMSE_KnownOffset) {
  PointCloudXYZPtr cloud1(new PointCloudXYZ);
  cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));

  PointCloudXYZPtr cloud2(new PointCloudXYZ);
  cloud2->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

  double rmse = CalibrationCalculator::computeRMSE(cloud1, cloud2);
  EXPECT_DOUBLE_EQ(rmse, 1.0);
}

TEST(CalibrationUtilsTest, ComputeRMSE_DifferentSizes) {
  PointCloudXYZPtr cloud1(new PointCloudXYZ);
  cloud1->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  cloud1->push_back(pcl::PointXYZ(1.0, 0.0, 0.0));

  PointCloudXYZPtr cloud2(new PointCloudXYZ);
  cloud2->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));

  double rmse = CalibrationCalculator::computeRMSE(cloud1, cloud2);
  EXPECT_DOUBLE_EQ(rmse, -1.0);  // Error condition
}

// ============================================================================
// Point Cloud Alignment Tests
// ============================================================================

TEST(CalibrationUtilsTest, AlignPointCloud_Identity) {
  PointCloudXYZPtr input(new PointCloudXYZ);
  input->push_back(pcl::PointXYZ(1.0, 2.0, 3.0));

  Eigen::Matrix4f identity = Eigen::Matrix4f::Identity();

  PointCloudXYZPtr output(new PointCloudXYZ);
  CalibrationCalculator::alignPointCloud(input, output, identity);

  ASSERT_EQ(output->size(), 1);
  EXPECT_FLOAT_EQ(output->points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(output->points[0].y, 2.0f);
  EXPECT_FLOAT_EQ(output->points[0].z, 3.0f);
}

TEST(CalibrationUtilsTest, AlignPointCloud_Translation) {
  PointCloudXYZPtr input(new PointCloudXYZ);
  input->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));

  Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
  T(0, 3) = 1.0;
  T(1, 3) = 2.0;
  T(2, 3) = 3.0;

  PointCloudXYZPtr output(new PointCloudXYZ);
  CalibrationCalculator::alignPointCloud(input, output, T);

  ASSERT_EQ(output->size(), 1);
  EXPECT_FLOAT_EQ(output->points[0].x, 1.0f);
  EXPECT_FLOAT_EQ(output->points[0].y, 2.0f);
  EXPECT_FLOAT_EQ(output->points[0].z, 3.0f);
}

// ============================================================================
// Sorting Tests
// ============================================================================

TEST(CalibrationUtilsTest, SortPatternCenters_Camera) {
  PointCloudXYZPtr input(new PointCloudXYZ);
  // Add points in random order
  input->push_back(pcl::PointXYZ(0.3, 0.2, 1.0));  // Bottom-right
  input->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));  // Top-left
  input->push_back(pcl::PointXYZ(0.3, 0.0, 1.0));  // Top-right
  input->push_back(pcl::PointXYZ(0.0, 0.2, 1.0));  // Bottom-left

  PointCloudXYZPtr output(new PointCloudXYZ);
  sortPatternCenters(input, output, "camera");

  ASSERT_EQ(output->size(), 4);
  // After sorting: top row (small y) first, then left to right (small x)
  EXPECT_NEAR(output->points[0].y, 0.0, 0.01);  // Top row
  EXPECT_NEAR(output->points[1].y, 0.0, 0.01);  // Top row
}

TEST(CalibrationUtilsTest, SortPatternCenters_LiDAR) {
  PointCloudXYZPtr input(new PointCloudXYZ);
  // Add points in random order
  input->push_back(pcl::PointXYZ(0.0, 0.3, 0.7));  // Bottom-right
  input->push_back(pcl::PointXYZ(0.0, 0.0, 1.0));  // Top-left
  input->push_back(pcl::PointXYZ(0.0, 0.3, 1.0));  // Top-right
  input->push_back(pcl::PointXYZ(0.0, 0.0, 0.7));  // Bottom-left

  PointCloudXYZPtr output(new PointCloudXYZ);
  sortPatternCenters(input, output, "lidar");

  ASSERT_EQ(output->size(), 4);
  // After sorting: top (large z) first
  EXPECT_NEAR(output->points[0].z, 1.0, 0.01);  // Top row
  EXPECT_NEAR(output->points[1].z, 1.0, 0.01);  // Top row
}

}  // namespace test
}  // namespace fast_calib

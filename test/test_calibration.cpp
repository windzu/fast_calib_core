/*
 * Unit tests for fast_calib_core calibration module
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fast_calib_core.hpp>

namespace fast_calib {
namespace test {

// ============================================================================
// CalibrationCalculator Tests
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

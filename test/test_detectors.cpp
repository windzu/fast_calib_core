/*
 * Unit tests for fast_calib_core detector modules
 */

#include <gtest/gtest.h>

#include <cmath>
#include <fast_calib_core.hpp>

namespace fast_calib {
namespace test {

// ============================================================================
// LiDARDetector Tests
// ============================================================================

class LiDARDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    params.target.circle_radius = 0.1;
    params.target.delta_width_circles = 0.5;
    params.target.delta_height_circles = 0.4;
    params.lidar_filter.x_min = 0.0;
    params.lidar_filter.x_max = 5.0;
    params.lidar_filter.y_min = -2.0;
    params.lidar_filter.y_max = 2.0;
    params.lidar_filter.z_min = -1.0;
    params.lidar_filter.z_max = 2.0;
  }

  CalibParams params;
};

TEST_F(LiDARDetectorTest, EmptyCloud) {
  PointCloudRingPtr empty_cloud(new PointCloudRing);

  LiDARDetector detector(params);
  auto result = detector.detect(empty_cloud, LiDARType::Solid);

  EXPECT_FALSE(result.success);
  EXPECT_TRUE(result.centers->empty());
}

TEST_F(LiDARDetectorTest, CloudWithInsufficientPoints) {
  PointCloudRingPtr sparse_cloud(new PointCloudRing);

  // Add just a few random points
  for (int i = 0; i < 3; ++i) {
    PointXYZRing pt;
    pt.x = 1.0f + i * 0.1f;
    pt.y = 0.0f;
    pt.z = 0.5f;
    pt.ring = 0;
    sparse_cloud->push_back(pt);
  }

  LiDARDetector detector(params);
  auto result = detector.detect(sparse_cloud, LiDARType::Solid);

  EXPECT_FALSE(result.success);
}

// ============================================================================
// QRDetector Tests
// ============================================================================

class QRDetectorTest : public ::testing::Test {
 protected:
  void SetUp() override {
    params.camera.fx = 500.0;
    params.camera.fy = 500.0;
    params.camera.cx = 320.0;
    params.camera.cy = 240.0;
    params.camera.k1 = 0.0;
    params.camera.k2 = 0.0;
    params.camera.p1 = 0.0;
    params.camera.p2 = 0.0;

    params.target.marker_size = 0.05;
    params.target.circle_radius = 0.1;
  }

  CalibParams params;
};

TEST_F(QRDetectorTest, EmptyImage) {
  cv::Mat empty_image;

  QRDetector detector(params);
  auto result = detector.detect(empty_image);

  EXPECT_FALSE(result.success);
}

TEST_F(QRDetectorTest, BlackImage) {
  cv::Mat black_image = cv::Mat::zeros(480, 640, CV_8UC3);

  QRDetector detector(params);
  auto result = detector.detect(black_image);

  EXPECT_FALSE(result.success);  // No markers detected
}

TEST_F(QRDetectorTest, ConstructorSetsIntrinsics) {
  QRDetector detector(params);
  // Just verify construction doesn't throw
  SUCCEED();
}

// ============================================================================
// Integration Tests
// ============================================================================

TEST(IntegrationTest, FullPipeline_SyntheticData) {
  // This test verifies the full calibration pipeline with synthetic data

  // Create known point correspondences
  PointCloudXYZPtr lidar_centers(new PointCloudXYZ);
  PointCloudXYZPtr camera_centers(new PointCloudXYZ);

  // Define a known transformation
  Eigen::Matrix4f known_T = Eigen::Matrix4f::Identity();
  known_T(0, 3) = 0.1f;   // 10cm translation in X
  known_T(1, 3) = 0.05f;  // 5cm translation in Y

  // Create points in LiDAR frame (2x2 grid at 1m distance)
  std::vector<Eigen::Vector3f> lidar_pts = {
      {1.0f, -0.15f, 0.15f},   // Top-left
      {1.0f, 0.15f, 0.15f},    // Top-right
      {1.0f, -0.15f, -0.15f},  // Bottom-left
      {1.0f, 0.15f, -0.15f}    // Bottom-right
  };

  for (const auto& pt : lidar_pts) {
    lidar_centers->push_back(pcl::PointXYZ(pt(0), pt(1), pt(2)));

    // Transform to camera frame
    Eigen::Vector4f homogeneous(pt(0), pt(1), pt(2), 1.0f);
    Eigen::Vector4f transformed = known_T * homogeneous;
    camera_centers->push_back(
        pcl::PointXYZ(transformed(0), transformed(1), transformed(2)));
  }

  // Run calibration
  CalibrationCalculator calc;
  auto result = calc.compute(lidar_centers, camera_centers);

  // Verify results
  EXPECT_TRUE(result.success);
  EXPECT_LT(result.rmse, 0.001);  // Sub-millimeter accuracy expected

  // Check transformation matrix matches
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EXPECT_NEAR(result.transformation(i, j), known_T(i, j), 0.01)
          << "Mismatch at (" << i << ", " << j << ")";
    }
  }
}

}  // namespace test
}  // namespace fast_calib

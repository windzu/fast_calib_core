/*
 * Unit tests for fast_calib_core types
 */

#include <gtest/gtest.h>

#include <fast_calib_core.hpp>

namespace fast_calib {
namespace test {

// ============================================================================
// Version Tests
// ============================================================================

TEST(TypesTest, VersionString) {
  std::string version = getVersionString();
  EXPECT_FALSE(version.empty());
  EXPECT_NE(version.find('.'), std::string::npos);
}

// ============================================================================
// CameraIntrinsics Tests
// ============================================================================

TEST(TypesTest, CameraIntrinsicsDefaultConstruction) {
  CameraIntrinsics cam;
  // Uses default values from the struct
  EXPECT_GT(cam.fx, 0.0);
  EXPECT_GT(cam.fy, 0.0);
  EXPECT_GT(cam.cx, 0.0);
  EXPECT_GT(cam.cy, 0.0);
}

TEST(TypesTest, CameraIntrinsicsGetCameraMatrix) {
  CameraIntrinsics cam;
  cam.fx = 500.0;
  cam.fy = 500.0;
  cam.cx = 320.0;
  cam.cy = 240.0;

  cv::Mat K = cam.getCameraMatrix();

  EXPECT_EQ(K.rows, 3);
  EXPECT_EQ(K.cols, 3);
  EXPECT_DOUBLE_EQ(K.at<double>(0, 0), 500.0);
  EXPECT_DOUBLE_EQ(K.at<double>(1, 1), 500.0);
  EXPECT_DOUBLE_EQ(K.at<double>(0, 2), 320.0);
  EXPECT_DOUBLE_EQ(K.at<double>(1, 2), 240.0);
  EXPECT_DOUBLE_EQ(K.at<double>(2, 2), 1.0);
}

TEST(TypesTest, CameraIntrinsicsGetDistCoeffs) {
  CameraIntrinsics cam;
  cam.k1 = 0.1;
  cam.k2 = -0.2;
  cam.p1 = 0.001;
  cam.p2 = -0.002;

  cv::Mat D = cam.getDistCoeffs();

  EXPECT_EQ(D.rows, 4);
  EXPECT_EQ(D.cols, 1);
  EXPECT_DOUBLE_EQ(D.at<double>(0), 0.1);
  EXPECT_DOUBLE_EQ(D.at<double>(1), -0.2);
  EXPECT_DOUBLE_EQ(D.at<double>(2), 0.001);
  EXPECT_DOUBLE_EQ(D.at<double>(3), -0.002);
}

// ============================================================================
// TargetParams Tests
// ============================================================================

TEST(TypesTest, TargetParamsDefaultConstruction) {
  TargetParams target;
  EXPECT_GT(target.marker_size, 0.0);
  EXPECT_GT(target.circle_radius, 0.0);
}

// ============================================================================
// CalibrationResult Tests
// ============================================================================

TEST(TypesTest, CalibrationResultDefaultConstruction) {
  CalibrationResult result;
  EXPECT_FALSE(result.success);
  EXPECT_LT(result.rmse, 0.0);
  EXPECT_TRUE(result.error_message.empty());

  // Check identity matrix
  EXPECT_FLOAT_EQ(result.transformation(0, 0), 1.0f);
  EXPECT_FLOAT_EQ(result.transformation(1, 1), 1.0f);
  EXPECT_FLOAT_EQ(result.transformation(2, 2), 1.0f);
  EXPECT_FLOAT_EQ(result.transformation(3, 3), 1.0f);
}

TEST(TypesTest, CalibrationResultGetRotationMatrix) {
  CalibrationResult result;
  result.transformation << 0.0, -1.0, 0.0, 1.0, 1.0, 0.0, 0.0, 2.0, 0.0, 0.0,
      1.0, 3.0, 0.0, 0.0, 0.0, 1.0;

  Eigen::Matrix3f R = result.getRotation();

  EXPECT_FLOAT_EQ(R(0, 0), 0.0f);
  EXPECT_FLOAT_EQ(R(0, 1), -1.0f);
  EXPECT_FLOAT_EQ(R(1, 0), 1.0f);
  EXPECT_FLOAT_EQ(R(1, 1), 0.0f);
  EXPECT_FLOAT_EQ(R(2, 2), 1.0f);
}

TEST(TypesTest, CalibrationResultGetTranslationVector) {
  CalibrationResult result;
  result.transformation << 1.0, 0.0, 0.0, 1.5, 0.0, 1.0, 0.0, 2.5, 0.0, 0.0,
      1.0, 3.5, 0.0, 0.0, 0.0, 1.0;

  Eigen::Vector3f t = result.getTranslation();

  EXPECT_FLOAT_EQ(t(0), 1.5f);
  EXPECT_FLOAT_EQ(t(1), 2.5f);
  EXPECT_FLOAT_EQ(t(2), 3.5f);
}

// ============================================================================
// LiDARFilterParams Tests
// ============================================================================

TEST(TypesTest, LiDARFilterParamsDefaultConstruction) {
  LiDARFilterParams params;
  EXPECT_GT(params.x_max, params.x_min);
  EXPECT_GT(params.y_max, params.y_min);
  EXPECT_GT(params.z_max, params.z_min);
}

// ============================================================================
// Default Logger Tests
// ============================================================================

TEST(TypesTest, DefaultLoggerDoesNotThrow) {
  EXPECT_NO_THROW(defaultLogger(LogLevel::Debug, "Debug message"));
  EXPECT_NO_THROW(defaultLogger(LogLevel::Info, "Info message"));
  EXPECT_NO_THROW(defaultLogger(LogLevel::Warning, "Warn message"));
  EXPECT_NO_THROW(defaultLogger(LogLevel::Error, "Error message"));
}

// ============================================================================
// PointXYZRing Tests
// ============================================================================

TEST(TypesTest, PointXYZRingConstruction) {
  PointXYZRing pt;
  pt.x = 1.0f;
  pt.y = 2.0f;
  pt.z = 3.0f;
  pt.ring = 5;

  EXPECT_FLOAT_EQ(pt.x, 1.0f);
  EXPECT_FLOAT_EQ(pt.y, 2.0f);
  EXPECT_FLOAT_EQ(pt.z, 3.0f);
  EXPECT_EQ(pt.ring, 5);
}

}  // namespace test
}  // namespace fast_calib

# Changelog

All notable changes to `fast_calib_core` will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [0.2.0] - 2024-12-15

### Added

- **Multi-scene joint optimization**: New `computeMultiScene()` method for calibrating from multiple observations
  - Supports weighted SVD optimization across all point correspondences
  - `SceneData` struct for organizing multi-scene data
  - Optional per-scene weights for prioritizing certain observations
  - `sort_points` parameter to control point sorting behavior
- **Weighted SVD solver**: New `solveRigidTransformWeighted()` static method
  - Implements weighted Procrustes/Wahba algorithm
  - Handles reflection cases (det(R) < 0)
  - Returns weighted RMSE
- Comprehensive unit tests for multi-scene calibration
- Standalone test program for multi-scene validation
- Updated README with multi-scene API documentation

### Changed

- `CalibrationCalculator` now supports both single-scene (`compute()`) and multi-scene (`computeMultiScene()`) calibration

### Reference

This update incorporates multi-scene calibration functionality based on the original FAST-Calib implementation:

- **Upstream project**: [FAST-Calib](https://github.com/hku-mars/FAST-Calib)
- **Reference commit**: `e51e1aa1ee5f79bd072c25568a617dfeb442ba17`
- **Reference file**: `src/multi_scene.cpp` (`SolveRigidTransformWeighted` function)

## [0.1.0] - 2024-XX-XX

### Added
- Initial release as standalone header-only library
- Core types and data structures (`types.hpp`)
- LiDAR circle detection for mechanical and solid-state LiDARs (`lidar_detector.hpp`)
- ArUco marker-based QR detection (`qr_detector.hpp`)
- SVD-based calibration computation (`calibration.hpp`)
- Point cloud projection and colorization utilities
- Result saving functionality
- CMake build system with find_package support
- Unit tests using Google Test
- Example programs

### Source Tracking

This library is derived from the original [FAST-Calib](https://github.com/hku-mars/FAST-Calib) project.

#### Upstream Commits Incorporated

The following commits from the upstream FAST-Calib repository have been incorporated into this library:

| Date | Upstream Commit | Description | Files Affected |
|------|-----------------|-------------|----------------|
| YYYY-MM-DD | `abc1234` | Initial port from FAST-Calib | All core files |

#### How to Track Upstream Changes

When syncing fixes from the original FAST-Calib project:

1. Check the upstream repository for new commits:
   ```bash
   git clone https://github.com/hku-mars/FAST-Calib.git
   cd FAST-Calib
   git log --oneline --since="LAST_SYNC_DATE"
   ```

2. For each relevant commit, add an entry to this table with:
   - The commit date
   - The upstream commit hash (short form)
   - A brief description of the change
   - Which files in `fast_calib_core` were affected

3. Update the library code to incorporate the fix

4. Update the version number following semver:
   - Bug fixes: patch version (0.1.x)
   - New features: minor version (0.x.0)
   - Breaking changes: major version (x.0.0)

#### Mapping: Upstream Files → Library Files

| Upstream (FAST-Calib) | This Library |
|-----------------------|--------------|
| `include/utility.h` (types) | `include/fast_calib_core/types.hpp` |
| `include/utility.h` (LiDAR detection) | `include/fast_calib_core/lidar_detector.hpp` |
| `include/utility.h` (QR detection) | `include/fast_calib_core/qr_detector.hpp` |
| `include/utility.h` (calibration) | `include/fast_calib_core/calibration.hpp` |
| `src/main.cpp` | N/A (ROS node, not ported) |
| `src/calib_result.cpp` | `include/fast_calib_core/calibration.hpp` (ResultSaver) |

---

## Notes for Maintainers

### Porting Guidelines

When porting fixes from upstream:

1. **Remove ROS dependencies**: Replace `ros::` calls with dependency injection (LogCallback)
2. **Use standard C++ types**: Avoid ROS-specific types like `sensor_msgs::PointCloud2`
3. **Keep algorithm logic intact**: The core math should remain unchanged
4. **Update tests**: Add or modify tests to cover the changed code
5. **Document the change**: Add entry to changelog with upstream commit reference

### Version Compatibility

- **PCL**: Tested with PCL 1.10+
- **OpenCV**: Tested with OpenCV 4.x (requires aruco module)
- **Eigen**: Tested with Eigen 3.3+
- **C++**: Requires C++17 or later

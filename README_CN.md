# fast_calib_core

[English](README.md) | [中文](README_CN.md)

一个与 ROS 无关的、仅头文件的 C++ 库，用于相机-激光雷达外参标定。

## 概述

`fast_calib_core` 提供了 [FAST-Calib](https://github.com/hku-mars/FAST-Calib) 项目的核心算法，与 ROS 依赖解耦。这使得标定功能可以在任何 C++ 项目中使用，包括 ROS1、ROS2 或独立应用程序。

## 特性

- **仅头文件库**：无需编译，只需包含头文件
- **与 ROS 无关**：可与任何 C++ 项目配合使用
- **单场景标定**：从单次观测快速标定
- **多场景联合优化**：基于加权 SVD 的多场景标定，提高精度
- **支持多种激光雷达类型**：支持机械式（Velodyne 风格）和固态激光雷达（Livox）
- **基于 ArUco 的标定板**：使用 ArUco 标记进行鲁棒检测
- **基于 SVD 的标定**：使用奇异值分解进行鲁棒的变换估计
- **可视化工具**：将点云投影到图像上并着色

## 依赖项

- [PCL](https://pointclouds.org/) >= 1.10（点云库）
- [OpenCV](https://opencv.org/) >= 4.0（含 ArUco 模块）
- [Eigen3](https://eigen.tuxfamily.org/)
- 支持 C++17 的编译器

## 安装

### 作为 Git 子模块

```bash
# 在你的项目目录中
git submodule add https://github.com/windzu/fast_calib_core.git external/fast_calib_core
```

然后在你的 `CMakeLists.txt` 中：

```cmake
add_subdirectory(external/fast_calib_core)
target_link_libraries(your_target PRIVATE fast_calib_core::fast_calib_core)
```

### 独立编译

```bash
mkdir build && cd build
cmake .. -DFAST_CALIB_CORE_BUILD_EXAMPLES=ON
make -j$(nproc)
```

### 系统安装

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
sudo make install
```

然后使用 `find_package`：

```cmake
find_package(fast_calib_core REQUIRED)
target_link_libraries(your_target PRIVATE fast_calib_core::fast_calib_core)
```

## 教程：相机-激光雷达标定

本教程演示如何使用 `calibrate_from_files` 示例程序进行相机-激光雷达外参标定。

### 测试数据

从 Google Drive 下载示例数据集：

- **[示例数据集 (Mid360 + 相机)](https://drive.google.com/file/d/15-ABjE6SKx4eZdFdzsdIFKC2Y_tXT-jv/view?usp=sharing)**

数据集包含：

- `11.pcd` - Livox Mid360 点云（3,634,009 个点）
- `11.png` - 相机图像（1440x1080）
- `intrinsics.yaml` - 相机内参

### 编译示例

```bash
cd fast_calib_core
mkdir build && cd build
cmake .. -DFAST_CALIB_CORE_BUILD_EXAMPLES=ON
make -j$(nproc)
```

### 运行标定

```bash
# 创建输出目录
mkdir -p output

# 运行标定（Mid360 使用固态激光雷达模式）
./examples/calibrate_from_files \
    ../data/mid360/11.pcd \
    ../data/mid360/11.png \
    ../data/mid360/intrinsics.yaml \
    --lidar-type solid \
    --output ./output
```

### 命令行选项

```
用法: calibrate_from_files <pcd_file> <image_file> <camera_yaml> [选项]

参数:
  pcd_file     激光雷达点云路径 (.pcd)
  image_file   相机图像路径 (.png, .jpg)
  camera_yaml  相机内参文件路径

选项:
  --lidar-type <type>  激光雷达类型: 'solid'（默认）或 'mech'
                       solid: 固态激光雷达（如 Livox Mid360）
                       mech:  机械式激光雷达（如 Velodyne, Ouster）
  --output <dir>       结果输出目录（默认: 当前目录）
```

### 预期输出

```
FAST-Calib Core Library v0.1.0
==========================================
LiDAR type: Solid-state

Loaded camera intrinsics:
  fx=522.124, fy=522.275
  cx=773.466, cy=534.053
  k1=0.0032495, k2=-0.0171041

Loaded XYZ point cloud (solid-state mode): 3634009 points
Loaded image: 1440x1080

Detecting circles in LiDAR point cloud...
[INFO]  [LiDARDetector] Starting solid-state LiDAR detection...
[INFO]  [LiDARDetector] Filtered cloud size: 1313027
[INFO]  [LiDARDetector] After voxel filter: 477382
[INFO]  [LiDARDetector] Plane cloud size: 121449
[INFO]  [LiDARDetector] Extracted 7271 edge points.
[INFO]  [LiDARDetector] Number of edge clusters: 6
  Found 4 circle centers
Detecting ArUco markers in image...
[INFO]  [QRDetector] Detected 4 markers
  Found 4 circle centers

Computing extrinsic calibration...
[INFO]  [Calibration] Calibration RMSE: 0.002005 m

Calibration successful!
RMSE: 0.002005 m

Transformation matrix (T_camera_lidar):
  [    0.046103,     0.997654,    -0.050602,    -0.222727]
  [   -0.009401,    -0.050220,    -0.998694,    -0.027279]
  [   -0.998893,     0.046519,     0.007064,     3.757941]
  [    0.000000,     0.000000,     0.000000,     1.000000]

Creating colored point cloud...
  Colored 1710778 points

Saving results to ./output/
  Results saved successfully

Done!
```

### 输出文件

| 文件 | 描述 |
|------|------|
| `single_calib_result.txt` | FAST-LIVO2 格式的标定结果 |
| `transformation.txt` | 4x4 变换矩阵 |
| `annotated_image.png` | 带有 ArUco 检测可视化的图像 |
| `colored_cloud.pcd` | 使用相机纹理着色的点云 |

### 结果文件格式

`single_calib_result.txt` 遵循 FAST-LIVO2 格式：

```yaml
# FAST-Calib result
cam_model: Pinhole
cam_width: 1440
cam_height: 1080
scale: 1.0
cam_fx: 522.124
cam_fy: 522.275
cam_cx: 773.466
cam_cy: 534.053
cam_d0: 0.0032495    # k1（径向畸变）
cam_d1: -0.0171041   # k2（径向畸变）
cam_d2: 0.000669657  # p1（切向畸变）
cam_d3: -0.000350205 # p2（切向畸变）

Rcl: [ 0.046103273,  0.997654080, -0.050601516,
      -0.009400862, -0.050219826, -0.998693764,
      -0.998892546,  0.046518769,  0.007063511]
Pcl: [-0.222726792, -0.027279139,  3.757940531]

RMSE: 0.002004615 m
```

### 可视化结果

**输入图像：**

![输入图像](docs/images/input_image.png)

**ArUco 检测结果：**

![标注图像](docs/images/annotated_image.png)

### 相机内参格式

相机内参 YAML 文件应包含：

```yaml
# 相机内参
image_width: 1440
image_height: 1080

# 焦距和主点
fx: 522.124   # 或 cam_fx
fy: 522.275   # 或 cam_fy
cx: 773.466   # 或 cam_cx
cy: 534.053   # 或 cam_cy

# 畸变系数（OpenCV 模型）
k1: 0.0032495    # 或 cam_d0
k2: -0.0171041   # 或 cam_d1
p1: 0.000669657  # 或 cam_d2
p2: -0.000350205 # 或 cam_d3
```

### 畸变参数映射

| FAST-Calib | OpenCV | 描述 |
|------------|--------|------|
| cam_d0 | k1 | 第一径向畸变系数 |
| cam_d1 | k2 | 第二径向畸变系数 |
| cam_d2 | p1 | 第一切向畸变系数 |
| cam_d3 | p2 | 第二切向畸变系数 |

---

## 配置文件接口

对于喜欢简单接口的用户，`calibrate` 工具接受单个 YAML 配置文件，指定所有标定参数和场景。

### 单场景标定

创建 `config.yaml` 文件：

```yaml
# 相机内参
camera:
  image_width: 1440
  image_height: 1080
  distortion_coefficients:
    k1: 0.00324949759262203
    k2: -0.0171040538369167
    p1: 0.000669657443377146
    p2: -0.000350205468789575
  intrinsics:
    fx: 522.123514287681
    fy: 522.275153384482
    cx: 773.466430504725
    cy: 534.053165700174

# 激光雷达配置
lidar:
  type: "solid"  # "solid" 用于 Livox，"mech" 用于机械式激光雷达

# 标定板参数
target:
  marker_size: 0.20
  delta_width_qr_center: 0.55
  delta_height_qr_center: 0.35
  circle_radius: 0.12

# 单场景（相对于配置文件位置的相对路径）
scenes:
  - name: "scene_01"
    image: "11.png"
    pointcloud: "11.pcd"
    filter:
      min: [1.0, -3.0, -2.0]
      max: [5.0, 3.0, 3.0]

# 输出配置
output:
  path: "output"
```

运行标定：

```bash
./build/examples/calibrate data/mid360/config.yaml
```

### 多场景联合标定

为了提高精度，配置多个场景：

```yaml
scenes:
  - name: "scene_11"
    image: "11.png"
    pointcloud: "11.pcd"
    filter:
      min: [1.0, -3.0, -2.0]
      max: [5.0, 3.0, 3.0]

  - name: "scene_22"
    image: "22.png"
    pointcloud: "22.pcd"
    filter:
      min: [1.0, -3.0, -2.0]
      max: [5.0, 3.0, 3.0]

  - name: "scene_33"
    image: "33.png"
    pointcloud: "33.pcd"
    filter:
      min: [1.0, -3.0, -2.0]
      max: [5.0, 3.0, 3.0]
```

工具会自动检测多场景配置并使用所有场景进行联合优化。

### 配置参考

| 节 | 字段 | 描述 |
|---|------|------|
| `camera` | `intrinsics` | fx, fy, cx, cy 焦距和主点 |
| `camera` | `distortion_coefficients` | k1, k2, p1, p2 畸变系数 |
| `lidar` | `type` | "solid"（Livox）或 "mech"（Velodyne/Ouster） |
| `target` | `marker_size` | ArUco 标记边长（米） |
| `target` | `delta_width_qr_center` | QR 中心之间的水平距离 |
| `target` | `delta_height_qr_center` | QR 中心之间的垂直距离 |
| `target` | `circle_radius` | 圆孔半径 |
| `scenes[].filter.min` | [x,y,z] | ROI 最小边界 |
| `scenes[].filter.max` | [x,y,z] | ROI 最大边界 |
| `output` | `path` | 输出目录（相对或绝对路径） |

---

## 快速开始（编程 API）

```cpp
#include <fast_calib_core.hpp>

using namespace fast_calib;

int main() {
    // 设置相机内参
    CameraIntrinsics camera;
    camera.fx = 500.0;
    camera.fy = 500.0;
    camera.cx = 320.0;
    camera.cy = 240.0;

    // 设置标定参数
    CalibParams params;
    params.camera = camera;
    params.target.marker_edge_size = 0.05;   // 5cm ArUco 标记
    params.target.marker_distance = 0.30;    // 标记中心间距 30cm
    params.target.circle_diameter = 0.20;    // 圆直径 20cm
    params.lidar_type = LiDARType::SolidState;

    // 加载数据
    PointCloudRingPtr cloud = loadPointCloud("scan.pcd");
    cv::Mat image = cv::imread("image.png");

    // 在激光雷达点云中检测圆
    LiDARDetector lidar_detector;
    auto lidar_result = lidar_detector.detect(cloud, params);

    // 在图像中检测标记并计算圆心
    QRDetector qr_detector(camera);
    auto qr_result = qr_detector.detect(image, params.target);

    // 计算标定
    CalibrationCalculator calculator;
    auto result = calculator.compute(lidar_result.centers, qr_result.centers);

    if (result.success) {
        std::cout << "标定 RMSE: " << result.rmse << " m" << std::endl;
        // 使用 result.transformation（4x4 矩阵）
    }

    return 0;
}
```

## API 参考

### 核心类型

- `CameraIntrinsics`：相机内参（fx, fy, cx, cy, 畸变系数）
- `TargetParams`：标定板参数（标记大小、距离）
- `CalibParams`：完整标定参数
- `CalibrationResult`：包含变换矩阵和 RMSE 的结果
- `LiDARType`：激光雷达类型枚举（Mechanical, SolidState）
- `PointXYZRing`：带有环信息的自定义 PCL 点类型

### 检测器

#### LiDARDetector

```cpp
LiDARDetector detector(logger);  // 可选的自定义日志器
LiDARDetectionResult result = detector.detect(cloud, params);
```

#### QRDetector

```cpp
QRDetector detector(camera_intrinsics);
QRDetectionResult result = detector.detect(image, target_params);
```

### 标定

#### CalibrationCalculator

**单场景标定：**

```cpp
CalibrationCalculator calc(logger);  // 可选的自定义日志器
CalibrationResult result = calc.compute(lidar_centers, camera_centers);
```

**多场景联合优化：**

```cpp
#include <fast_calib_core.hpp>

using namespace fast_calib;

// 从多次观测收集场景数据
std::vector<SceneData> scenes;
scenes.emplace_back(lidar_centers_1, camera_centers_1);
scenes.emplace_back(lidar_centers_2, camera_centers_2);
scenes.emplace_back(lidar_centers_3, camera_centers_3);

// 执行联合优化
CalibrationCalculator calc;
CalibrationResult result = calc.computeMultiScene(scenes);

// 使用自定义权重（可选）
std::vector<double> weights = {2.0, 1.0, 1.0};  // 场景 1 权重更高
result = calc.computeMultiScene(scenes, &weights);

// 使用预排序的点对应关系（跳过内部排序）
result = calc.computeMultiScene(scenes, nullptr, false);

if (result.success) {
    std::cout << "多场景 RMSE: " << result.rmse << " m" << std::endl;
    // 使用 result.transformation（4x4 矩阵）
}
```

**SceneData 结构体：**

```cpp
struct SceneData {
  PointCloudXYZPtr lidar_centers;  // 激光雷达坐标系中的圆心（4 个点）
  PointCloudXYZPtr qr_centers;     // 相机坐标系中的圆心（4 个点）
  
  bool isValid() const;  // 如果两者都恰好有 4 个点则返回 true
};
```

### 工具函数

#### 点云投影

```cpp
PointCloudXYZRGBPtr colored;
projectPointCloudToImage(cloud, transformation, K, D, image, colored);
```

#### 结果保存

```cpp
ResultSaver saver("/output/path");
saver.saveCalibrationResults(params, result, colored_cloud, annotated_image);
```

## 自定义日志

该库使用依赖注入进行日志记录。提供你自己的回调函数：

```cpp
void myLogger(fast_calib::LogLevel level, const std::string& msg) {
    // 你的日志实现
    std::cout << msg << std::endl;
}

LiDARDetector detector(myLogger);
```

## 标定板

该库期望的标定板应具有：

- 4 个排列成正方形的 ArUco 标记
- 4 个以标记位置为中心的圆形镂空
- 默认 ArUco 字典：DICT_6X6_250

可调参数：

- `marker_edge_size`：ArUco 标记的大小（默认：0.05m）
- `marker_distance`：标记中心之间的距离（默认：0.30m）
- `circle_diameter`：圆形镂空的直径（默认：0.20m）

## ROS 集成

本库设计为由 ROS 特定包进行封装：

- **ROS1**：参见 [FAST-Calib](https://github.com/hku-mars/FAST-Calib)
- **ROS2**：参见 [FAST-Calib-ROS2](https://github.com/windzu/FAST-Calib-ROS2)

## 与上游同步

本库跟踪上游 FAST-Calib 项目。参见 [CHANGELOG.md](CHANGELOG.md) 了解：

- 已合并的上游提交
- 项目间的文件映射
- 移植修复的指南

## 许可证

本项目遵循与原始 FAST-Calib 项目相同的许可证。
详情请参见 [LICENSE](LICENSE)。

## 引用

如果你在学术工作中使用本库，请引用原始 FAST-Calib 论文：

```bibtex
@article{zheng2024fast,
  title={FAST-Calib: Fast and Accurate Extrinsic Calibration for LiDAR-Camera Systems},
  author={Zheng, Chunran and others},
  journal={...},
  year={2024}
}
```

## 致谢

- HKU MARS Lab 的 FAST-Calib 原作者
- PCL、OpenCV 和 Eigen 社区

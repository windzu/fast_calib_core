/*
 * FAST-Calib Core Library
 *
 * Configuration file parser for calibration.
 * Supports YAML-like configuration files for single and multi-scene
 * calibration.
 *
 * Original project: https://github.com/hku-mars/FAST-Calib
 *
 * This file is subject to the terms and conditions outlined in the 'LICENSE'
 * file.
 */

#ifndef FAST_CALIB_CORE_CONFIG_PARSER_HPP
#define FAST_CALIB_CORE_CONFIG_PARSER_HPP

#include <algorithm>
#include <array>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "types.hpp"

namespace fast_calib {

// ============================================================================
// Configuration Structures
// ============================================================================

/**
 * @brief Point cloud filter configuration
 */
struct FilterConfig {
  bool enabled = false;
  std::array<float, 3> min = {-100.0f, -100.0f, -100.0f};
  std::array<float, 3> max = {100.0f, 100.0f, 100.0f};
};

/**
 * @brief Scene configuration for calibration
 */
struct SceneConfig {
  std::string name;
  std::string image_path;
  std::string pointcloud_path;
  FilterConfig filter;
};

/**
 * @brief Output configuration
 */
struct OutputConfig {
  std::string path = "./output";
  std::string parent_frame = "lidar";
  std::string child_frame = "camera";
};

/**
 * @brief Debug configuration
 */
struct DebugConfig {
  bool enabled = false;
  bool save_raw_data = false;
  bool save_intermediate_clouds = false;
  bool save_annotated_image = false;
  bool save_colored_cloud = true;
};

/**
 * @brief Complete calibration configuration
 */
struct CalibConfig {
  CameraIntrinsics camera;
  LiDARType lidar_type = LiDARType::Solid;
  TargetParams target;
  std::vector<SceneConfig> scenes;
  OutputConfig output;
  DebugConfig debug;

  // Config file directory (for resolving relative paths)
  std::string config_dir;

  /**
   * @brief Check if this is a multi-scene configuration
   */
  bool isMultiScene() const { return scenes.size() > 1; }

  /**
   * @brief Resolve relative path to absolute path
   */
  std::string resolvePath(const std::string& path) const {
    if (path.empty()) return path;
    if (path[0] == '/') return path;  // Already absolute
    return config_dir + "/" + path;
  }
};

// ============================================================================
// Simple YAML-like Parser
// ============================================================================

/**
 * @brief Configuration file parser
 *
 * Parses YAML-like configuration files for calibration.
 * Note: This is a simple parser, not a full YAML parser.
 */
class ConfigParser {
 public:
  /**
   * @brief Parse configuration file
   * @param filename Path to configuration file
   * @param config Output configuration
   * @return true if parsing succeeded
   */
  static bool parse(const std::string& filename, CalibConfig& config) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      std::cerr << "Cannot open config file: " << filename << std::endl;
      return false;
    }

    // Get config directory for relative path resolution
    size_t last_slash = filename.find_last_of("/\\");
    config.config_dir = (last_slash != std::string::npos)
                            ? filename.substr(0, last_slash)
                            : ".";

    std::string line;
    std::string current_section;
    std::string current_subsection;
    int current_scene_idx = -1;
    bool in_filter = false;

    while (std::getline(file, line)) {
      // Remove comments
      size_t comment_pos = line.find('#');
      if (comment_pos != std::string::npos) {
        line = line.substr(0, comment_pos);
      }

      // Skip empty lines
      if (line.find_first_not_of(" \t") == std::string::npos) continue;

      // Count leading spaces for indentation
      size_t indent = line.find_first_not_of(" ");
      std::string trimmed = trim(line);

      // Check for list item
      bool is_list_item = trimmed[0] == '-';
      if (is_list_item) {
        trimmed = trim(trimmed.substr(1));
      }

      // Parse key: value
      size_t colon_pos = trimmed.find(':');
      if (colon_pos == std::string::npos) continue;

      std::string key = trim(trimmed.substr(0, colon_pos));
      std::string value = trim(trimmed.substr(colon_pos + 1));

      // Handle sections based on indentation
      if (indent == 0) {
        current_section = key;
        current_subsection.clear();
        in_filter = false;
        continue;
      }

      // Handle scenes list
      if (current_section == "scenes") {
        if (is_list_item && key == "name") {
          // New scene
          SceneConfig scene;
          scene.name = removeQuotes(value);
          config.scenes.push_back(scene);
          current_scene_idx = static_cast<int>(config.scenes.size()) - 1;
          in_filter = false;
        } else if (current_scene_idx >= 0) {
          if (key == "image") {
            config.scenes[current_scene_idx].image_path = removeQuotes(value);
          } else if (key == "pointcloud") {
            config.scenes[current_scene_idx].pointcloud_path =
                removeQuotes(value);
          } else if (key == "filter") {
            in_filter = true;
            config.scenes[current_scene_idx].filter.enabled = true;
          } else if (in_filter && key == "min") {
            parseArray3(value, config.scenes[current_scene_idx].filter.min);
          } else if (in_filter && key == "max") {
            parseArray3(value, config.scenes[current_scene_idx].filter.max);
          }
        }
        continue;
      }

      // Handle camera section
      if (current_section == "camera") {
        if (key == "intrinsics" || key == "distortion_coefficients") {
          current_subsection = key;
        } else if (current_subsection == "intrinsics") {
          if (key == "fx")
            config.camera.fx = std::stod(value);
          else if (key == "fy")
            config.camera.fy = std::stod(value);
          else if (key == "cx")
            config.camera.cx = std::stod(value);
          else if (key == "cy")
            config.camera.cy = std::stod(value);
        } else if (current_subsection == "distortion_coefficients") {
          if (key == "k1")
            config.camera.k1 = std::stod(value);
          else if (key == "k2")
            config.camera.k2 = std::stod(value);
          else if (key == "p1")
            config.camera.p1 = std::stod(value);
          else if (key == "p2")
            config.camera.p2 = std::stod(value);
        }
        continue;
      }

      // Handle lidar section
      if (current_section == "lidar") {
        if (key == "type") {
          std::string type_str = removeQuotes(value);
          config.lidar_type = (type_str == "mech" || type_str == "mechanical")
                                  ? LiDARType::Mech
                                  : LiDARType::Solid;
        }
        continue;
      }

      // Handle target section
      if (current_section == "target") {
        if (key == "marker_size")
          config.target.marker_size = std::stof(value);
        else if (key == "delta_width_qr_center")
          config.target.delta_width_qr_center = std::stof(value);
        else if (key == "delta_height_qr_center")
          config.target.delta_height_qr_center = std::stof(value);
        else if (key == "circle_radius")
          config.target.circle_radius = std::stof(value);
        continue;
      }

      // Handle output section
      if (current_section == "output") {
        if (key == "path")
          config.output.path = removeQuotes(value);
        else if (key == "parent_frame")
          config.output.parent_frame = removeQuotes(value);
        else if (key == "child_frame")
          config.output.child_frame = removeQuotes(value);
        continue;
      }

      // Handle debug section
      if (current_section == "debug") {
        if (key == "enabled")
          config.debug.enabled = parseBool(value);
        else if (key == "save_raw_data")
          config.debug.save_raw_data = parseBool(value);
        else if (key == "save_intermediate_clouds")
          config.debug.save_intermediate_clouds = parseBool(value);
        else if (key == "save_annotated_image")
          config.debug.save_annotated_image = parseBool(value);
        else if (key == "save_colored_cloud")
          config.debug.save_colored_cloud = parseBool(value);
        continue;
      }
    }

    return !config.scenes.empty() && config.camera.fx > 0;
  }

  /**
   * @brief Print configuration summary
   */
  static void printConfig(const CalibConfig& config) {
    std::cout << "\n=== Calibration Configuration ===" << std::endl;
    std::cout << "Camera:" << std::endl;
    std::cout << "  fx=" << config.camera.fx << ", fy=" << config.camera.fy
              << std::endl;
    std::cout << "  cx=" << config.camera.cx << ", cy=" << config.camera.cy
              << std::endl;
    std::cout << "  k1=" << config.camera.k1 << ", k2=" << config.camera.k2
              << std::endl;

    std::cout << "LiDAR type: "
              << (config.lidar_type == LiDARType::Solid ? "solid-state"
                                                        : "mechanical")
              << std::endl;

    std::cout << "Scenes: " << config.scenes.size() << std::endl;
    for (size_t i = 0; i < config.scenes.size(); ++i) {
      const auto& scene = config.scenes[i];
      std::cout << "  [" << i << "] " << scene.name << std::endl;
      std::cout << "      image: " << scene.image_path << std::endl;
      std::cout << "      pointcloud: " << scene.pointcloud_path << std::endl;
      if (scene.filter.enabled) {
        std::cout << "      filter: [" << scene.filter.min[0] << ","
                  << scene.filter.min[1] << "," << scene.filter.min[2]
                  << "] -> [" << scene.filter.max[0] << ","
                  << scene.filter.max[1] << "," << scene.filter.max[2] << "]"
                  << std::endl;
      }
    }

    std::cout << "Output: " << config.output.path << std::endl;
    std::cout << "Debug: " << (config.debug.enabled ? "enabled" : "disabled")
              << std::endl;
    std::cout << "================================\n" << std::endl;
  }

 private:
  static std::string trim(const std::string& s) {
    size_t start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    size_t end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
  }

  static std::string removeQuotes(const std::string& s) {
    std::string result = trim(s);
    if (result.size() >= 2) {
      if ((result.front() == '"' && result.back() == '"') ||
          (result.front() == '\'' && result.back() == '\'')) {
        return result.substr(1, result.size() - 2);
      }
    }
    return result;
  }

  static bool parseBool(const std::string& s) {
    std::string lower = s;
    std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
    return (lower == "true" || lower == "yes" || lower == "1");
  }

  static void parseArray3(const std::string& s, std::array<float, 3>& arr) {
    // Parse [x, y, z] format
    std::string cleaned = s;
    // Remove brackets
    cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), '['),
                  cleaned.end());
    cleaned.erase(std::remove(cleaned.begin(), cleaned.end(), ']'),
                  cleaned.end());

    std::stringstream ss(cleaned);
    std::string token;
    int i = 0;
    while (std::getline(ss, token, ',') && i < 3) {
      arr[i++] = std::stof(trim(token));
    }
  }
};

}  // namespace fast_calib

#endif  // FAST_CALIB_CORE_CONFIG_PARSER_HPP

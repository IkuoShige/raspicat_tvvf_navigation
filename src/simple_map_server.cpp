#include "raspicat_tvvf_navigation/simple_map_server.hpp"
#include <yaml-cpp/yaml.h>
#include <opencv2/opencv.hpp>
#include <filesystem>

namespace raspicat_tvvf_navigation
{

SimpleMapServer::SimpleMapServer()
: Node("simple_map_server")
{
  // Declare parameter
  this->declare_parameter("map_yaml_path", "");

  // Get parameter
  map_yaml_path_ = this->get_parameter("map_yaml_path").as_string();

  if (map_yaml_path_.empty()) {
    RCLCPP_ERROR(this->get_logger(), "map_yaml_path parameter is required!");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Loading map from: %s", map_yaml_path_.c_str());

  // Create publisher with Transient Local QoS (latched)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(1))
             .transient_local()
             .reliable();
  map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map", qos);

  // Load and publish map
  if (!loadAndPublishMap(map_yaml_path_)) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Map server initialized successfully");
}

bool SimpleMapServer::loadAndPublishMap(const std::string& yaml_path)
{
  // Parse YAML
  std::string image_path;
  double resolution;
  std::vector<double> origin;
  double occupied_thresh;
  double free_thresh;
  int negate;

  if (!parseMapYaml(yaml_path, image_path, resolution, origin,
                     occupied_thresh, free_thresh, negate)) {
    return false;
  }

  // Load image
  int width, height;
  std::vector<uint8_t> image_data;
  if (!loadImageFile(image_path, width, height, image_data)) {
    return false;
  }

  // Convert to OccupancyGrid
  map_msg_ = convertToOccupancyGrid(width, height, image_data, resolution,
                                      origin, occupied_thresh, free_thresh, negate);

  // Publish map
  map_pub_->publish(map_msg_);

  RCLCPP_INFO(this->get_logger(), "Map loaded successfully: %dx%d, resolution: %.3f m/px",
              width, height, resolution);

  return true;
}

bool SimpleMapServer::parseMapYaml(
  const std::string& yaml_path,
  std::string& image_path,
  double& resolution,
  std::vector<double>& origin,
  double& occupied_thresh,
  double& free_thresh,
  int& negate)
{
  try {
    YAML::Node config = YAML::LoadFile(yaml_path);

    // Parse required fields
    if (!config["image"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'image' field in YAML");
      return false;
    }
    if (!config["resolution"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'resolution' field in YAML");
      return false;
    }
    if (!config["origin"]) {
      RCLCPP_ERROR(this->get_logger(), "Missing 'origin' field in YAML");
      return false;
    }

    // Get image path (relative to YAML file)
    std::string image_filename = config["image"].as<std::string>();
    std::filesystem::path yaml_dir = std::filesystem::path(yaml_path).parent_path();
    image_path = (yaml_dir / image_filename).string();

    // Get resolution
    resolution = config["resolution"].as<double>();

    // Get origin [x, y, theta]
    origin.clear();
    for (const auto& val : config["origin"]) {
      origin.push_back(val.as<double>());
    }
    if (origin.size() != 3) {
      RCLCPP_ERROR(this->get_logger(), "Origin must have 3 values [x, y, theta]");
      return false;
    }

    // Get thresholds (with defaults)
    occupied_thresh = config["occupied_thresh"] ? config["occupied_thresh"].as<double>() : 0.65;
    free_thresh = config["free_thresh"] ? config["free_thresh"].as<double>() : 0.196;
    negate = config["negate"] ? config["negate"].as<int>() : 0;

    RCLCPP_INFO(this->get_logger(), "YAML parsed: image=%s, resolution=%.3f",
                image_filename.c_str(), resolution);

    return true;

  } catch (const YAML::Exception& e) {
    RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
    return false;
  }
}

bool SimpleMapServer::loadImageFile(
  const std::string& image_path,
  int& width,
  int& height,
  std::vector<uint8_t>& data)
{
  // Load image using OpenCV
  cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);

  if (image.empty()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load image: %s", image_path.c_str());
    return false;
  }

  width = image.cols;
  height = image.rows;

  // Convert to vector
  data.resize(width * height);
  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      data[y * width + x] = image.at<uint8_t>(y, x);
    }
  }

  RCLCPP_INFO(this->get_logger(), "Image loaded: %dx%d", width, height);

  return true;
}

nav_msgs::msg::OccupancyGrid SimpleMapServer::convertToOccupancyGrid(
  int width,
  int height,
  const std::vector<uint8_t>& image_data,
  double resolution,
  const std::vector<double>& origin,
  double occupied_thresh,
  double free_thresh,
  int negate)
{
  nav_msgs::msg::OccupancyGrid grid;

  // Header
  grid.header.stamp = this->now();
  grid.header.frame_id = "map";

  // Info
  grid.info.resolution = resolution;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.origin.position.x = origin[0];
  grid.info.origin.position.y = origin[1];
  grid.info.origin.position.z = 0.0;

  // Orientation from yaw (origin[2])
  double yaw = origin[2];
  grid.info.origin.orientation.x = 0.0;
  grid.info.origin.orientation.y = 0.0;
  grid.info.origin.orientation.z = std::sin(yaw / 2.0);
  grid.info.origin.orientation.w = std::cos(yaw / 2.0);

  // Data
  grid.data.resize(width * height);

  for (int y = 0; y < height; y++) {
    for (int x = 0; x < width; x++) {
      int index = y * width + x;
      uint8_t pixel = image_data[index];

      // Convert pixel value to occupancy probability
      double occ;
      if (negate) {
        occ = pixel / 255.0;
      } else {
        occ = (255 - pixel) / 255.0;
      }

      // Convert to OccupancyGrid value
      int8_t value;
      if (occ > occupied_thresh) {
        value = 100;  // Occupied
      } else if (occ < free_thresh) {
        value = 0;    // Free
      } else {
        value = -1;   // Unknown
      }

      // Note: OccupancyGrid uses row-major order with (0,0) at lower-left
      // Image uses row-major order with (0,0) at upper-left
      // Flip vertically
      int grid_index = (height - 1 - y) * width + x;
      grid.data[grid_index] = value;
    }
  }

  return grid;
}

}  // namespace raspicat_tvvf_navigation

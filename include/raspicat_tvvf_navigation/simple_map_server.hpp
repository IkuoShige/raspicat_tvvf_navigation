#ifndef RASPICAT_TVVF_NAVIGATION__SIMPLE_MAP_SERVER_HPP_
#define RASPICAT_TVVF_NAVIGATION__SIMPLE_MAP_SERVER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <string>

namespace raspicat_tvvf_navigation
{

class SimpleMapServer : public rclcpp::Node
{
public:
  SimpleMapServer();

private:
  /**
   * @brief Load map from YAML file and publish to /map topic
   * @param yaml_path Path to map YAML file
   * @return true if successful, false otherwise
   */
  bool loadAndPublishMap(const std::string& yaml_path);

  /**
   * @brief Parse map YAML file
   * @param yaml_path Path to YAML file
   * @param image_path Output: path to image file
   * @param resolution Output: map resolution
   * @param origin Output: map origin [x, y, theta]
   * @param occupied_thresh Output: occupied threshold
   * @param free_thresh Output: free threshold
   * @param negate Output: negate flag
   * @return true if successful, false otherwise
   */
  bool parseMapYaml(
    const std::string& yaml_path,
    std::string& image_path,
    double& resolution,
    std::vector<double>& origin,
    double& occupied_thresh,
    double& free_thresh,
    int& negate);

  /**
   * @brief Load image file (PGM or PNG)
   * @param image_path Path to image file
   * @param width Output: image width
   * @param height Output: image height
   * @param data Output: image data (grayscale)
   * @return true if successful, false otherwise
   */
  bool loadImageFile(
    const std::string& image_path,
    int& width,
    int& height,
    std::vector<uint8_t>& data);

  /**
   * @brief Convert image data to OccupancyGrid
   * @param width Image width
   * @param height Image height
   * @param image_data Image data (grayscale)
   * @param resolution Map resolution
   * @param origin Map origin
   * @param occupied_thresh Occupied threshold
   * @param free_thresh Free threshold
   * @param negate Negate flag
   * @return OccupancyGrid message
   */
  nav_msgs::msg::OccupancyGrid convertToOccupancyGrid(
    int width,
    int height,
    const std::vector<uint8_t>& image_data,
    double resolution,
    const std::vector<double>& origin,
    double occupied_thresh,
    double free_thresh,
    int negate);

  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  nav_msgs::msg::OccupancyGrid map_msg_;

  std::string map_yaml_path_;
};

}  // namespace raspicat_tvvf_navigation

#endif  // RASPICAT_TVVF_NAVIGATION__SIMPLE_MAP_SERVER_HPP_

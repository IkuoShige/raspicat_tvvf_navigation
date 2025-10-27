#ifndef ROBOMASTER_S1_WAYPOINT_NAVIGATION__CSV_READER_HPP_
#define ROBOMASTER_S1_WAYPOINT_NAVIGATION__CSV_READER_HPP_

#include <string>
#include <vector>
#include <geometry_msgs/msg/pose.hpp>

namespace raspicat_tvvf_navigation
{

struct Waypoint
{
  int id;
  geometry_msgs::msg::Pose pose;
  std::string command;
  bool reached;
  bool skipped;
  int retry_count;

  Waypoint()
  : id(0), reached(false), skipped(false), retry_count(0) {}
};

class CSVReader
{
public:
  /**
   * @brief Read waypoints from CSV file
   * @param filename Path to CSV file
   * @return Vector of waypoints
   * @throws std::runtime_error if file cannot be read or format is invalid
   */
  static std::vector<Waypoint> readWaypoints(const std::string& filename);

private:
  static std::vector<std::string> splitLine(const std::string& line, char delimiter);
  static Waypoint parseLine(const std::vector<std::string>& fields, int line_number);
};

}  // namespace raspicat_tvvf_navigation

#endif  // ROBOMASTER_S1_WAYPOINT_NAVIGATION__CSV_READER_HPP_

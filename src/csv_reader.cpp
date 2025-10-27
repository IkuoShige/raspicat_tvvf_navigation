#include "raspicat_tvvf_navigation/csv_reader.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <rclcpp/rclcpp.hpp>

namespace raspicat_tvvf_navigation
{

std::vector<Waypoint> CSVReader::readWaypoints(const std::string& filename)
{
  std::vector<Waypoint> waypoints;
  std::ifstream file(filename);

  if (!file.is_open()) {
    throw std::runtime_error("Cannot open CSV file: " + filename);
  }

  std::string line;
  int line_number = 0;

  // Read header line
  if (!std::getline(file, line)) {
    throw std::runtime_error("Empty CSV file");
  }
  line_number++;

  // Expected header: id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
  RCLCPP_INFO(rclcpp::get_logger("csv_reader"), "CSV Header: %s", line.c_str());

  // Read data lines
  while (std::getline(file, line)) {
    line_number++;

    // Skip empty lines
    if (line.empty() || line.find_first_not_of(" \t\r\n") == std::string::npos) {
      continue;
    }

    try {
      auto fields = splitLine(line, ',');
      if (fields.size() < 8) {
        RCLCPP_WARN(rclcpp::get_logger("csv_reader"),
                    "Line %d: insufficient fields (%zu), skipping",
                    line_number, fields.size());
        continue;
      }

      Waypoint wp = parseLine(fields, line_number);
      waypoints.push_back(wp);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("csv_reader"),
                   "Line %d: parsing error: %s", line_number, e.what());
      throw;
    }
  }

  file.close();

  if (waypoints.empty()) {
    throw std::runtime_error("No valid waypoints found in CSV file");
  }

  RCLCPP_INFO(rclcpp::get_logger("csv_reader"),
              "Successfully loaded %zu waypoints from %s",
              waypoints.size(), filename.c_str());

  return waypoints;
}

std::vector<std::string> CSVReader::splitLine(const std::string& line, char delimiter)
{
  std::vector<std::string> fields;
  std::stringstream ss(line);
  std::string field;

  while (std::getline(ss, field, delimiter)) {
    // Trim whitespace
    field.erase(0, field.find_first_not_of(" \t\r\n"));
    field.erase(field.find_last_not_of(" \t\r\n") + 1);
    fields.push_back(field);
  }

  return fields;
}

Waypoint CSVReader::parseLine(const std::vector<std::string>& fields, int line_number)
{
  Waypoint wp;

  try {
    // id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
    wp.id = std::stoi(fields[0]);

    wp.pose.position.x = std::stod(fields[1]);
    wp.pose.position.y = std::stod(fields[2]);
    wp.pose.position.z = std::stod(fields[3]);

    wp.pose.orientation.x = std::stod(fields[4]);
    wp.pose.orientation.y = std::stod(fields[5]);
    wp.pose.orientation.z = std::stod(fields[6]);
    wp.pose.orientation.w = std::stod(fields[7]);

    // Command field is optional (index 8)
    if (fields.size() > 8) {
      wp.command = fields[8];
    } else {
      wp.command = "";
    }

    wp.reached = false;
    wp.skipped = false;
    wp.retry_count = 0;

  } catch (const std::exception& e) {
    throw std::runtime_error(
      "Failed to parse waypoint at line " + std::to_string(line_number) +
      ": " + std::string(e.what()));
  }

  return wp;
}

}  // namespace raspicat_tvvf_navigation

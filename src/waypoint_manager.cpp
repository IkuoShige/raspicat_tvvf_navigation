#include "raspicat_tvvf_navigation/waypoint_manager.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "raspicat_tvvf_navigation/command_utils.hpp"

namespace raspicat_tvvf_navigation
{

WaypointManager::WaypointManager(double position_tolerance_strict,
                                 double orientation_tolerance_strict,
                                 double position_tolerance_loose,
                                 double orientation_tolerance_loose)
: current_index_(0),
  position_tolerance_strict_(position_tolerance_strict),
  orientation_tolerance_strict_(orientation_tolerance_strict),
  position_tolerance_loose_(position_tolerance_loose),
  orientation_tolerance_loose_(orientation_tolerance_loose)
{
}

bool WaypointManager::loadWaypoints(const std::string& filename)
{
  try {
    waypoints_ = CSVReader::readWaypoints(filename);
    current_index_ = 0;
    return true;
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("waypoint_manager"),
                 "Failed to load waypoints: %s", e.what());
    return false;
  }
}

std::optional<Waypoint> WaypointManager::getCurrentWaypoint() const
{
  if (current_index_ >= waypoints_.size()) {
    return std::nullopt;
  }
  return waypoints_[current_index_];
}

std::optional<Waypoint> WaypointManager::getNextWaypoint() const
{
  size_t next_index = current_index_ + 1;
  if (next_index >= waypoints_.size()) {
    return std::nullopt;
  }
  return waypoints_[next_index];
}

bool WaypointManager::isWaypointReached(const geometry_msgs::msg::Pose& current_pose) const
{
  auto wp = getCurrentWaypoint();
  if (!wp.has_value()) {
    return false;
  }

  double distance = calculateDistance(current_pose, wp->pose);
  double yaw_diff = calculateYawDiff(current_pose, wp->pose);

  const auto parsed = parse_command(wp->command);
  const bool use_strict_tolerance = (parsed.tolerance_mode == ToleranceMode::STRICT);
  const double pos_tol = use_strict_tolerance
      ? position_tolerance_strict_
      : position_tolerance_loose_;
  const double ori_tol = use_strict_tolerance
      ? orientation_tolerance_strict_
      : orientation_tolerance_loose_;

  const double kEpsilon = 1e-9;
  bool position_reached = distance <= (pos_tol + kEpsilon);
  bool orientation_reached = std::abs(yaw_diff) <= (ori_tol + kEpsilon);

  return position_reached && orientation_reached;
}

double WaypointManager::getDistanceToWaypoint(const geometry_msgs::msg::Pose& current_pose) const
{
  auto wp = getCurrentWaypoint();
  if (!wp.has_value()) {
    return 0.0;
  }
  return calculateDistance(current_pose, wp->pose);
}

double WaypointManager::getOrientationDiff(const geometry_msgs::msg::Pose& current_pose) const
{
  auto wp = getCurrentWaypoint();
  if (!wp.has_value()) {
    return 0.0;
  }
  return calculateYawDiff(current_pose, wp->pose);
}

void WaypointManager::markCurrentReached()
{
  if (current_index_ < waypoints_.size()) {
    waypoints_[current_index_].reached = true;
    current_index_++;
  }
}

void WaypointManager::skipCurrentWaypoint()
{
  if (current_index_ < waypoints_.size()) {
    waypoints_[current_index_].skipped = true;
    current_index_++;
  }
}

void WaypointManager::incrementRetryCount()
{
  if (current_index_ < waypoints_.size()) {
    waypoints_[current_index_].retry_count++;
  }
}

int WaypointManager::getCurrentRetryCount() const
{
  if (current_index_ < waypoints_.size()) {
    return waypoints_[current_index_].retry_count;
  }
  return 0;
}

bool WaypointManager::isCompleted() const
{
  return current_index_ >= waypoints_.size();
}

size_t WaypointManager::getTotalWaypoints() const
{
  return waypoints_.size();
}

size_t WaypointManager::getCompletedWaypoints() const
{
  size_t count = 0;
  for (const auto& wp : waypoints_) {
    if (wp.reached) {
      count++;
    }
  }
  return count;
}

size_t WaypointManager::getSkippedWaypoints() const
{
  size_t count = 0;
  for (const auto& wp : waypoints_) {
    if (wp.skipped) {
      count++;
    }
  }
  return count;
}

const std::vector<Waypoint>& WaypointManager::getAllWaypoints() const
{
  return waypoints_;
}

int WaypointManager::getCurrentIndex() const
{
  return static_cast<int>(current_index_);
}

void WaypointManager::reset()
{
  current_index_ = 0;
  for (auto& wp : waypoints_) {
    wp.reached = false;
    wp.skipped = false;
    wp.retry_count = 0;
  }
}

double WaypointManager::calculateDistance(
  const geometry_msgs::msg::Pose& p1,
  const geometry_msgs::msg::Pose& p2) const
{
  double dx = p1.position.x - p2.position.x;
  double dy = p1.position.y - p2.position.y;
  double dz = p1.position.z - p2.position.z;
  return std::sqrt(dx*dx + dy*dy + dz*dz);
}

double WaypointManager::calculateYawDiff(
  const geometry_msgs::msg::Pose& p1,
  const geometry_msgs::msg::Pose& p2) const
{
  // Convert quaternion to yaw angle
  auto getYaw = [](const geometry_msgs::msg::Quaternion& q) {
    // yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
    double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
  };

  double yaw1 = getYaw(p1.orientation);
  double yaw2 = getYaw(p2.orientation);

  // Normalize angle difference to [-pi, pi]
  double diff = yaw1 - yaw2;
  while (diff > M_PI) diff -= 2.0 * M_PI;
  while (diff < -M_PI) diff += 2.0 * M_PI;

  return diff;
}

}  // namespace raspicat_tvvf_navigation

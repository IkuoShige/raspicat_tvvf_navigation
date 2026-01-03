#ifndef ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_
#define ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_

#include <vector>
#include <optional>
#include <geometry_msgs/msg/pose.hpp>
#include "raspicat_tvvf_navigation/csv_reader.hpp"

namespace raspicat_tvvf_navigation
{

class WaypointManager
{
public:
  WaypointManager(double position_tolerance_strict,
                  double orientation_tolerance_strict,
                  double position_tolerance_loose,
                  double orientation_tolerance_loose);

  /**
   * @brief Load waypoints from CSV file
   */
  bool loadWaypoints(const std::string& filename);

  /**
   * @brief Get current waypoint
   */
  std::optional<Waypoint> getCurrentWaypoint() const;

  /**
   * @brief Get next waypoint (current_index + 1)
   */
  std::optional<Waypoint> getNextWaypoint() const;

  /**
   * @brief Check if current waypoint is reached
   */
  bool isWaypointReached(const geometry_msgs::msg::Pose& current_pose) const;

  /**
   * @brief Calculate distance to current waypoint
   */
  double getDistanceToWaypoint(const geometry_msgs::msg::Pose& current_pose) const;

  /**
   * @brief Calculate orientation difference to current waypoint (radians)
   */
  double getOrientationDiff(const geometry_msgs::msg::Pose& current_pose) const;

  /**
   * @brief Mark current waypoint as reached and move to next
   */
  void markCurrentReached();

  /**
   * @brief Skip current waypoint
   */
  void skipCurrentWaypoint();

  /**
   * @brief Increment retry count for current waypoint
   */
  void incrementRetryCount();

  /**
   * @brief Get retry count for current waypoint
   */
  int getCurrentRetryCount() const;

  /**
   * @brief Check if all waypoints are completed
   */
  bool isCompleted() const;

  /**
   * @brief Get total number of waypoints
   */
  size_t getTotalWaypoints() const;

  /**
   * @brief Get number of completed waypoints
   */
  size_t getCompletedWaypoints() const;

  /**
   * @brief Get number of skipped waypoints
   */
  size_t getSkippedWaypoints() const;

  /**
   * @brief Get all waypoints (for visualization)
   */
  const std::vector<Waypoint>& getAllWaypoints() const;

  /**
   * @brief Get current waypoint index
   */
  int getCurrentIndex() const;

  /**
   * @brief Reset to initial state
   */
  void reset();

private:
  std::vector<Waypoint> waypoints_;
  size_t current_index_;
  double position_tolerance_strict_;
  double orientation_tolerance_strict_;
  double position_tolerance_loose_;
  double orientation_tolerance_loose_;

  double calculateDistance(const geometry_msgs::msg::Pose& p1,
                          const geometry_msgs::msg::Pose& p2) const;
  double calculateYawDiff(const geometry_msgs::msg::Pose& p1,
                         const geometry_msgs::msg::Pose& p2) const;
};

}  // namespace raspicat_tvvf_navigation

#endif  // ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_MANAGER_HPP_

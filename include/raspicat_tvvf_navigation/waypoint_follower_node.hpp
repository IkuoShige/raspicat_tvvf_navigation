#ifndef ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_FOLLOWER_NODE_HPP_
#define ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_FOLLOWER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include <string>

#include "raspicat_tvvf_navigation/waypoint_manager.hpp"
#include "raspicat_tvvf_navigation/msg/waypoint_status.hpp"
#include "raspicat_tvvf_navigation/tolerance_config.hpp"
#include "raspicat_tvvf_navigation/command_utils.hpp"

namespace raspicat_tvvf_navigation
{

enum class NavigationState
{
  IDLE,
  LOADING_WAYPOINTS,
  NAVIGATING,
  WAYPOINT_REACHED,
  WAITING,
  EXECUTING_COMMAND,
  COMPLETED,
  ERROR
};

enum class WaitReason
{
  NONE,           // Not waiting
  TIME,           // wait:N - waiting for time duration
  TOPIC,          // wait_topic:/topic_name - waiting for topic
  PAUSE           // pause - waiting for manual resume
};

class WaypointFollowerNode : public rclcpp::Node
{
public:
  WaypointFollowerNode();

private:
  // State machine
  NavigationState current_state_;
  void transitionToState(NavigationState new_state);
  std::string stateToString(NavigationState state) const;

  // State handlers
  void handleIdleState();
  void handleLoadingState();
  void handleNavigatingState(const std::optional<geometry_msgs::msg::Pose>& robot_pose);
  void handleWaypointReachedState();
  void handleWaitingState();
  void handleExecutingCommandState();
  void handleCompletedState();
  void handleErrorState();

  // Timestamp initialization
  void initializeTimestamps();

  // Command execution
  void executeWaypointCommand(const std::string& command);
  bool parseWaitCommand(const std::string& command, double& wait_seconds);
  bool parseWaitTopicCommand(const std::string& command, std::string& topic_name);

  // Wait topic callback
  void waitTopicCallback(const std_msgs::msg::Bool::SharedPtr msg);

  // RViz visualization
  void publishWaypointMarkers();
  visualization_msgs::msg::Marker createWaypointMarker(
    const Waypoint& wp, int index, bool is_current, bool is_reached, bool is_skipped);

  // Service callbacks
  void startNavigationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void pauseNavigationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void resumeNavigationCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  void skipWaypointCallback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // Publishers
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoint_markers_pub_;
  rclcpp::Publisher<msg::WaypointStatus>::SharedPtr status_pub_;

  // Subscribers
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wait_topic_sub_;

  // Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr resume_service_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr skip_service_;

  // Timer
  rclcpp::TimerBase::SharedPtr control_timer_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Waypoint management
  std::unique_ptr<WaypointManager> waypoint_manager_;

  // Parameters
  std::string waypoint_csv_path_;
  double position_tolerance_strict_;
  double orientation_tolerance_strict_;
  double position_tolerance_loose_;
  double orientation_tolerance_loose_;
  int max_retry_count_;
  double goal_timeout_;
  bool auto_start_;
  bool enable_skip_on_timeout_;
  bool loop_navigation_;
  std::string global_frame_;
  std::string robot_base_frame_;

  // State variables
  bool paused_;
  bool goal_sent_;  // true if goal_pose already sent for current waypoint
  rclcpp::Time goal_sent_time_;
  rclcpp::Time wait_start_time_;
  double wait_duration_;
  WaitReason wait_reason_;
  std::string wait_topic_name_;
  bool wait_topic_received_;
  std::string error_message_;

protected:
  // Testing helpers
  bool loadWaypointsForTest(const std::string& path) { return waypoint_manager_->loadWaypoints(path); }
  void setStateForTest(NavigationState state) { current_state_ = state; }
  NavigationState getStateForTest() const { return current_state_; }
  void markReachedForTest() { waypoint_manager_->markCurrentReached(); }

  // Main control loop
  void controlLoop();

  // TF utilities (protected for tests)
  virtual std::optional<geometry_msgs::msg::Pose> getRobotPose();
};

}  // namespace raspicat_tvvf_navigation

#endif  // ROBOMASTER_S1_WAYPOINT_NAVIGATION__WAYPOINT_FOLLOWER_NODE_HPP_

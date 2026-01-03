#include "raspicat_tvvf_navigation/waypoint_follower_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <chrono>

using namespace std::chrono_literals;

namespace raspicat_tvvf_navigation
{

WaypointFollowerNode::WaypointFollowerNode()
: Node("waypoint_follower_node"),
  current_state_(NavigationState::IDLE),
  paused_(false),
  goal_sent_(false),
  wait_duration_(0.0),
  wait_reason_(WaitReason::NONE),
  wait_topic_received_(false)
{
  // Declare parameters
  this->declare_parameter("waypoint_csv_path", "");
  this->declare_parameter("position_tolerance_strict", 0.3);
  this->declare_parameter("orientation_tolerance_strict", 0.3);
  this->declare_parameter("position_tolerance_loose", 0.5);
  this->declare_parameter("orientation_tolerance_loose", 3.14);
  // レガシー互換用（旧パラメータ名）
  this->declare_parameter("position_tolerance", -1.0);
  this->declare_parameter("orientation_tolerance", -1.0);
  this->declare_parameter("max_retry_count", 3);
  this->declare_parameter("goal_timeout", 30.0);
  this->declare_parameter("auto_start", false);
  this->declare_parameter("enable_skip_on_timeout", true);
  this->declare_parameter("loop_navigation", false);
  this->declare_parameter("global_frame", "map");
  this->declare_parameter("robot_base_frame", "base_link");

  // Get parameters
  waypoint_csv_path_ = this->get_parameter("waypoint_csv_path").as_string();
  position_tolerance_strict_ = this->get_parameter("position_tolerance_strict").as_double();
  orientation_tolerance_strict_ = this->get_parameter("orientation_tolerance_strict").as_double();
  position_tolerance_loose_ = this->get_parameter("position_tolerance_loose").as_double();
  orientation_tolerance_loose_ = this->get_parameter("orientation_tolerance_loose").as_double();
  const double legacy_position_tolerance = this->get_parameter("position_tolerance").as_double();
  const double legacy_orientation_tolerance = this->get_parameter("orientation_tolerance").as_double();

  {
    ToleranceConfig raw{position_tolerance_strict_, orientation_tolerance_strict_,
                        position_tolerance_loose_, orientation_tolerance_loose_};
    ToleranceConfig defaults{0.3, 0.3, 0.5, 3.14};
    auto resolved = resolve_tolerance_config(
      raw, defaults, legacy_position_tolerance, legacy_orientation_tolerance);
    position_tolerance_strict_ = resolved.position_strict;
    orientation_tolerance_strict_ = resolved.orientation_strict;
    position_tolerance_loose_ = resolved.position_loose;
    orientation_tolerance_loose_ = resolved.orientation_loose;
  }
  max_retry_count_ = this->get_parameter("max_retry_count").as_int();
  goal_timeout_ = this->get_parameter("goal_timeout").as_double();
  auto_start_ = this->get_parameter("auto_start").as_bool();
  enable_skip_on_timeout_ = this->get_parameter("enable_skip_on_timeout").as_bool();
  loop_navigation_ = this->get_parameter("loop_navigation").as_bool();
  global_frame_ = this->get_parameter("global_frame").as_string();
  robot_base_frame_ = this->get_parameter("robot_base_frame").as_string();

  // Create waypoint manager
  waypoint_manager_ = std::make_unique<WaypointManager>(
    position_tolerance_strict_,
    orientation_tolerance_strict_,
    position_tolerance_loose_,
    orientation_tolerance_loose_);

  // TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Publishers
  goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "goal_pose", 10);
  next_goal_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "next_goal_pose", 10);
  waypoint_markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "waypoint_markers", 10);
  status_pub_ = this->create_publisher<msg::WaypointStatus>(
    "waypoint_status", 10);

  // Services
  start_service_ = this->create_service<std_srvs::srv::Trigger>(
    "start_waypoint_navigation",
    std::bind(&WaypointFollowerNode::startNavigationCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "pause_waypoint_navigation",
    std::bind(&WaypointFollowerNode::pauseNavigationCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  resume_service_ = this->create_service<std_srvs::srv::Trigger>(
    "resume_waypoint_navigation",
    std::bind(&WaypointFollowerNode::resumeNavigationCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  skip_service_ = this->create_service<std_srvs::srv::Trigger>(
    "skip_current_waypoint",
    std::bind(&WaypointFollowerNode::skipWaypointCallback, this,
              std::placeholders::_1, std::placeholders::_2));

  // Control timer (10 Hz)
  control_timer_ = this->create_wall_timer(
    100ms, std::bind(&WaypointFollowerNode::controlLoop, this));

  // Initialize timestamps with node clock to avoid time source mismatch
  initializeTimestamps();

  RCLCPP_INFO(this->get_logger(), "Waypoint Follower Node initialized");
  RCLCPP_INFO(this->get_logger(), "CSV path: %s", waypoint_csv_path_.c_str());
  RCLCPP_INFO(this->get_logger(), "auto_start parameter: %s", auto_start_ ? "true" : "false");

  // Auto-start navigation if enabled
  if (auto_start_) {
    RCLCPP_INFO(this->get_logger(), "Auto-start enabled, beginning navigation...");
    waypoint_manager_->reset();
    paused_ = false;
    transitionToState(NavigationState::LOADING_WAYPOINTS);
  } else {
    RCLCPP_INFO(this->get_logger(), "Auto-start disabled. Call /start_waypoint_navigation service to begin.");
  }
}

void WaypointFollowerNode::initializeTimestamps()
{
  // Initialize timestamps with node clock to match use_sim_time setting
  const auto now = this->now();
  goal_sent_time_ = now;
  wait_start_time_ = now;
}

void WaypointFollowerNode::controlLoop()
{
  if (paused_) {
    return;
  }

  const auto robot_pose = getRobotPose();

  switch (current_state_) {
    case NavigationState::IDLE:
      handleIdleState();
      break;
    case NavigationState::LOADING_WAYPOINTS:
      handleLoadingState();
      break;
    case NavigationState::NAVIGATING:
      handleNavigatingState(robot_pose);
      break;
    case NavigationState::WAYPOINT_REACHED:
      handleWaypointReachedState();
      break;
    case NavigationState::WAITING:
      handleWaitingState();
      break;
    case NavigationState::EXECUTING_COMMAND:
      handleExecutingCommandState();
      break;
    case NavigationState::COMPLETED:
      handleCompletedState();
      break;
    case NavigationState::ERROR:
      handleErrorState();
      break;
  }

  // Publish visualization and status
  publishWaypointMarkers();

  // Publish status
  auto status_msg = msg::WaypointStatus();
  auto current_wp = waypoint_manager_->getCurrentWaypoint();

  if (current_wp.has_value()) {
    status_msg.current_waypoint_id = current_wp->id;
    status_msg.current_command = current_wp->command;

    if (robot_pose.has_value()) {
      status_msg.distance_to_goal = waypoint_manager_->getDistanceToWaypoint(*robot_pose);
      status_msg.orientation_diff = waypoint_manager_->getOrientationDiff(*robot_pose);
    }
  } else {
    status_msg.current_waypoint_id = -1;
  }

  status_msg.total_waypoints = static_cast<int>(waypoint_manager_->getTotalWaypoints());
  status_msg.completed_waypoints = static_cast<int>(waypoint_manager_->getCompletedWaypoints());
  status_msg.skipped_waypoints = static_cast<int>(waypoint_manager_->getSkippedWaypoints());
  status_msg.state = stateToString(current_state_);

  status_pub_->publish(status_msg);
}

void WaypointFollowerNode::handleIdleState()
{
  // Do nothing, waiting for start service call
}

void WaypointFollowerNode::handleLoadingState()
{
  RCLCPP_INFO(this->get_logger(), "Loading waypoints from: %s", waypoint_csv_path_.c_str());

  if (waypoint_manager_->loadWaypoints(waypoint_csv_path_)) {
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %zu waypoints",
                waypoint_manager_->getTotalWaypoints());
    transitionToState(NavigationState::NAVIGATING);
  } else {
    error_message_ = "Failed to load waypoints from CSV file";
    transitionToState(NavigationState::ERROR);
  }
}

void WaypointFollowerNode::handleNavigatingState(const std::optional<geometry_msgs::msg::Pose>& robot_pose)
{
  auto current_wp = waypoint_manager_->getCurrentWaypoint();

  if (!current_wp.has_value()) {
    // No more waypoints
    transitionToState(NavigationState::COMPLETED);
    return;
  }

  if (!robot_pose.has_value()) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Cannot get robot pose");
    return;
  }

  // Check if waypoint is reached
  if (waypoint_manager_->isWaypointReached(*robot_pose)) {
    RCLCPP_INFO(this->get_logger(), "Waypoint %d reached!", current_wp->id);
    transitionToState(NavigationState::WAYPOINT_REACHED);
    return;
  }

  // Check timeout (only if goal has been sent)
  if (goal_sent_ && (this->now() - goal_sent_time_).seconds() > goal_timeout_) {
    RCLCPP_WARN(this->get_logger(), "Goal timeout for waypoint %d", current_wp->id);

    waypoint_manager_->incrementRetryCount();

    if (waypoint_manager_->getCurrentRetryCount() >= max_retry_count_) {
      if (enable_skip_on_timeout_) {
        RCLCPP_WARN(this->get_logger(), "Max retries reached, skipping waypoint %d",
                    current_wp->id);
        waypoint_manager_->skipCurrentWaypoint();
        goal_sent_ = false;  // Reset for next waypoint
        transitionToState(NavigationState::NAVIGATING);
      } else {
        error_message_ = "Max retries reached for waypoint " +
                        std::to_string(current_wp->id);
        transitionToState(NavigationState::ERROR);
      }
      return;
    }

    // Retry: allow re-sending goal
    RCLCPP_INFO(this->get_logger(), "Retrying waypoint %d (attempt %d/%d)",
                current_wp->id,
                waypoint_manager_->getCurrentRetryCount() + 1,
                max_retry_count_);
    goal_sent_ = false;  // Reset to allow re-sending
  }

  // Send goal pose only once per waypoint (or on retry)
  if (!goal_sent_) {
    auto goal_msg = geometry_msgs::msg::PoseStamped();
    goal_msg.header.stamp = this->now();
    goal_msg.header.frame_id = global_frame_;
    goal_msg.pose = current_wp->pose;

    goal_pose_pub_->publish(goal_msg);
    goal_sent_time_ = this->now();
    goal_sent_ = true;

    RCLCPP_INFO(this->get_logger(), "Sent goal for waypoint %d (x=%.2f, y=%.2f)",
                current_wp->id, current_wp->pose.position.x, current_wp->pose.position.y);

    auto next_wp = waypoint_manager_->getNextWaypoint();
    if (next_wp.has_value()) {
      auto next_goal_msg = geometry_msgs::msg::PoseStamped();
      next_goal_msg.header.stamp = this->now();
      next_goal_msg.header.frame_id = global_frame_;
      next_goal_msg.pose = next_wp->pose;
      next_goal_pose_pub_->publish(next_goal_msg);
      RCLCPP_INFO(this->get_logger(), "Published next goal for waypoint %d (x=%.2f, y=%.2f)",
                  next_wp->id, next_wp->pose.position.x, next_wp->pose.position.y);
    }
  }
}

void WaypointFollowerNode::handleWaypointReachedState()
{
  auto current_wp = waypoint_manager_->getCurrentWaypoint();
  if (!current_wp.has_value()) {
    transitionToState(NavigationState::COMPLETED);
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Reached waypoint %d", current_wp->id);

  const bool has_command = is_action_command(current_wp->command);

  // Execute command if present (strict/loose/空は通常コマンド扱いせず即到達とみなす)
  if (has_command) {
    // Do NOT mark as reached here - it will be marked when command completes
    executeWaypointCommand(current_wp->command);
  } else {
    // No command, mark as reached and proceed to next waypoint
    waypoint_manager_->markCurrentReached();

    if (waypoint_manager_->isCompleted()) {
      if (loop_navigation_) {
        RCLCPP_INFO(this->get_logger(), "All waypoints completed, looping back to start");
        waypoint_manager_->reset();
        transitionToState(NavigationState::NAVIGATING);
      } else {
        transitionToState(NavigationState::COMPLETED);
      }
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }
  }
}

void WaypointFollowerNode::handleWaitingState()
{
  bool wait_complete = false;

  switch (wait_reason_) {
    case WaitReason::TIME:
      // Time-based wait (wait:N)
      if ((this->now() - wait_start_time_).seconds() >= wait_duration_) {
        RCLCPP_INFO(this->get_logger(), "Time-based wait complete");
        wait_complete = true;
      }
      break;

    case WaitReason::TOPIC:
      // Topic-based wait (wait_topic:/topic_name)
      if (wait_topic_received_) {
        RCLCPP_INFO(this->get_logger(), "Topic-based wait complete");
        wait_complete = true;
      }
      break;

    case WaitReason::PAUSE:
      // Manual pause (pause command) - wait for resume service
      // paused_ flag is checked, no automatic completion
      break;

    case WaitReason::NONE:
      // Should not be in WAITING state with NONE reason
      RCLCPP_WARN(this->get_logger(), "In WAITING state but wait_reason is NONE");
      wait_complete = true;
      break;
  }

  // If wait is complete, proceed to next waypoint
  if (wait_complete) {
    wait_reason_ = WaitReason::NONE;
    waypoint_manager_->markCurrentReached();

    if (waypoint_manager_->isCompleted()) {
      if (loop_navigation_) {
        RCLCPP_INFO(this->get_logger(), "All waypoints completed, looping back to start");
        waypoint_manager_->reset();
        transitionToState(NavigationState::NAVIGATING);
      } else {
        transitionToState(NavigationState::COMPLETED);
      }
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }
  }
}

void WaypointFollowerNode::handleExecutingCommandState()
{
  // Currently handled in executeWaypointCommand
  // Transition is done there
}

void WaypointFollowerNode::handleCompletedState()
{
  RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                       "All waypoints completed! Completed: %zu, Skipped: %zu",
                       waypoint_manager_->getCompletedWaypoints(),
                       waypoint_manager_->getSkippedWaypoints());
}

void WaypointFollowerNode::handleErrorState()
{
  RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Navigation error: %s", error_message_.c_str());
}

void WaypointFollowerNode::executeWaypointCommand(const std::string& command)
{
  RCLCPP_INFO(this->get_logger(), "Executing command: %s", command.c_str());

  const auto parsed = parse_command(command);
  const std::string& action = parsed.action_command;

  if (action == "continue" || action.empty()) {
    if (waypoint_manager_->isCompleted()) {
      transitionToState(NavigationState::COMPLETED);
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }
  }
  else if (action == "pause" || action == "stop") {
    RCLCPP_INFO(this->get_logger(), "Pause command - waiting for manual resume");
    wait_reason_ = WaitReason::PAUSE;
    paused_ = true;
    transitionToState(NavigationState::WAITING);
  }
  else if (action.rfind("wait:", 0) == 0) {
    double wait_seconds;
    if (parseWaitCommand(action, wait_seconds)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for %.1f seconds", wait_seconds);
      wait_reason_ = WaitReason::TIME;
      wait_start_time_ = this->now();
      wait_duration_ = wait_seconds;
      transitionToState(NavigationState::WAITING);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid wait command format: %s", command.c_str());
      transitionToState(NavigationState::NAVIGATING);
    }
  }
  else if (action.rfind("wait_topic:", 0) == 0) {
    std::string topic_name;
    if (parseWaitTopicCommand(action, topic_name)) {
      RCLCPP_INFO(this->get_logger(), "Waiting for topic: %s", topic_name.c_str());
      wait_reason_ = WaitReason::TOPIC;
      wait_topic_name_ = topic_name;
      wait_topic_received_ = false;

      // Create subscriber for the wait topic
      wait_topic_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        topic_name, 10,
        std::bind(&WaypointFollowerNode::waitTopicCallback, this, std::placeholders::_1));

      transitionToState(NavigationState::WAITING);
    } else {
      RCLCPP_WARN(this->get_logger(), "Invalid wait_topic command format: %s", command.c_str());
      transitionToState(NavigationState::NAVIGATING);
    }
  }
  else if (action == "skip_if_fail") {
    // This is handled in the navigation timeout logic
    if (waypoint_manager_->isCompleted()) {
      transitionToState(NavigationState::COMPLETED);
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }
  }
  else {
    RCLCPP_WARN(this->get_logger(), "Unknown command: %s, continuing...", command.c_str());
    if (waypoint_manager_->isCompleted()) {
      transitionToState(NavigationState::COMPLETED);
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }
  }
}

bool WaypointFollowerNode::parseWaitCommand(const std::string& command, double& wait_seconds)
{
  try {
    // Format: "wait:N" where N is seconds
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos) {
      return false;
    }

    std::string seconds_str = command.substr(colon_pos + 1);
    wait_seconds = std::stod(seconds_str);
    return wait_seconds >= 0.0;
  } catch (...) {
    return false;
  }
}

bool WaypointFollowerNode::parseWaitTopicCommand(const std::string& command, std::string& topic_name)
{
  try {
    // Format: "wait_topic:/topic_name"
    size_t colon_pos = command.find(':');
    if (colon_pos == std::string::npos || colon_pos >= command.length() - 1) {
      return false;
    }

    topic_name = command.substr(colon_pos + 1);
    // Topic name should not be empty and should start with /
    return !topic_name.empty();
  } catch (...) {
    return false;
  }
}

void WaypointFollowerNode::waitTopicCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (wait_reason_ == WaitReason::TOPIC && msg->data) {
    RCLCPP_INFO(this->get_logger(), "Received true from topic %s", wait_topic_name_.c_str());
    wait_topic_received_ = true;
    // Unsubscribe to avoid accumulating subscriptions
    wait_topic_sub_.reset();
  }
}

void WaypointFollowerNode::publishWaypointMarkers()
{
  visualization_msgs::msg::MarkerArray marker_array;

  const auto& waypoints = waypoint_manager_->getAllWaypoints();
  int current_index = waypoint_manager_->getCurrentIndex();

  for (size_t i = 0; i < waypoints.size(); ++i) {
    bool is_current = (static_cast<int>(i) == current_index);
    bool is_reached = waypoints[i].reached;
    bool is_skipped = waypoints[i].skipped;

    auto marker = createWaypointMarker(waypoints[i], i, is_current, is_reached, is_skipped);
    marker_array.markers.push_back(marker);
  }

  // Add path line markers
  if (waypoints.size() > 1) {
    visualization_msgs::msg::Marker line_marker;
    line_marker.header.frame_id = global_frame_;
    line_marker.header.stamp = this->now();
    line_marker.ns = "waypoint_path";
    line_marker.id = static_cast<int>(waypoints.size());
    line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_marker.action = visualization_msgs::msg::Marker::ADD;
    line_marker.scale.x = 0.05;
    line_marker.color.r = 0.5;
    line_marker.color.g = 0.5;
    line_marker.color.b = 1.0;
    line_marker.color.a = 0.5;

    for (const auto& wp : waypoints) {
      geometry_msgs::msg::Point p;
      p.x = wp.pose.position.x;
      p.y = wp.pose.position.y;
      p.z = wp.pose.position.z + 0.1;
      line_marker.points.push_back(p);
    }

    marker_array.markers.push_back(line_marker);
  }

  waypoint_markers_pub_->publish(marker_array);
}

visualization_msgs::msg::Marker WaypointFollowerNode::createWaypointMarker(
  const Waypoint& wp, int index, bool is_current, bool is_reached, bool is_skipped)
{
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = global_frame_;
  marker.header.stamp = this->now();
  marker.ns = "waypoints";
  marker.id = index;
  marker.type = visualization_msgs::msg::Marker::ARROW;
  marker.action = visualization_msgs::msg::Marker::ADD;

  marker.pose = wp.pose;
  marker.pose.position.z += 0.3;

  marker.scale.x = 0.5;  // length
  marker.scale.y = 0.1;  // width
  marker.scale.z = 0.1;  // height

  if (is_skipped) {
    // Red for skipped
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 0.7;
  } else if (is_reached) {
    // White for reached
    marker.color.r = 0.8;
    marker.color.g = 0.8;
    marker.color.b = 0.8;
    marker.color.a = 0.5;
  } else if (is_current) {
    // Green for current target
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
  } else {
    // Blue for pending
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 0.7;
  }

  return marker;
}

std::optional<geometry_msgs::msg::Pose> WaypointFollowerNode::getRobotPose()
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      global_frame_, robot_base_frame_, tf2::TimePointZero);

    geometry_msgs::msg::Pose pose;
    pose.position.x = transform.transform.translation.x;
    pose.position.y = transform.transform.translation.y;
    pose.position.z = transform.transform.translation.z;
    pose.orientation = transform.transform.rotation;

    return pose;
  } catch (const tf2::TransformException& ex) {
    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "Could not get robot pose: %s", ex.what());
    return std::nullopt;
  }
}

void WaypointFollowerNode::transitionToState(NavigationState new_state)
{
  if (current_state_ != new_state) {
    RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s",
                stateToString(current_state_).c_str(),
                stateToString(new_state).c_str());

    // Reset goal_sent_ when entering NAVIGATING state (new waypoint)
    if (new_state == NavigationState::NAVIGATING) {
      goal_sent_ = false;
    }

    current_state_ = new_state;
  }
}

std::string WaypointFollowerNode::stateToString(NavigationState state) const
{
  switch (state) {
    case NavigationState::IDLE: return "IDLE";
    case NavigationState::LOADING_WAYPOINTS: return "LOADING";
    case NavigationState::NAVIGATING: return "NAVIGATING";
    case NavigationState::WAYPOINT_REACHED: return "WAYPOINT_REACHED";
    case NavigationState::WAITING: return "WAITING";
    case NavigationState::EXECUTING_COMMAND: return "EXECUTING_COMMAND";
    case NavigationState::COMPLETED: return "COMPLETED";
    case NavigationState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

void WaypointFollowerNode::startNavigationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ == NavigationState::IDLE ||
      current_state_ == NavigationState::COMPLETED ||
      current_state_ == NavigationState::ERROR) {
    waypoint_manager_->reset();
    paused_ = false;
    transitionToState(NavigationState::LOADING_WAYPOINTS);
    response->success = true;
    response->message = "Navigation started";
  } else {
    response->success = false;
    response->message = "Navigation already running";
  }
}

void WaypointFollowerNode::pauseNavigationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  paused_ = true;
  response->success = true;
  response->message = "Navigation paused";
}

void WaypointFollowerNode::resumeNavigationCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  paused_ = false;

  // If in WAITING state with PAUSE reason, transition to next waypoint
  if (current_state_ == NavigationState::WAITING && wait_reason_ == WaitReason::PAUSE) {
    wait_reason_ = WaitReason::NONE;
    waypoint_manager_->markCurrentReached();

    if (waypoint_manager_->isCompleted()) {
      if (loop_navigation_) {
        RCLCPP_INFO(this->get_logger(), "All waypoints completed, looping back to start");
        waypoint_manager_->reset();
        transitionToState(NavigationState::NAVIGATING);
      } else {
        transitionToState(NavigationState::COMPLETED);
      }
    } else {
      transitionToState(NavigationState::NAVIGATING);
    }

    response->success = true;
    response->message = "Resumed from pause command";
  } else {
    response->success = true;
    response->message = "Navigation resumed";
  }
}

void WaypointFollowerNode::skipWaypointCallback(
  const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  if (current_state_ == NavigationState::NAVIGATING || current_state_ == NavigationState::WAITING) {
    auto current_wp = waypoint_manager_->getCurrentWaypoint();
    if (current_wp.has_value()) {
      // Clean up any wait-related state
      if (current_state_ == NavigationState::WAITING) {
        wait_reason_ = WaitReason::NONE;
        if (wait_topic_sub_) {
          wait_topic_sub_.reset();
        }
      }

      waypoint_manager_->skipCurrentWaypoint();
      response->success = true;
      response->message = "Skipped waypoint " + std::to_string(current_wp->id);

      if (waypoint_manager_->isCompleted()) {
        if (loop_navigation_) {
          RCLCPP_INFO(this->get_logger(), "All waypoints completed, looping back to start");
          waypoint_manager_->reset();
          transitionToState(NavigationState::NAVIGATING);
        } else {
          transitionToState(NavigationState::COMPLETED);
        }
      } else {
        transitionToState(NavigationState::NAVIGATING);
      }
    } else {
      response->success = false;
      response->message = "No current waypoint to skip";
    }
  } else {
    response->success = false;
    response->message = "Not in navigating or waiting state";
  }
}

}  // namespace raspicat_tvvf_navigation

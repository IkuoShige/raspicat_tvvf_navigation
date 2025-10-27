# Waypoint navigation status message

# Current waypoint information
int32 current_waypoint_id
int32 total_waypoints
int32 completed_waypoints
int32 skipped_waypoints

# Current state
string state  # IDLE, LOADING, NAVIGATING, WAITING, COMPLETED, ERROR

# Current waypoint command
string current_command

# Distance to current goal
float64 distance_to_goal

# Orientation difference to current goal (radians)
float64 orientation_diff

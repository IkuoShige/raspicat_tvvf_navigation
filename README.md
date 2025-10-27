# Raspicat TVVF Navigation

TVVF (Time-Varying Vector Field) based navigation and waypoint following system for Raspicat robot with Livox MID-360 LiDAR.

## Quick start: sim

**Option 1: Auto-start (recommended)**
```bash
# Terminal 1: Launch Gazebo
ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py

# Terminal 2: Launch navigation with auto-start
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py \
  use_sim_time:=true \
  auto_start:=true

# Terminal 3: Enable motor (wait for Gazebo to be fully loaded)
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
```

**Option 2: Manual start**
```bash
# Terminal 1: Launch Gazebo
ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py

# Terminal 2: Launch navigation (auto_start is false by default)
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true

# Terminal 3: Enable motor and start navigation manually
ros2 service call /motor_power std_srvs/SetBool '{data: true}'
ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger
```

## Overview

This package provides a **unified, self-contained navigation solution** for the Raspicat robot using:
- **Livox MID-360** 3D LiDAR sensor
- **TVVF-based local planner** (tvvf_vo_c) with differential drive support
- **Complete navigation stack**: map_scan_manager, emcl2, obstacle_tracker
- **Waypoint follower**: Autonomous waypoint navigation

This package is **completely independent** with no dependency on robomaster_s1 packages.

## Architecture

### Simple Two-Component Structure

1. **raspicat_gazebo**: Gazebo environment and robot spawning (simulation only)
2. **raspicat_tvvf_navigation**: Complete navigation + waypoint system (works on both sim and real robot)

### Single Launch File Philosophy

- **One launch file** (`waypoint_navigation.launch.py`) does everything: Navigation + Waypoint
- No need to separate navigation and waypoint - waypoint is always used
- `use_sim_time` parameter switches between simulation and real robot

## Usage

### Simulation

```bash
# Terminal 1: Start Gazebo with Livox-equipped Raspicat
ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py

# Terminal 2: Start Navigation + Waypoint system
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true
```

### Real Robot

```bash
# Single command - no Gazebo needed
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false
```

### Launch Parameters

- `use_sim_time`: Use simulation time (default: false)
  - `true` for Gazebo simulation
  - `false` for real robot
  - Example: `use_sim_time:=true`

- `map_file`: Path to map YAML file (default: maps/maps.yaml)
  - Absolute path to custom map file
  - Example: `map_file:=/path/to/your/map.yaml`

- `waypoint_csv`: Path to waypoint CSV file (default: maps/maps.csv)
  - Absolute path to custom waypoint file
  - Example: `waypoint_csv:=/path/to/your/waypoints.csv`

- `auto_start`: Automatically start waypoint navigation (default: false)
  - `true`: Start navigation immediately after launch
  - `false`: Wait for `/start_waypoint_navigation` service call
  - Example: `auto_start:=true`

- `rviz`: Launch RViz2 for visualization (default: true)
  - Example: `rviz:=false`

**Example with multiple parameters**:
```bash
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py \
  use_sim_time:=true \
  auto_start:=true \
  waypoint_csv:=/home/user/my_waypoints.csv
```

## Waypoint Navigation Control

### Available Services

#### Start/Stop/Control Services
```bash
# Start waypoint navigation
ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger

# Pause navigation (robot stops but keeps current waypoint)
ros2 service call /pause_waypoint_navigation std_srvs/srv/Trigger

# Resume paused navigation
ros2 service call /resume_waypoint_navigation std_srvs/srv/Trigger

# Skip current waypoint and move to next one
ros2 service call /skip_current_waypoint std_srvs/srv/Trigger
```

## Waypoint CSV Format

Define waypoints in a CSV file (`maps/maps.csv`):

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
0,0.0,0.0,0.0,0,0,0,1,
1,2.0,1.0,0.0,0,0,0.7071068,0.7071068,
2,4.0,0.0,0.0,0,0,1,0,
```

**Format Details**:
- **First line must be header**: `id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,`
- **Each subsequent line is a waypoint**:
  - `id` (int): Waypoint ID (0-indexed)
  - `pose_x`, `pose_y`, `pose_z` (double): 3D position in map frame (meters)
  - `rot_x`, `rot_y`, `rot_z`, `rot_w` (double): Orientation as quaternion
  - `command` (string, optional): Special command to execute at this waypoint (see below)
- **Quaternion orientation**: Use online converters or tf transformations to convert from Euler angles
  - Example: yaw=0° → (0,0,0,1), yaw=90° → (0,0,0.7071068,0.7071068), yaw=180° → (0,0,1,0)
- **Empty lines are skipped**, but comments are not supported

**Example with 5 waypoints**:
```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
0,0.0,0.0,0.0,0,0,0,1,
1,1.0,0.0,0.0,0,0,0,1,
2,1.0,1.0,0.0,0,0,0.7071068,0.7071068,
3,0.0,1.0,0.0,0,0,1,0,
4,0.0,0.0,0.0,0,0,-1,0,
```

### Waypoint Command Field

The `command` field allows special behaviors at waypoints (e.g., stopping at crosswalks, waiting for traffic lights).

**Supported Commands**:

1. **Empty or no command** - Normal waypoint, proceed immediately to next
   ```csv
   0,1.0,2.0,0.0,0,0,0,1,
   ```

2. **`wait:N`** - Wait for N seconds, then automatically proceed to next waypoint
   ```csv
   1,5.0,2.0,0.0,0,0,0,1,wait:5.0
   ```
   - Example: Stop at stop line for 5 seconds

3. **`pause`** - Wait until `/resume_waypoint_navigation` service is called
   ```csv
   2,10.0,2.0,0.0,0,0,0,1,pause
   ```
   - Example: Stop at checkpoint, wait for manual confirmation
   - Resume: `ros2 service call /resume_waypoint_navigation std_srvs/srv/Trigger`

4. **`wait_topic:/topic_name`** - Wait until specified Bool topic publishes `true`
   ```csv
   3,10.5,2.0,0.0,0,0,0,1,wait_topic:/crossing_safe
   ```
   - Example: Stop at crosswalk, wait for external node to confirm safety
   - Topic type: `std_msgs/msg/Bool`
   - Automatically proceeds when topic receives `data: true`

**Skip any waiting state**: Use `/skip_current_waypoint` service to force skip current waypoint
```bash
ros2 service call /skip_current_waypoint std_srvs/srv/Trigger
```

**Practical Example - Crosswalk Navigation**:
```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,command,
0,0.0,0.0,0.0,0,0,0,1,
1,5.0,0.0,0.0,0,0,0,1,
2,9.5,0.0,0.0,0,0,0,1,wait:3.0
3,10.0,0.0,0.0,0,0,0,1,wait_topic:/traffic_light_green
4,15.0,0.0,0.0,0,0,0,1,
```
- Waypoint 2: Stop at stop line for 3 seconds
- Waypoint 3: Wait for external traffic light detection node to publish `true` to `/traffic_light_green`
- Waypoint 4: Cross and continue

## Waypoint Follower Parameters

All parameters can be configured in `config/waypoint_follower_params.yaml`:

### Path Configuration
- `waypoint_csv_path` (string): Path to CSV file with waypoints
  - Default: `"maps/maps.csv"`

### Tolerance Settings
- `position_tolerance` (double): How close robot must get to waypoint position (meters)
  - Default: `0.3` meters
- `orientation_tolerance` (double): How close robot orientation must match waypoint (radians)
  - Default: `3.14` radians (allows any orientation)

### Retry and Timeout
- `max_retry_count` (int): Maximum number of retry attempts per waypoint
  - Default: `3`
- `goal_timeout` (double): Maximum time to reach a waypoint before timeout (seconds)
  - Default: `30.0` seconds
- `enable_skip_on_timeout` (bool): Automatically skip waypoint after max retries
  - Default: `true`

### Behavior Settings
- `auto_start` (bool): Automatically start navigation when node launches
  - Default: `false`
  - If `true`, no service call needed
  - If `false`, call `/start_waypoint_navigation` service
- `loop_navigation` (bool): Loop back to first waypoint after completing all
  - Default: `false` (stops after last waypoint)
  - If `true`, repeats waypoint sequence indefinitely

### Frame IDs
- `global_frame` (string): Global/map frame name
  - Default: `"map"`
- `robot_base_frame` (string): Robot base frame name
  - Default: `"base_link"`

## Package Structure

```
raspicat_tvvf_navigation/
├── src/                      # Waypoint follower C++ implementation
│   ├── csv_reader.cpp
│   ├── waypoint_manager.cpp
│   ├── waypoint_follower_node.cpp
│   └── waypoint_follower_main.cpp
├── include/                  # Header files
├── msg/                      # Custom messages
│   └── WaypointStatus.msg
├── launch/                   # Launch file
│   └── waypoint_navigation.launch.py    # Unified launch file
├── config/                   # Configuration files
│   ├── map_scan_params.yaml             # 3D to 2D scan conversion
│   ├── emcl2_params.yaml                # Localization
│   ├── obstacle_tracker_params.yaml     # Obstacle detection
│   ├── tvvf_vo_params.yaml              # Local planner
│   ├── map_regions.yaml                 # Map regions
│   └── waypoint_follower_params.yaml    # Waypoint follower
├── maps/                     # Map and waypoint files
│   ├── maps.yaml
│   ├── maps.pgm
│   └── maps.csv
└── rviz/                     # RViz configuration
```

## Navigation Stack Components

The launch file starts all necessary nodes:

1. **map_scan_manager**: Converts 3D Livox point cloud to 2D laser scans
   - `scan/localization`: High-height scan (0.15m - 2.0m)
   - `scan`: Low-height scan (0.0m - 0.4m) for obstacles
2. **nav2_map_server**: Publishes static map
3. **emcl2**: Monte Carlo localization
4. **obstacle_tracker**: DBSCAN-based obstacle detection
5. **tvvf_vo_c**: TVVF-based local planner (differential drive mode)
6. **waypoint_follower_node**: CSV-based waypoint navigation
7. **RViz2**: Visualization (optional)

## Robot Configuration (Raspicat)

- Robot type: **Differential drive**
- Robot radius: ~0.15 m
- Wheelbase: ~0.16 m
- Max linear velocity: 0.4 m/s
- Max angular velocity: 1.2 rad/s
- Livox MID-360 mount height: ~20 cm from base_link

## Topics

### Subscribed by Navigation Stack
- `/livox/lidar` (sensor_msgs/PointCloud2): 3D point cloud from Livox LiDAR
- `/odom` (nav_msgs/Odometry): Odometry from robot

### Published by Navigation Stack
- `scan` (sensor_msgs/LaserScan): 2D laser scan for obstacle detection
- `scan/localization` (sensor_msgs/LaserScan): 2D laser scan for localization
- `/cmd_vel` (geometry_msgs/Twist): Velocity commands to robot (linear.y = 0 for diff drive)

### Published by Waypoint Follower
- `goal_pose` (geometry_msgs/PoseStamped): Current navigation goal waypoint
- `waypoint_markers` (visualization_msgs/MarkerArray): Waypoint visualization markers for RViz
- `waypoint_status` (raspicat_tvvf_navigation/WaypointStatus): Current waypoint navigation status

### WaypointStatus Message Fields
```
int32 current_waypoint_id      # ID of current waypoint (0-indexed)
int32 total_waypoints           # Total number of waypoints
int32 completed_waypoints       # Number of completed waypoints
int32 skipped_waypoints         # Number of skipped waypoints
string state                    # Current state: IDLE, LOADING, NAVIGATING, WAITING, COMPLETED, ERROR
string current_command          # Current command being executed
float64 distance_to_goal        # Distance to current goal (meters)
float64 orientation_diff        # Orientation difference to goal (radians)
```

Monitor waypoint status in real-time:
```bash
ros2 topic echo /waypoint_status
```

## Key Features

✅ **Unified Launch**: One launch file for complete system
✅ **Sim/Real Flexible**: Same launch works for both with `use_sim_time` parameter
✅ **Complete Independence**: No robomaster_s1 dependencies
✅ **Differential Drive Support**: TVVF planner properly handles non-holonomic constraints
✅ **Livox Integration**: Full 3D LiDAR support with 2D scan generation
✅ **Easy Deployment**: Simple two-step process for simulation, one-step for real robot

## Example Workflow

### Development in Simulation
1. Start Gazebo: `ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py`
2. Wait for robot to spawn and Gazebo to be ready
3. Enable motor: `ros2 service call /motor_power std_srvs/SetBool '{data: true}'`
4. Start navigation: `ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true`
5. Set initial pose in RViz (if needed)
6. Start waypoint navigation: `ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger`
   - Or use `auto_start:=true` in step 4 to skip this step

### Deployment to Real Robot
1. Copy maps and waypoints to robot
2. Launch: `ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false`
3. Set initial pose in RViz
4. Start waypoint navigation: `ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger`
   - Or use `auto_start:=true` in step 2 to skip this step

## Dependencies

### Core
- ROS 2 Humble
- raspicat_description (robot URDF)
- raspicat_gazebo (simulation only)
- ros2_livox_simulation (simulation only)

### Navigation Stack
- tvvf_vo_c
- map_scan_manager
- emcl2
- obstacle_tracker
- nav2_map_server
- nav2_lifecycle_manager

## License

Apache-2.0

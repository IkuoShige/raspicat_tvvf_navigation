# Raspicat TVVF Navigation

TVVF (Time-Varying Vector Field) based navigation and waypoint following system for Raspicat robot with Livox MID-360 LiDAR.

## Quick start: sim

```bash
# colcon build
cd ros2_ws/src/
git clone https://github.com/IkuoShige/raspicat_tvvf_navigation.git -b feat/waypoint_function_without_nav2
git clone https://github.com/CIT-Autonomous-Robot-Lab/raspicat_sim.git -b feat/livox-sim
git clone https://github.com/IkuoShige/tvvf_vo_cpp.git -b feat/waypoint_navigation
git clone https://github.com/IkuoShige/livox_laser_simulation_ros2.git
git clone https://github.com/CIT-Autonomous-Robot-Lab/map_scan_manager.git
git clone ttps://github.com/cafeline/obstacle_tracker.git
git clone https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2.git
cd ../ && colcon build --packages-up-to raspicat_tvvf_navigation --symlink-install
# Terminal 1: Launch Gazebo
ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py

# Terminal 2: Launch navigation
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true

# Terminal 3: Enable motor (wait for Gazebo to be fully loaded)
ros2 service call /motor_power std_srvs/SetBool '{data: true}'

# If auto_start is disabled in config/waypoint_follower_params.yaml:
ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger
```

**Note**: All waypoint navigation parameters (including `auto_start` and `waypoint_csv_path`) are configured in `config/waypoint_follower_params.yaml`. Launch file arguments have been removed in favor of centralized YAML configuration.

## Overview

This package provides a **unified, self-contained navigation solution** for the Raspicat robot using:
- **Livox MID-360** 3D LiDAR sensor (sim: [livox_laser_simulation_ros2](https://github.com/IkuoShige/livox_laser_simulation_ros2))
- **TVVF-based local planner** ([tvvf_vo_cpp](https://github.com/IkuoShige/tvvf_vo_cpp/tree/feat/waypoint_navigation)) with differential drive support
- **Complete navigation stack**: [map_scan_manager](https://github.com/CIT-Autonomous-Robot-Lab/map_scan_manager), [emcl2](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2), [obstacle_tracker](https://github.com/cafeline/obstacle_tracker)
- **Waypoint follower**: Autonomous waypoint navigation with advanced command support
- **Centralized configuration**: All parameters managed through YAML files
- **Waypoint editor integration**: Compatible with [waypoint_editor](https://github.com/kzm784/waypoint_editor) for easy waypoint creation

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

# Terminal 2: Start Navigation + Waypoint system (edit config/waypoint_follower_params.yaml to set `use_sim_time:=true`, `auto_start:=true`)
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py
```

### Real Robot

```bash
# Single command - no Gazebo needed (edit config/waypoint_follower_params.yaml to set `use_sim_time:=false`, `auto_start:=true`)
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false
```

### Launch Parameters

Only minimal launch arguments are supported:

- `use_sim_time`: Use simulation time (default: true)
  - `true` for Gazebo simulation
  - `false` for real robot
  - Example: `use_sim_time:=true`

- `map_file`: Path to map YAML file (default: maps/maps.yaml)
  - Absolute path to custom map file
  - Example: `map_file:=/path/to/your/map.yaml`

- `rviz`: Launch RViz2 for visualization (default: true)
  - Example: `rviz:=false`

**Waypoint and behavior parameters** are configured in `config/waypoint_follower_params.yaml`:
- `waypoint_csv_path`: Path to waypoint CSV file
- `auto_start`: Automatically start navigation on launch (true/false)
- `position_tolerance_strict` / `orientation_tolerance_strict`: Default (strict) reach thresholds
- `position_tolerance_loose` / `orientation_tolerance_loose`: Reach thresholds when `command=loose`
- `loop_navigation`: Loop back to first waypoint after completing all
- Other parameters (see Waypoint Follower Parameters section below)

**Example**:
```bash
# Edit config/waypoint_follower_params.yaml to set waypoint_csv_path and auto_start
ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true
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

Waypoint CSV files should be created using [waypoint_editor](https://github.com/kzm784/waypoint_editor) - a GUI tool for creating and editing waypoint files with map visualization.

Example CSV format (`maps/maps.csv`):

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

1. **`loose`** - このウェイポイントだけ緩いトレランス（`*_loose`）で到達判定
   ```csv
   1,1.0,2.0,0.0,0,0,0,1,loose
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
   3,10.5,2.0,0.0,0,0,0,1,wait_topic:/test_crossing_safe
   ```
   - Example: Stop at crosswalk, wait for external node to confirm safety
   - Topic type: `std_msgs/msg/Bool`
   - Automatically proceeds when topic receives `data: true`
   - Resume: `ros2 topic pub /test_crossing_safe std_msgs/Bool "data: true"`

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

**All parameters must be configured in `config/waypoint_follower_params.yaml`**. Launch file arguments for waypoint settings have been removed in favor of centralized YAML configuration.

### Path Configuration
- `waypoint_csv_path` (string): Absolute path to CSV file with waypoints
  - Default: `"/home/ikuo/docker_play/s1_ws/src/ros2_ws/src/raspicat_tvvf_navigation/maps/maps.csv"`
  - **Important**: Update this path to match your system and waypoint file location
  - Waypoint files should be created using [waypoint_editor](https://github.com/kzm784/waypoint_editor)

### Tolerance Settings
- `position_tolerance_strict` / `orientation_tolerance_strict`: デフォルトで使用する厳しめの到達判定
  - Default: `0.3` m / `0.3` rad
- `position_tolerance_loose` / `orientation_tolerance_loose`: `command=loose` のウェイポイントで使用する緩め判定
  - Default: `0.5` m / `3.14` rad

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

### Configuration Example

Edit `config/waypoint_follower_params.yaml`:

```yaml
waypoint_follower_node:
  ros__parameters:
    # Path to waypoint CSV file (use absolute path)
    waypoint_csv_path: "/home/your_username/your_workspace/waypoints/my_waypoints.csv"

    # Waypoint tolerance
    position_tolerance_strict: 0.3  # meters
    orientation_tolerance_strict: 0.3  # radians
    position_tolerance_loose: 0.5  # meters
    orientation_tolerance_loose: 3.14  # radians

    # Retry and timeout settings
    max_retry_count: 3
    goal_timeout: 30.0  # seconds

    # Behavior settings
    auto_start: true  # Set to true to start navigation immediately
    enable_skip_on_timeout: true
    loop_navigation: false  # Set to true for continuous loop

    # Frame IDs
    global_frame: "map"
    robot_base_frame: "base_link"

    # Use simulation time (set to true for Gazebo)
    use_sim_time: true
```

**Note**: After editing the YAML file, rebuild the package:
```bash
cd /path/to/your/workspace
colcon build --packages-select raspicat_tvvf_navigation --symlink-install
```

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
2. **simple_map_server**: Publishes static map (lightweight, no Nav2 dependency)
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
✅ **No Nav2 Dependency**: Lightweight custom map server, no Nav2 installation required
✅ **Differential Drive Support**: TVVF planner properly handles non-holonomic constraints
✅ **Livox Integration**: Full 3D LiDAR support with 2D scan generation
✅ **Easy Deployment**: Simple two-step process for simulation, one-step for real robot
✅ **Centralized YAML Configuration**: All waypoint and behavior parameters in one config file
✅ **Waypoint Editor Integration**: Compatible with [waypoint_editor](https://github.com/kzm784/waypoint_editor) GUI tool
✅ **Advanced Waypoint Commands**: Support for timed waits, manual pauses, and topic-based triggers

## Example Workflow

### Development in Simulation
1. **Create waypoints** using [waypoint_editor](https://github.com/kzm784/waypoint_editor)
2. **Configure parameters**: Edit `config/waypoint_follower_params.yaml`
   - Set `waypoint_csv_path` to your waypoint file
   - Set `auto_start: true` for automatic start (optional)
   - Set `use_sim_time: true` for Gazebo
3. **Rebuild package**: `colcon build --packages-select raspicat_tvvf_navigation --symlink-install`
4. **Start Gazebo**: `ros2 launch raspicat_gazebo raspicat_gazebo_livox.launch.py`
5. **Wait for robot to spawn** and Gazebo to be ready
6. **Start navigation**: `ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true`
7. **Enable motor**: `ros2 service call /motor_power std_srvs/SetBool '{data: true}'`
8. **Set initial pose** in RViz (if needed)
9. If `auto_start: false`, start manually: `ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger`

### Deployment to Real Robot
1. **Create waypoints** using [waypoint_editor](https://github.com/kzm784/waypoint_editor) on the real map
2. **Configure parameters**: Edit `config/waypoint_follower_params.yaml`
   - Set `waypoint_csv_path` to your waypoint file (use absolute path)
   - Set `auto_start: true` for automatic start (optional)
   - Set `use_sim_time: false` for real robot
3. **Copy package** to robot or build on robot
4. **Launch navigation**: `ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false`
5. **Set initial pose** in RViz
6. If `auto_start: false`, start manually: `ros2 service call /start_waypoint_navigation std_srvs/srv/Trigger`

## Dependencies

### Core
- ROS 2 Humble
- raspicat_description (robot URDF)
- raspicat_gazebo (simulation only)

### Navigation Stack
- tvvf_vo_c
- map_scan_manager
- emcl2
- obstacle_tracker

### System Libraries
- yaml-cpp (map YAML parsing) - `apt install libyaml-cpp-dev`
- OpenCV (map image loading) - typically pre-installed

**Note**: This package does NOT require Nav2 (nav2_map_server, nav2_lifecycle_manager). It uses a lightweight custom map server implementation.

## License

Apache-2.0

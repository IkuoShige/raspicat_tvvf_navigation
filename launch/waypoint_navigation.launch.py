#!/usr/bin/env python3
"""
Raspicat TVVF Waypoint Navigation Launch File

Launches complete navigation stack + waypoint following for Raspicat.
Works with both simulation and real robot.

Usage:
  Simulation: ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=true
  Real Robot: ros2 launch raspicat_tvvf_navigation waypoint_navigation.launch.py use_sim_time:=false
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('raspicat_tvvf_navigation')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map_yaml')
    waypoint_csv = LaunchConfiguration('waypoint_csv')
    regions_config = LaunchConfiguration('regions_config')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    auto_start = LaunchConfiguration('auto_start')

    # Configuration files
    map_scan_config = os.path.join(pkg_dir, 'config', 'map_scan_params.yaml')
    emcl2_config = os.path.join(pkg_dir, 'config', 'emcl2_params.yaml')
    obstacle_tracker_config = os.path.join(pkg_dir, 'config', 'obstacle_tracker_params.yaml')
    tvvf_vo_config = os.path.join(pkg_dir, 'config', 'tvvf_vo_params.yaml')
    waypoint_params = os.path.join(pkg_dir, 'config', 'waypoint_follower_params.yaml')

    # Default resources resolved via package share
    default_map_yaml = os.path.join(pkg_dir, 'maps', 'navigation_map_2_edge_stopline.yaml')
    default_waypoint_csv = os.path.join(pkg_dir, 'maps', 'tsukuba_WP_1.csv')
    default_regions_yaml = os.path.join(pkg_dir, 'config', 'map_regions.yaml')
    default_rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=default_map_yaml,
        description='Occupancy grid YAML file for simple_map_server'
    )

    declare_regions_config = DeclareLaunchArgument(
        'regions_config',
        default_value=default_regions_yaml,
        description='Region configuration file path for map_scan_manager / pointcloud filters'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='RViz configuration file'
    )

    declare_waypoint_csv = DeclareLaunchArgument(
        'waypoint_csv',
        default_value=default_waypoint_csv,
        description='Path to waypoint CSV file'
    )

    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start waypoint navigation on launch (true/false)'
    )

    # Navigation + Waypoint nodes group
    navigation_waypoint_nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),

            # 1. map_scan_manager - 3D LiDAR to 2D LaserScan conversion
            Node(
                package='map_scan_manager',
                executable='map_scan_manager_node',
                name='map_scan_manager',
                output='screen',
                parameters=[
                    map_scan_config,
                    {'map_regions_config_path': regions_config}
                ]
            ),

            # 2. simple_map_server - Publish map for localization and planning
            Node(
                package='raspicat_tvvf_navigation',
                executable='simple_map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'map_yaml_path': map_yaml,
                    'use_sim_time': use_sim_time
                }]
            ),

            # 3. emcl2 - Monte Carlo Localization with expansion resetting
            Node(
                package='emcl2',
                executable='emcl2_node',
                name='emcl2',
                output='screen',
                parameters=[emcl2_config],
                remappings=[
                    ('scan', 'scan/localization')  # Use high-height scan for localization
                ]
            ),

            # 4. obstacle_tracker - Detect and track obstacles using DBSCAN
            Node(
                package='obstacle_tracker',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                output='screen',
                parameters=[obstacle_tracker_config]
                # Uses default 'scan' topic (low-height scan for obstacle detection)
            ),

            # 5. tvvf_vo_cpp - Time-Varying Vector Field local planner
            Node(
                package='tvvf_vo_c',
                executable='tvvf_vo_c_node',
                name='tvvf_vo_c_node',
                output='screen',
                parameters=[tvvf_vo_config]
            ),

            # 6. Waypoint follower node
            Node(
                package='raspicat_tvvf_navigation',
                executable='waypoint_follower_node',
                name='waypoint_follower_node',
                output='screen',
                parameters=[
                    waypoint_params,
                    {
                        'waypoint_csv_path': waypoint_csv,
                        'auto_start': auto_start,
                    }
                ]
            ),

            # 7. RViz2 - Visualization (optional)
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', rviz_config],
                condition=IfCondition(rviz)
            )
        ]
    )

    # Create launch description
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_map_yaml,
        declare_regions_config,
        declare_rviz,
        declare_rviz_config,
        declare_waypoint_csv,
        declare_auto_start,
        # Launch navigation + waypoint nodes
        navigation_waypoint_nodes,
    ])

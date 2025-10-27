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
    map_file = LaunchConfiguration('map_file')
    rviz = LaunchConfiguration('rviz')

    # Configuration files
    map_scan_config = os.path.join(pkg_dir, 'config', 'map_scan_params.yaml')
    map_regions_config = os.path.join(pkg_dir, 'config', 'map_regions.yaml')
    emcl2_config = os.path.join(pkg_dir, 'config', 'emcl2_params.yaml')
    obstacle_tracker_config = os.path.join(pkg_dir, 'config', 'obstacle_tracker_params.yaml')
    tvvf_vo_config = os.path.join(pkg_dir, 'config', 'tvvf_vo_params.yaml')
    waypoint_params = os.path.join(pkg_dir, 'config', 'waypoint_follower_params.yaml')
    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation.rviz')

    # Default files
    default_map_file = os.path.join(pkg_dir, 'maps', 'maps.yaml')

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_map_file = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz2 for visualization'
    )

    # Lifecycle nodes for nav2_lifecycle_manager
    lifecycle_nodes = ['map_server']

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
                    {'map_regions_config_path': map_regions_config}
                ]
            ),

            # 2. nav2_map_server - Publish map for localization and planning
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{'yaml_filename': map_file}]
            ),

            # 3. nav2_lifecycle_manager - Manage lifecycle nodes
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                parameters=[
                    {'autostart': True},
                    {'node_names': lifecycle_nodes}
                ]
            ),

            # 4. emcl2 - Monte Carlo Localization with expansion resetting
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

            # 5. obstacle_tracker - Detect and track obstacles using DBSCAN
            Node(
                package='obstacle_tracker',
                executable='obstacle_tracker',
                name='obstacle_tracker',
                output='screen',
                parameters=[obstacle_tracker_config]
                # Uses default 'scan' topic (low-height scan for obstacle detection)
            ),

            # 6. tvvf_vo_cpp - Time-Varying Vector Field local planner
            Node(
                package='tvvf_vo_c',
                executable='tvvf_vo_c_node',
                name='tvvf_vo_c_node',
                output='screen',
                parameters=[tvvf_vo_config]
            ),

            # 7. Waypoint follower node
            Node(
                package='raspicat_tvvf_navigation',
                executable='waypoint_follower_node',
                name='waypoint_follower_node',
                output='screen',
                parameters=[waypoint_params]
            ),

            # 8. RViz2 - Visualization (optional)
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
        declare_map_file,
        declare_rviz,
        # Launch navigation + waypoint nodes
        navigation_waypoint_nodes,
    ])

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    laser_filter_node = Node(
        package="laser_filters",
        executable="scan_to_scan_filter_chain",
        parameters=[
            PathJoinSubstitution([
                get_package_share_directory("mbzirc_dock"),
                "config", "scan_filter_chain.yaml",
            ])
        ],
        remappings=[
            ('scan', '/usv/slot6/scan'),
            ('scan_filtered', '/usv/slot6/scan_filtered'),
        ]
    )

    server_node = Node(
        package="mbzirc_dock",
        executable="dock_server",
        remappings=[
            ('scan', '/usv/slot6/scan_filtered'),
            ('cmd_vel', '/usv/cmd_vel'),
        ]
    )

    return LaunchDescription([
        laser_filter_node,
        server_node,
    ])

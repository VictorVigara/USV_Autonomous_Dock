from launch import LaunchDescription
from launch_ros.actions import Node
import launch

def generate_launch_description():
    return LaunchDescription([
        launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', '/home/victor/Downloads/rosbag/rosbag2_2023_10_05-12_50_13/rosbag2_2023_10_05-12_50_13_0.mcap', '-s', 'mcap', '--l'],
            output='screen'
        ), 
        Node(
            package='lidar',
            namespace='lidar',
            executable='lidar_test',
            name='lidar_name'
        )
    ])
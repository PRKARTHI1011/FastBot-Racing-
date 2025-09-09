from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share_dir = get_package_share_directory('fastbot_racing')
    
    return LaunchDescription([
        Node(
            package='fastbot_racing',
            executable='path_tracker', # Use our new executable
            name='path_tracker_node',
            output='screen',
            parameters=[
                # This is the key parameter to tune for performance
                {'lookahead_distance': 1.2}, 
                {'linear_speed': 1.0} 
            ]
        ),
    ])
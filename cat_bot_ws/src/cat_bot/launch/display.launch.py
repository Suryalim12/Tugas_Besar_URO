from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', 'urdf/cat_bot.urdf', '-entity', 'cat_bot'],
            output='screen'),
    ])

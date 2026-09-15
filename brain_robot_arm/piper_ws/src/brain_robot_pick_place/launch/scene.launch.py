from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory('brain_robot_pick_place'))
    parameters = str(package_share / 'config' / 'scene.yaml')

    return LaunchDescription([
        Node(
            package='brain_robot_pick_place',
            executable='scene_manager_node.py',
            name='scene_manager',
            output='screen',
            parameters=[parameters],
        ),
        Node(
            package='brain_robot_pick_place',
            executable='cup_follow_node.py',
            name='cup_follow',
            output='screen',
            parameters=[parameters],
        ),
    ])

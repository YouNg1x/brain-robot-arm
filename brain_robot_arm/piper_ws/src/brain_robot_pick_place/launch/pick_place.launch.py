from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder(
        'piper', package_name='piper_with_gripper_moveit').to_moveit_configs()

    return LaunchDescription([
        Node(
            package='brain_robot_pick_place',
            executable='pick_place_node',
            name='pick_place_node',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {
                    'simulation_only': True,
                    'execute': True,
                    'full_sequence': True,
                    'require_zero_state': False,
                    'gripper_tip_offset': 0.1358,
                    'pregrasp_clearance': 0.12,
                    'approach_clearance': 0.06,
                    'grasp_height_offset': 0.0,
                    'lift_clearance': 0.15,
                    'grasp_yaw': 0.0,
                    'place_x': 0.40,
                    'place_y': 0.12,
                    'observation_pause_seconds': 2.0,
                    'trajectory_preview_seconds': 1.0,
                    'use_sim_time': True,
                },
            ],
        ),
    ])

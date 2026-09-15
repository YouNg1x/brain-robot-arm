import os
import re

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def remove_comments(text):
    return re.sub(r'<!--(.*?)-->', '', text, flags=re.DOTALL)


def generate_launch_description():
    start_raw_viewer = LaunchConfiguration('start_raw_viewer')
    package_share = FindPackageShare(
        package='brain_robot_pick_place').find('brain_robot_pick_place')
    xacro_path = os.path.join(
        package_share, 'urdf', 'piper_with_wrist_camera_gazebo.xacro')

    document = xacro.parse(open(xacro_path, encoding='utf-8'))
    xacro.process_doc(document)
    robot_description = {
        'robot_description': remove_comments(document.toxml())
    }

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo', '--verbose',
            '-s', 'libgazebo_ros_init.so',
            '-s', 'libgazebo_ros_factory.so',
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': True},
            robot_description,
            {'publish_frequency': 15.0},
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'piper', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'joint_state_broadcaster',
        ],
        output='screen',
    )
    arm_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'arm_controller',
        ],
        output='screen',
    )
    gripper_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'gripper_controller',
        ],
        output='screen',
    )
    gripper8_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller', '--set-state', 'active',
            'gripper8_controller',
        ],
        output='screen',
    )

    load_controllers_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_controller],
        )
    )
    load_motion_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_controller,
            on_exit=[
                arm_controller,
                gripper_controller,
                gripper8_controller,
            ],
        )
    )

    gripper_mirror = Node(
        package='piper_gazebo',
        executable='joint8_ctrl.py',
        output='screen',
    )
    camera_viewer = Node(
        package='brain_robot_pick_place',
        executable='wrist_camera_viewer.py',
        name='wrist_camera_viewer',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(start_raw_viewer),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'start_raw_viewer',
            default_value='false',
            description='Start the legacy Tkinter raw camera viewer.',
        ),
        load_controllers_after_spawn,
        load_motion_controllers,
        gripper_mirror,
        gazebo,
        robot_state_publisher,
        spawn_robot,
        camera_viewer,
    ])

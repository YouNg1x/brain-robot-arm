import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(path):
    with open(path, encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def generate_launch_description():
    cube_share = get_package_share_directory('brain_robot_ball_pick')
    base_share = get_package_share_directory('brain_robot_pick_place')
    moveit = MoveItConfigsBuilder(
        'piper', package_name='piper_with_gripper_moveit').to_moveit_configs()
    servo_params = {'moveit_servo': _load_yaml(
        os.path.join(base_share, 'config', 'piper_servo_real.yaml'))}
    move_group_params = [
        moveit.to_dict(),
        {
            # This process is only the state/planning-scene authority for the
            # protected visual path.  Physical trajectories remain disabled;
            # piper_jog_adapter is the sole arm-command output path.
            'allow_trajectory_execution': False,
            'publish_robot_description_semantic': True,
            'publish_planning_scene': True,
            'publish_geometry_updates': True,
            'publish_state_updates': True,
            'publish_transforms_updates': True,
            'monitor_dynamics': False,
            'use_sim_time': False,
        },
    ]
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        name='robot_state_publisher', output='screen',
        parameters=[moveit.robot_description, {'use_sim_time': False}],
        remappings=[('/joint_states', '/piper_moveit_joint_states')])
    # Calibrated physical mount: the gripper is 10 cm forward and 5 cm below
    # the camera optical origin; camera axes are parallel to gripper_base.
    camera_mount_tf = Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='piper_camera_mount_tf', output='screen',
        arguments=['0.0', '-0.05', '-0.10', '0.0', '0.0', '0.0',
                   'gripper_base', 'camera_link'])
    return LaunchDescription([
        robot_state_publisher,
        camera_mount_tf,
        Node(package='moveit_ros_move_group', executable='move_group', name='move_group',
             output='screen', parameters=move_group_params,
             remappings=[('/joint_states', '/piper_moveit_joint_states')]),
        Node(package='moveit_servo', executable='servo_node_main', name='servo_node',
             output='screen', parameters=[servo_params, moveit.robot_description,
             moveit.robot_description_semantic, moveit.robot_description_kinematics,
             {'use_sim_time': False}]),
        Node(package='brain_robot_pick_place', executable='piper_jog_adapter.py',
             name='piper_jog_adapter', output='screen', parameters=[{
                 'motion_enabled': False,
                 'input_topic': '/servo_node/delta_joint_cmds',
                 'joint_state_topic': '/joint_states',
                 'filtered_joint_state_topic': '/piper_moveit_joint_states',
                 'output_topic': '/joint_commands',
                 'emergency_stop_topic': '/brain_robot_control/emergency_stop',
                 'max_velocity_rad_s': 0.007,
             }]),
        Node(package='brain_robot_pick_place', executable='visual_search_controller',
             name='visual_search_controller', output='screen', parameters=[
                 moveit.to_dict(),
                 os.path.join(cube_share, 'config', 'cube_task_real.yaml'),
                 {'backend': 'piper', 'use_sim_time': False}]),
        Node(package='brain_robot_pick_place', executable='grasp_lift_executor',
             name='grasp_lift_executor', output='screen', parameters=[
                 moveit.to_dict(),
                 os.path.join(cube_share, 'config', 'cube_task_real.yaml'),
                 {'use_sim_time': False}]),
    ])

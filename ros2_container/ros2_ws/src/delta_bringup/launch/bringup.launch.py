from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='delta_nodes', executable='manipulator_bridge', name='manipulator_bridge',
            parameters=[],
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='mobile_base_bridge', name='mobile_base_bridge',
            parameters=[],
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='camera_target_bridge', name='camera_target_bridge',
            parameters=[],
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='target_transform', name='target_transform',
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='target_tracker', name='target_tracker',
            parameters=[],
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='command_arbiter', name='command_arbiter',
            output='screen',
        ),
        Node(
            package='delta_nodes', executable='frontend_bridge', name='frontend_bridge',
            parameters=[],
            output='screen',
        ),
    ])

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='detection_relay',
            executable='relay_node',
            name='detection_relay',
            output='screen',
            parameters=[{'host': '0.0.0.0', 'port': 9999}],
        ),
        # Add esp32_bridge / mobile_bridge Node(...) entries here once
        # those packages are ported, so one launch file brings up the
        # whole ROS2-side graph.
    ])

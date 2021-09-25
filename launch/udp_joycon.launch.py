from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_joycon',
            executable='joy_node.py',
            remappings=[('udp_joy/data', 'joy')],
            parameters=[{'port': 37501}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[
                {"require_enable_button":   False},
                {"axis_linear.x":           3},
                {"axis_linear.y":           2},
                {"axis_angular.yaw":        0},
                {"scale_linear.x":          0.2},
                {"scale_linear.y":          -0.2},
                {"scale_angular.yaw":       -0.7},
            ],
        )
    ])

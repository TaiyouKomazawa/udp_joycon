import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():
    port_launch_arg = DeclareLaunchArgument(
        'port', default_value=TextSubstitution(text='37501')
    )
    linear_launch_arg = DeclareLaunchArgument(
        'linear_max', default_value=TextSubstitution(text='0.2') #[m/s]
    )
    linear_launch_arg = DeclareLaunchArgument(
        'angular_max', default_value=TextSubstitution(text='0.7') #[rad/s]
    )

    return LaunchDescription([
        port_launch_arg,
        linear_launch_arg,
        Node(
            package='udp_joycon',
            executable='joy_node.py',
            remappings=[('udp_joy/data', 'joy')],
            parameters=[{'port': LaunchConfiguration('port')}],
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            parameters=[
                {'require_enable_button':   False},
                {'axis_linear.x':           3},
                {'axis_linear.y':           2},
                {'axis_angular.yaw':        0},
                {'scale_linear.x':          LaunchConfiguration('linear_max')},
                {'scale_linear.y':          ['-',LaunchConfiguration('linear_max')]},
                {'scale_angular.yaw':       ['-',LaunchConfiguration('angular_max')]},
            ],
        )
    ])

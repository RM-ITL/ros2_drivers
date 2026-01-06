from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('hik_mvs_camera'), 'config', 'camera.yaml'
            ]),
            description='Path to camera parameter file'
        ),
        Node(
            package='hik_mvs_camera',
            executable='mvs_camera_node',
            name='mvs_camera',
            output='screen',
            parameters=[params_file],
            remappings=[('camera_image', '/camera_image')],
        )
    ])

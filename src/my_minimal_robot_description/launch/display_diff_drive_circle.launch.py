from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('my_minimal_robot_description')
    xacro_file = os.path.join(package_share, 'urdf', 'minimal_robot.xacro')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )

    diff_drive_node = Node(
        package='my_turtle_controller',
        executable='diff_drive_circle',
        name='diff_drive_circle',
        parameters=[
            {'wheel_separation': 0.4},
            {'wheel_radius': 0.12},
        ],
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher,
        diff_drive_node,
        rviz
    ])

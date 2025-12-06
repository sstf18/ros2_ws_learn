from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = FindPackageShare('my_turtle_controller')

    # 1. sub launch： turtlesim + spawn turtle2
    turtlesim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'turtlesim_only.launch.py'])
        )
    )

    # 2. sub launch：TF broadcaster + follower
    tf_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share, 'launch', 'tf_system.launch.py'])
        )
    )
    
    # 3. teleop for controll whole system 
    teleop = Node(
        package='turtlesim',
        executable='turtle_teleop_key',
        name='teleop',
        output='screen'
    )

    return LaunchDescription([
        turtlesim_launch,
        tf_system_launch,
        teleop, 
    ])


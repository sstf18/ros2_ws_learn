from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch.substitutions import Command, LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    package_share = get_package_share_directory('my_minimal_robot_description')
    xacro_file = os.path.join(package_share, 'urdf', 'minimal_robot.xacro')
    
    base_height = LaunchConfiguration('base_height')
    sensor_size = LaunchConfiguration('sensor_size')
    
    robot_description_content = ParameterValue(
        Command(['xacro ', xacro_file,
                 ' base_height:=', base_height,
                 ' sensor_size:=', sensor_size,
        ]),
        value_type=str
    )
    
    base_height_arg = DeclareLaunchArgument(
        'base_height',
        default_value='0.2',
        description='Height of the base link'
    )

    sensor_size_arg = DeclareLaunchArgument(
        'sensor_size',
        default_value='0.1',
        description='Size of the sensor cube'
    )

    # robot state publisher 
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        base_height_arg,
        sensor_size_arg,
        robot_state_publisher,
        rviz_node
    ])


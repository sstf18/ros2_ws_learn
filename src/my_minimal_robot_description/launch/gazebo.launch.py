from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share = get_package_share_directory('my_minimal_robot_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'minimal_robot.xacro')

    # 注意：这里不用 ParameterValue，只用 Command 作为“字符串替换”
    robot_description_cmd = Command(['xacro ', xacro_file])

    # 1️⃣ 启动 Ignition Gazebo
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        launch_arguments={
            'gz_args': 'empty.sdf'
        }.items()
    )

    # 2️⃣ robot_state_publisher：based on URDF publish TF
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_cmd}],
        output='screen'
    )

    # 3️⃣ 在 Gazebo 中生成机器人：用 -string 传入 URDF 字符串
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-name', 'minimal_robot',
            '-string', robot_description_cmd,
        ]
    )
    
    # 4 bridge: Gazebo LaserScan -> ROS2 /scan
    bridge_scan = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    
    # 4 diff drive node
    diff_drive_node = Node(
        package='my_turtle_controller',
        executable='simple_diff_drive',
        name='simple_diff_drive',
        parameters=[
            {'wheel_separation': 0.1},
            {'wheel_radius': 0.05},   # 建议跟 URDF 里的 wheel_radius 保持一致
        ],
        output='screen'
    )
    
    # 5 Rviz (in Gazebo's TF/odom envi )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )
    
    return LaunchDescription([
        gazebo,
        rsp,
        spawn,
        bridge_scan,
        diff_drive_node,
        rviz
    ])


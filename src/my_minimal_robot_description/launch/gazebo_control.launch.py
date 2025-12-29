from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('my_minimal_robot_description')
    xacro_file = os.path.join(pkg_share, "urdf", "minimal_robot.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_file]),
        value_type=str
    )

    # 强制软件渲染，减少 Gazebo 在 UTM 里因为 OpenGL 崩溃
    env = SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1')

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}]
    )

    # 启动 Gazebo（用 ign gazebo）
    gazebo = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "empty.sdf"],
        output="screen"
    )

    # 将机器人注入 Gazebo（从 /robot_description topic 读取 URDF）
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "minimal_robot",
            "-topic", "robot_description",
        ],
        output="screen"
    )
    
    return LaunchDescription([
        env,                      
        robot_state_publisher,
        gazebo,
        spawn_entity,
    ])


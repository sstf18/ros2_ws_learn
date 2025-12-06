from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        #1. Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        #2. Spawn turtle2 at a known location
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn',
                'turtlesim/srv/Spawn',
                "{x: 2.0, y: 8.0, theta: 0.0, name: 'turtle2'}"
            ],
            output='screen'
        )
    ])


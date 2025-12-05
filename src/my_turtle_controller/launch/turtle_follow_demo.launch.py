from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    return LaunchDescription([

        # 1. Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
        ),

        # 3. Spawn turtle2
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn',
                'turtlesim/srv/Spawn',
                "{x: 2.0, y: 8.0, theta: 0.0, name: 'turtle2'}",
            ],
            output='screen',
        ),

        # 4. turtle1 broadcaster
        Node(
            package='my_turtle_controller',
            executable='turtle_tf_broadcaster',
            name='turtle1_broadcaster',
        ),

        # 5. turtle2 broadcaster
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_broadcaster',
            name='turtle2_broadcaster',
        ),

        # 6. let turtle2 follow turtle1
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_follow_turtle1',
            name='turtle2_follower',
        ),
    ])



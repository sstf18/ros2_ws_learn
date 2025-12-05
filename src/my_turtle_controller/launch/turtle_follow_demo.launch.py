from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess 
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    linear_gain = LaunchConfiguration('linear_gain')
    angular_gain = LaunchConfiguration('angular_gain')
    
    return LaunchDescription([
        # ===== 1. Launch parameter declare =====
        DeclareLaunchArgument(
            'linear_gain',
            default_value='1.0',
            description='P controller linear gain for follower'
        ),
        DeclareLaunchArgument(
            'angular_gain',
            default_value='6.0',
            description='P controller angular gain for follower'
        ),

        # 2. Start turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        # 3. Spawn turtle2
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/spawn',
                'turtlesim/srv/Spawn',
                "{x: 2.0, y: 8.0, theta: 0.0, name: 'turtle2'}",
            ],
            output='screen'
        ),

        # 4. turtle1 broadcaster
        Node(
            package='my_turtle_controller',
            executable='turtle_tf_broadcaster',
            name='turtle1_broadcaster',
            output='screen'
        ),

        # 5. turtle2 broadcaster
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_broadcaster',
            name='turtle2_broadcaster',
            output='screen'
        ),

        # 6. let turtle2 follow turtle1
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_follow_turtle1',
            name='turtle2_follower',
            parameters=[
                {'linear_gain': linear_gain},
                {'angular_gain': angular_gain},            
            ],
            output='screen'
        ),   
    ])



from launch import LaunchDescription 
from launch_ros.actions import Node 

def generate_launch_description(): 
    return LaunchDescription([
        # --- Turtle1 TF Broadcaster ---
        Node(
            package='my_turtle_controller',
            executable='turtle_tf_broadcaster',
            name='turtle1_broadcaster',
            output='screen'
        ),
        
        # --- Turtle2 TF Broadcaster ---
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_broadcaster',
            name='turtle2_broadcaster',
            output='screen'
        ),
        
        # --- Turtle2 Follow Turtle1 (Follower Node) ---
        Node(
            package='my_turtle_controller',
            executable='turtle2_tf_follow_turtle1',
            name='turtle2_follower',
            output='screen'
        ), 
    ])

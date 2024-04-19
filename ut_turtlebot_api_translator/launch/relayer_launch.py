from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
           package='ut_turtlebot_api_translator',
            namespace='turtlebot_action_relayer',
            executable='turtlebot_action_relayer',
            name='turtlebot_action_relayer',
            respawn=True
            )
    ])

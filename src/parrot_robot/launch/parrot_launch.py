from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Hardware node for direct servo + wings control
        Node(
            package="parrot_robot",
            executable="parrot_node",
            name="parrot_node",
            output="screen"
        ),

        # HTTP bridge node (Flask server exposing REST API)
        Node(
            package="parrot_robot",
            executable="http_server_node",
            name="http_server_node",
            output="screen"
        ),

        # Sound playback node
        Node(
            package="parrot_robot",
            executable="sound_node",
            name="sound_node",
            output="screen"
        ),
    ])
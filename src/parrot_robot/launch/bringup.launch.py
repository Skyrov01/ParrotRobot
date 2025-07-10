from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='parrot_robot', executable='http_server_node'),
        Node(package='parrot_robot', executable='parrot_controller'),
        Node(package='parrot_robot', executable='head_node'),
        Node(package='parrot_robot', executable='wings_node'),
        Node(package='parrot_robot', executable='sound_node'),
        Node(package='parrot_robot', executable='camera_node'),
        Node(package='parrot_robot', executable='yolo_node'),
    ])

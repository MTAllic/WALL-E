from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='llm_talkers',  # Replace with your actual package name
            executable='llm_publisher',  # This should match the entry point for your Python script
            name='llm_publisher',
            output='screen',
        ),
    ])


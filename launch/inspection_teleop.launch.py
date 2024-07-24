import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('inspection_teleop'),
        'config',
        'twist_mux.yaml'
        )
    
    return LaunchDescription([
        # Launch joy_node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{'dev': '/dev/input/js0'}]
        ),
        # Launch joy_to_twist_function
        Node(
            package='inspection_teleop',
            executable='joy_to_twist',
            name='joy_to_twist',
            output='screen',
        ),
        # Launch twist_mux node
        Node(
            package='twist_mux',
            executable='twist_mux',
            name='twist_mux',
            output='screen',
            parameters=[config]  # Adjust the path to your configuration file
        ),

        # Launch twist_to_twist_stamped node
        Node(
            package='inspection_teleop',  # Replace with your package name
            executable='twist_to_twist_stamped',
            name='twist_to_twist_stamped',
            output='screen',
        ),
    ])

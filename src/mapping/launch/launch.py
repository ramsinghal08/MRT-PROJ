from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
     return LaunchDescription([
        Node(
            package='mapping',
            executable='generate',
            name='map_generation'
        ),
        Node(
            package='mapping',
            executable='explore',
            name='exploring'
        ),
     ])


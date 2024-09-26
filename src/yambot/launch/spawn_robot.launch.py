from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '--entity', 'my_robot',
                '--robot_namespace', 'my_robot',
                '--file', '<path_to_your_urdf>',
                '--ros-args', '--param', 'use_sim_time:=true'
            ],
            output='screen',
        ),
    ])

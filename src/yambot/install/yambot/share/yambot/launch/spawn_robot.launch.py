from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Node to launch Gazebo with a built-in world
        Node(
            package='gazebo_ros',
            executable='gzserver',
            arguments=['/usr/share/gazebo/worlds/sunrise.world'],  # Using the built-in sunrise world
            output='screen'
        ),
        
        # Node to spawn your robot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '--entity', 'my_robot',  # Entity name for your robot
                '--robot_namespace', 'my_robot',  # Namespace for the robot
                '--file', '/home/janvi/yambot_ws/src/yambot/urdf/my_robot.urdf',  # Path to your URDF
                '--unpause',  # Optionally unpause the simulation
                '--ros-args', 
                '--param', 'use_sim_time:=true'  # Set simulation time parameter
            ],
            output='screen',
        ),
    ])


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
    
    # The name of the robot should match the name of the robot in the xacro file
    robotXacroName = 'differential_drive_robot'
    
    namePackage = 'yambot'
    
    # Relative path to the xacro file defining the robot
    modelFileRelativePath = 'urdf/mobile_robot.xacro'
    # Relative path to the gazebo world file
    worldFileRelativePath = 'urdf/empty_world.world'
    
    # Absolute path to the model - joins the 2 strings
    pathModelFile = os.path.join(get_package_share_directory(namePackage), modelFileRelativePath)
    
    # Absolute path to the world model
    pathWorldFile = os.path.join(get_package_share_directory(namePackage), worldFileRelativePath)

    # Get the robot description from the xacro model file
    robotDescription = xacro.process_file(pathModelFile).toxml()

    # This is the launch file from the gazebo_ros package
    gazebo_rosPackageLaunch = PythonLaunchDescriptionSource(
        os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
    )
    
    # This is the launch description
    gazeboLaunch = IncludeLaunchDescription(
        gazebo_rosPackageLaunch,
        launch_arguments={'world': pathWorldFile}.items()
    )
    
    # Create the ROS node to spawn the model
    spawnModelNode = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', robotXacroName],
        output='screen'
    )

    # Robot state publisher node
    nodeRobotStatePublisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robotDescription, 'use_sim_time': True}]
    )
       

    # Create an empty launch description object
    launchDescriptionObject = LaunchDescription()

    # Add gazeboLaunch and nodes to the launch description
    launchDescriptionObject.add_action(gazeboLaunch)
    launchDescriptionObject.add_action(spawnModelNode)
    launchDescriptionObject.add_action(nodeRobotStatePublisher)

    return launchDescriptionObject


import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
import xacro

def generate_launch_description():
	
	# the name of the robot should match the name of the robot in the xacro file
	robotXacroName='simple_robot'
	
	namePackage = 'yambot'
	
	# relative path to the xacro file defining the robot
	modelFileRelativePath = 'urdf/test_robot.xacro'
	# relative path to the gazebo world file
	worldFileRelativePath = 'urdf/empty_world.world'
	
	# this is the absolute path to the model - joins the 2 strings
	pathModelFile = os.path.join(get_package_share_directory(namePackage),modelFileRelativePath)
	
	# absolute path to the world model
	pathWorldFile = os.path.join(get_package_share_directory(namePackage),worldFileRelativePath)
	#get the robot description from the xacro model file
	robotDescription = xacro.process_file(pathModelFile).toxml()

	# this is the launch file from the gazebo_ros package
	gazebo_rosPackageLaunch=PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py'))
	
	# this is the launch description
	gazeboLaunch=IncludeLaunchDescription(gazebo_rosPackageLaunch,launch_arguments={'world': pathWorldFile}.items())
	
	# here we create the ros node
	spawnModelNode = Node(package='gazebo_ros', executable='spawn_entity.py', arguments=['-topic', 'robot_description', '-entity', robotXacroName], output='screen')

	# robot state publisher node
	nodeRobotStatePublisher = Node(
		package = 'robot_state_publisher',
		executable = 'robot_state_publisher',
		output = 'screen',
		parameters=[{'robot_description': robotDescription,
		'use_sim_time': True}]
	)

	# here we create an empty launch description object
	launchDescriptionObject = LaunchDescription()

	# we add gazeboLaunch
	launchDescriptionObject.add_action(gazeboLaunch)

	# we add the two nodes
	launchDescriptionObject.add_action(spawnModelNode)
	launchDescriptionObject.add_action(nodeRobotStatePublisher)

	return launchDescriptionObject

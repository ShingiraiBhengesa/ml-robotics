# robot_description.launch.py
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('lynxmotion_pick_place')
    
    # Path to URDF file
    urdf_file = os.path.join(pkg_dir, 'urdf', 'lynxmotion_arm.urdf.xacro')
    
    # Robot description
    robot_description = Command(['xacro ', urdf_file])
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': False
        }]
    )
    
    # Joint state publisher (for visualization - publishes zero joint states)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': False
        }]
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher
    ])

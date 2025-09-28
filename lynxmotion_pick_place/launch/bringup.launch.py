# lynxmotion_pick_place/launch/bringup.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('lynxmotion_pick_place'),
        'config', 'zedm.yaml'
    )

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'),
                         'launch', 'zedm.launch.py')
        ),
        launch_arguments={'param_file': cfg}.items()
    )

    app = Node(
        package='lynxmotion_pick_place',
        executable='demo',   # change if your console_script is named differently
        name='pick_place',
        output='screen'
    )

    return LaunchDescription([zed, app])


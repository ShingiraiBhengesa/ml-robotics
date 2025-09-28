from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    cfg = os.path.join(
        get_package_share_directory('lynxmotion_pick_place'),
        'config', 'zedm.yaml'
    )

    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'),
                         'launch', 'zed_camera.launch.py')
        ),
        launch_arguments={
            'camera_model': 'zedm',
            'param_file': cfg
        }.items()
    )

    app = Node(
        package='lynxmotion_pick_place',
        executable='demo',   # <-- change to your actual executable if different
        name='pick_place',
        output='screen'
    )

    return LaunchDescription([zed, app])

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

    # Robot description launch
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lynxmotion_pick_place'),
                         'launch', 'robot_description.launch.py')
        )
    )

    # ZED camera launch
    zed = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('zed_wrapper'),
                         'launch', 'zedm.launch.py')
        ),
        launch_arguments={'param_file': cfg}.items()
    )

    # Launch YOLOv8 detector (replaces problematic VLM detector)
    yolo_detector = Node(
        package='lynxmotion_pick_place',
        executable='simple_yolo_detector',
        name='yolo_detector', 
        output='screen',
        parameters=[{
            'confidence_threshold': 0.25,
            'topic_rgb': '/zedm/zed_node/left/image_rect_color'
        }]
    )
    
    depth_sampler = Node(
        package='lynxmotion_pick_place',
        executable='depth_sampler_node',
        name='depth_sampler',
        output='screen',
        parameters=[{
            'window': 7,
            'topic_depth': '/zedm/zed_node/depth/depth_registered',
            'topic_info': '/zedm/zed_node/left/camera_info'
        }]
    )
    
    target_selector = Node(
        package='lynxmotion_pick_place',
        executable='target_selector_node',
        name='target_selector',
        output='screen',
        parameters=[{
            'base_frame': 'base_link',
            'camera_frame': 'zed_left_camera_frame'
        }]
    )
    
    task_manager = Node(
        package='lynxmotion_pick_place',
        executable='task_manager_node',
        name='task_manager',
        output='screen'
    )
    
    ik_node = Node(
        package='lynxmotion_pick_place',
        executable='ik_node',
        name='ik_node',
        output='screen',
        parameters=[{
            'grip_angle_deg': 90.0
        }]
    )
    
    arduino_bridge = Node(
        package='lynxmotion_pick_place',
        executable='arduino_bridge_node',
        name='arduino_bridge',
        output='screen',
        parameters=[{
            'port': '/dev/ttyUSB0',  # Change to your Arduino port
            'baud': 115200,
            'move_time_ms': 400,
            'dry_run': True  # Set to False when ready for real hardware
        }]
    )

    return LaunchDescription([
        robot_description,
        zed,
        yolo_detector,
        depth_sampler,
        target_selector,
        task_manager,
        ik_node,
        arduino_bridge
    ])

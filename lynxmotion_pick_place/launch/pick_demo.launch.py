from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_cam',
        arguments=['0.25', '0.00', '0.35', '0', '0', '1.57', 'base_link', 'zed_left_camera_frame']
    )

    vlm = Node(package='lynxmotion_pick_place', executable='vlm_detector_node', name='vlm_detector',
               parameters=[{'model_id':'IDEA-Research/grounding-dino-tiny',
                            'box_threshold':0.35, 'text_threshold':0.25,
                            'topic_rgb':'/zed/left/image_rect_color'}])

    depth = Node(package='lynxmotion_pick_place', executable='depth_sampler_node', name='depth_sampler')
    selector = Node(package='lynxmotion_pick_place', executable='target_selector_node', name='target_selector',
                    parameters=[{'base_frame':'base_link','camera_frame':'zed_left_camera_frame'}])
    ik = Node(package='lynxmotion_pick_place', executable='ik_node', name='ik_node',
              parameters=[{'grip_angle_deg':90.0}])
    arduino = Node(package='lynxmotion_pick_place', executable='arduino_bridge_node', name='arduino_bridge',
                   parameters=[{'port':'COM3','baud':115200,'move_time_ms':400,'dry_run': True}])
    taskman = Node(package='lynxmotion_pick_place', executable='task_manager_node', name='task_manager')

    return LaunchDescription([static_tf, vlm, depth, selector, ik, arduino, taskman])

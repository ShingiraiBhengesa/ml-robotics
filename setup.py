from setuptools import setup
package_name = 'lynxmotion_pick_place'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}.utils'],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', [f'{package_name}/launch/pick_demo.launch.py']),
        (f'share/{package_name}/config', [f'{package_name}/config/arm_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Shingirai Bhengesa',
    author_email='you@example.com',
    description='ZED-M + VLM + IK + Arduino',
    license='MIT',
    entry_points={
        'console_scripts': [
            'vlm_detector_node = lynxmotion_pick_place.vlm_detector_node:main',
            'depth_sampler_node = lynxmotion_pick_place.depth_sampler_node:main',
            'target_selector_node = lynxmotion_pick_place.target_selector_node:main',
            'ik_node = lynxmotion_pick_place.ik_node:main',
            'arduino_bridge_node = lynxmotion_pick_place.arduino_bridge_node:main',
            'task_manager_node = lynxmotion_pick_place.task_manager_node:main',
        ],
    },
)

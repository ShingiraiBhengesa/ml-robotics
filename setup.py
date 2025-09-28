from setuptools import setup, find_packages
from glob import glob
import os

package_name = 'lynxmotion_pick_place'

setup(
    name='lynxmotion-pick-place',   # it's OK if yours uses hyphen; keep it consistent
    version='0.1.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Pick & place stack',
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

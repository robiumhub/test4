from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'spatibot_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), [
            'launch/spatibot_gui_real.launch.py',
            'launch/spatibot_gui_demo.launch.py',
            'launch/robot_api_node.launch.py',
        ]),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'matplotlib',
        'numpy',
        'pillow',
        'google-generativeai',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='mdemir',
    maintainer_email='mdemir@todo.todo',
    description='GUI application for TurtleBot 4 control with docking, movement, and visualization',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'spatibot_gui = spatibot_gui.spatibot_gui_node:main',
            'demo_publisher = spatibot_gui.demo_publisher:main',
            'robot_api_node = spatibot_gui.robot_api_node:main',
            'robot_cli = spatibot_gui.robot_cli:main',
        ],
    },
)

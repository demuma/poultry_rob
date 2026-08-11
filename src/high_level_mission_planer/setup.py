import os
from setuptools import find_packages, setup

package_name = 'high_level_mission_planer'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            [
                'launch/mission_executor.launch.py',
                'launch/simulation.launch.py',
                'launch/robot_mission.launch.py',
            ]),
        (os.path.join('share', package_name, 'config'),
            ['config/mission_executor.yaml']),
        (os.path.join('share', package_name, 'rviz'),
            ['rviz/mission_visualization.rviz']),
        (os.path.join('share', package_name, 'urdf'),
            ['urdf/poultry_robot_visual.urdf']),
        (os.path.join('share', package_name, 'meshes'),
            ['meshes/mid-360-asm.dae']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Timo Lange',
    maintainer_email='timo.lange@haw-hamburg.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_executor = high_level_mission_planer.mission_executor:main',
            'fake_nav2_server = high_level_mission_planer.fake_nav2_server:main',
            'mission_visualizer = high_level_mission_planer.mission_visualizer:main',
            'target_manager = high_level_mission_planer.target_manager:main',
        ],
    },
)

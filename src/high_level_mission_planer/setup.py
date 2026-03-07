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
            ['launch/mission_executor.launch.py']),
        (os.path.join('share', package_name, 'config'),
            ['config/mission_executor.yaml']),
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
            'mission_executor = high_level_mission_planer.mission_executor:main'
        ],
    },
)

from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'tiago_pick_place'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),

        # Install launch files so that:
        # ros2 launch tiago_pick_place task3.launch.py
        # can find them after colcon build.
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
        (
            os.path.join('share', package_name, 'maps'),
            glob('maps/*'),
        ),
    ],

    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='ines',
    maintainer_email='ismesquita@gmail.com',
    description='Task 3 pick and place for TIAGo',
    license='TODO',

    tests_require=['pytest'],

    entry_points={
        'console_scripts': [

            # ROS executable name
            #        ↓
            # Python module                    function
            #        ↓                            ↓
            'cube_tracker = tiago_pick_place.cube_tracker:main',
            'task3_coordinator = tiago_pick_place.task3_coordinator:main',
        ],
    },
)

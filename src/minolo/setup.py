from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'minolo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'params'), glob("params/*.yaml")),
        (os.path.join('share', package_name, 'urdf'), glob("urdf/*.urdf")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='diglo',
    maintainer_email='davidedigloria87@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['minolo_bringup = minolo.main_node:main',
                            'odometry = minolo.diff_odometry:main',
                            'motor_controller = minolo.diff_motor_controller:main',
                            'motor_interface = minolo.hoverboard_interface:main'
        ],
    },
)

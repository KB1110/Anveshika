from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'slam_lidar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        # (os.path.join('share', package_name, 'configuration'), glob('configuration/*.config.lua')),
        ('share/' + package_name + '/configuration/', ['configuration/config.lua'] ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kb',
    maintainer_email='kartikbakshi10@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mapping_node = slam_lidar.mapping_node:main',
            'navigation_node = slam_lidar.navigation_node:main',
            'driving_node = slam_lidar.driving_node:main',
            'imu_node = slam_lidar.imu_node:main',
        ],
    },
)

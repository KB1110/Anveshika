from setuptools import find_packages, setup
import os
import glob

package_name = 'anveshika_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/params/', ['params/realsense.yaml'] )
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
            'detection_node = anveshika_detection.detection_node:main',
        ],
    },
)

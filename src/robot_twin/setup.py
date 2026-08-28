import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_twin'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kwlee',
    maintainer_email='kwlee@todo.todo',
    description='Robot twin — REST gateway over the robot control layer',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_twin = robot_twin.main:main',
        ],
    },
)

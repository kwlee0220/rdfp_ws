import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'rdfp'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*'))),
        (os.path.join('share', package_name, 'dataset', 'sql'),
            glob(os.path.join('rdfp', 'dataset', 'sql', '*.sql'))),
    ],
    install_requires=['setuptools', 'PyTurboJPEG'],
    zip_safe=True,
    maintainer='kwlee',
    maintainer_email='kwlee@todo.todo',
    description='Franka Panda Cartesian path planning and execution using MoveIt2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_twist_teleop = rdfp.teleop.keyboard_twist_teleop:main',
            'session_teleop = rdfp.teleop.session_teleop:main',
            'camera_node = rdfp.camera.camera_node:main',
            'rdfp_camera_node = rdfp.camera.rdfp_camera_node:main',
            'rdfp_image_viewer_node = rdfp.camera.rdfp_image_viewer_node:main',
            'image_viewer_node = rdfp.camera.image_viewer_node:main',
            'ee_pose_node = rdfp.moveit.ee_pose_publisher:main',
            'image_recorder_node = rdfp.recorder.image_recorder_node:main',
            'session_control_node = rdfp.session.session_control_node:main',
            'gripper_control_node = rdfp.moveit.gripper_control_node:main',
            'gripper_command_subscriber = rdfp.moveit.gripper_command_subscriber:main',
            'rdfp_image_recorder = rdfp.recorder.rdfp_image_recorder_node:main',
            'rosbag = rdfp.rosbag.cli:main',
            'import = rdfp.dataset.import_cmd:main',
            'replay = rdfp.dataset.replay_cmd:main',
            'stats = rdfp.dataset.stats_cmd:main',
            'list = rdfp.dataset.list_cmd:main',
            'init-db = rdfp.dataset.init_db_cmd:main',
			'target_joint_states_publisher = rdfp.moveit.target_joint_states_publisher:main',
			'target_joint_states_executor = rdfp.moveit.target_joint_states_executor:main',
			'replay_gui = rdfp.dataset.replay_gui_cmd:main',
        ],
    },
)

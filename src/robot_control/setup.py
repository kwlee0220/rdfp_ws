import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_control'

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
        (os.path.join('share', package_name, 'description'),
            glob(os.path.join('description', '*'))),
        (os.path.join('share', package_name, 'data'),
            glob(os.path.join('data', '*'))),
    ],
    install_requires=['setuptools', 'PyTurboJPEG'],
    zip_safe=True,
    maintainer='kwlee',
    maintainer_email='kwlee@todo.todo',
    description='Robot control layer — MoveIt2 clients, camera, scene state, bringup launches',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # camera
            'camera_node = robot_control.camera.camera_node:main',
            'image_capture_node = robot_control.camera.image_capture:main',
            'image_viewer_node = robot_control.camera.image_viewer_node:main',
            # moveit
            'ee_pose_node = robot_control.moveit.ee_pose_publisher:main',
            'ee_twist_node = robot_control.moveit.ee_twist_publisher:main',
            'servo_auto_start_node = robot_control.moveit.servo_auto_start_node:main',
            'isaac_gripper_bridge = robot_control.isaac.gripper_action_bridge_node:main',
            'isaac_scene_state_node = robot_control.isaac.scene_state_node:main',
            # 백엔드 중립 이름. 노드가 Isaac 전용이 아니라 토픽 remap 으로
            # 어느 스택에나 붙는다 — 펑션베이도 같은 노드를 쓴다.
            'servo_command_bridge = robot_control.isaac.servo_command_bridge_node:main',
            'isaac_servo_bridge = robot_control.isaac.servo_command_bridge_node:main',
            'target_joint_cmds_publisher = robot_control.moveit.target_joint_cmds_publisher:main',
            'target_joint_cmds_executor = robot_control.moveit.target_joint_cmds_executor:main',
            'target_joint_states_publisher = robot_control.moveit.target_joint_states_publisher:main',
            'target_joint_states_executor = robot_control.moveit.target_joint_states_executor:main',
            # gripper — GripperNode 계약 구현 (docs/moveit/GripperNode_Design.md)
            'gripper_action_node = robot_control.gripper.gripper_action_node:main',
            # scene
            'mock_scene_state_node = robot_control.scene.mock_scene_state_node:main',
            # functionbay 백엔드 브리지
            'fb_joint_state_fusion = robot_control.functionbay.joint_state_fusion_node:main',
            'fb_readiness_gate = robot_control.functionbay.readiness_gate_node:main',
        ],
    },
)

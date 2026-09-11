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
            glob(os.path.join('config', '*.*'))),
        # **하위 디렉터리는 위 글롭이 안 담는다.** 빠뜨리면 소스 트리에서는 되고
        # 설치본에서만 프로파일이 비어 조용히 실패한다.
        (os.path.join('share', package_name, 'config', 'backends'),
            glob(os.path.join('config', 'backends', '*'))),
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
            # servo 의 joint_topic 에 컨트롤러 명령 위치를 준다 — 부하 처짐이 servo 명령에
            # 적분되는 래칫을 끊는다 (docs/teleop/servo_vs_planned_motion.md §3)
            'commanded_joint_state_node = robot_control.moveit.commanded_joint_state_node:main',
            'isaac_scene_state_node = robot_control.isaac.scene_state_node:main',
            # 백엔드 중립 이름. 노드가 Isaac 전용이 아니라 토픽 remap 으로
            # 어느 스택에나 붙는다 — 펑션베이도 같은 노드를 쓴다.
            'servo_command_bridge = robot_control.isaac.servo_command_bridge_node:main',
            'target_joint_cmds_publisher = robot_control.moveit.target_joint_cmds_publisher:main',
            'target_joint_cmds_executor = robot_control.moveit.target_joint_cmds_executor:main',
            'target_joint_states_publisher = robot_control.moveit.target_joint_states_publisher:main',
            'target_joint_states_executor = robot_control.moveit.target_joint_states_executor:main',
            # gripper — GripperNode 계약 구현 (docs/gripper/GripperNode_Design.md)
            'gripper_action_node = robot_control.gripper.gripper_action_node:main',
            'robotiq_2f_gripper_node = robot_control.gripper.robotiq_2f_gripper_node:main',
            # scene
            'mock_scene_state_node = robot_control.scene.mock_scene_state_node:main',
            # 이름은 scene 노드 계열(mock_/isaac_)을 따른다 — `fb_` 접두는 브리지 쪽 규칙이다.
            'functionbay_scene_state_node = robot_control.functionbay.scene_state_node:main',
            # functionbay 백엔드 브리지
            'fb_joint_state_fusion = robot_control.functionbay.joint_state_fusion_node:main',
            'fb_readiness_gate = robot_control.functionbay.readiness_gate_node:main',
        ],
    },
)

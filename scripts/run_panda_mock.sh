#!/usr/bin/env bash
# Panda + MoveIt2 **제어 계층** 스택을 mock hardware 로 띄운다.
#
# 뜨는 것: ros2_control / move_group / servo / rviz2 / camera / ee_pose /
#          gripper_control / mock_scene_state
#
# 수집 노드(session_control · image_recorder · target_joint_cmds_publisher)는
# 뜨지 않는다. 세션/에피소드 연산까지 필요하면 수집 계열을 쓴다:
#
#   ros2 launch rdfp rdfp_panda_mock.launch.py
#
# 이 launch 에는 **설정 파일을 바꿔 끼우는 인자가 없다** (config_file 없음).
# 카메라 기본값은 설치본의 robot_control/config/image_pipeline.yaml 에서 오므로,
# 소스 YAML 을 고쳤다면 `colcon build --packages-select robot_control` 이 필요하다.
# 값 하나만 바꿀 때는 인자로 넘긴다 — 전체 목록은 `--show-args` 로 본다.
#
#   ./scripts/run_panda_mock.sh enable_camera_node:=false
#   ./scripts/run_panda_mock.sh camera_resolution:=640x480 enable_scene:=false
#   ./scripts/run_panda_mock.sh --show-args

# ros2 launch robot_control panda_mock.launch.py "$@"
ros2 launch robot_control panda_mock.launch.py enable_camera_node:=false "$@"

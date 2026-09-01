#!/usr/bin/env bash
# Panda + MoveIt2 **제어 계층** 스택을 펑션베이 시뮬레이터 백엔드로 띄운다.
#
# 뜨는 것: joint_state_fusion / readiness_gate / move_group / servo / rviz2 /
#          ee_pose      (ros2_control 컨트롤러는 없다 — 시뮬레이터가 대신한다)
#
# **시뮬레이터(Unity + ros_tcp_endpoint)가 먼저 떠 있어야 한다.** readiness_gate 가
# /output/panda_joint 의 첫 메시지를 기다리며, fb_ready_timeout(기본 60s) 안에 오지
# 않으면 launch 전체가 실패 종료한다.
#
# **도메인/RMW 를 시뮬레이터 쪽과 반드시 일치시킨다.** 어긋나면 게이트가 조용히
# 타임아웃될 뿐 원인이 로그에 드러나지 않는다.
#
#   source ~/development/ros/.ros2rc     # RMW=fastdds, ROS_DOMAIN_ID
#
# 카메라는 시뮬레이터가 /camera_image 로 발행한다. OpenCV 카메라 노드는 기본 off
# 이며(enable_camera_node), 주기·해상도·camera_id 는 인자로 열지 않는다 — 우리가
# 정하는 값이 아니라 시뮬레이터가 주는 대로 쓰기 때문이다. camera_info 는
# 시뮬레이터가 제공하지 않는다.
#
# 수집 노드(session_control · image_recorder · target_joint_cmds_publisher)는 뜨지
# 않는다. 세션/에피소드와 녹화까지 필요하면 수집 계열을 쓴다:
#
#   ros2 launch rdfp rdfp_panda_functionbay.launch.py
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_panda_functionbay.sh
#   ./scripts/run_panda_functionbay.sh camera_image_topic:=/camera_image
#   ./scripts/run_panda_functionbay.sh fb_ready_timeout:=120
#   ./scripts/run_panda_functionbay.sh --show-args

ros2 launch robot_control panda_functionbay.launch.py "$@"

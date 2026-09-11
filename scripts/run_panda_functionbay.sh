#!/usr/bin/env bash
# Panda + MoveIt2 **제어 계층** 스택을 펑션베이 시뮬레이터 백엔드로 띄운다.
#
# 뜨는 것: joint_state_fusion / readiness_gate / move_group / servo / ee_pose /
#          그리퍼 / scene      (ros2_control 컨트롤러는 없다 — 시뮬레이터가 대신한다)
#
# rviz2 와 이미지 뷰어는 **기본 off** 다 — 시뮬레이터가 이미 자기 화면을 그린다:
#   enable_rviz:=true / enable_image_viewer:=true
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
# 카메라는 시뮬레이터가 /camera_image/compressed (CompressedImage, jpeg, 10 Hz) 로
# 발행한다 — 2026-09-08 새 빌드에서 /camera_image (Image) 에서 바뀌었다. 뷰어는
# sensor_msgs/Image 만 구독하므로 enable_image_viewer:=true 로 켜면 압축을 raw 로
# 되살리는 camera_republish 가 함께 뜬다. OpenCV 카메라 노드는 기본 off 이며
# (enable_camera), 주기·해상도·camera_id 는 인자로 열지 않는다 — 우리가 정하는
# 값이 아니라 시뮬레이터가 주는 대로 쓰기 때문이다. camera_info 는 시뮬레이터가
# 제공하지 않는다.
#
# 수집 노드(session_control · image_recorder · target_joint_cmds_publisher)는 뜨지
# 않는다. 세션/에피소드와 녹화까지 필요하면 수집 계열을 쓴다:
#
#   ros2 launch rdfp rdfp_panda_functionbay.launch.py
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_panda_functionbay.sh
#   ./scripts/run_panda_functionbay.sh enable_image_viewer:=true   # 화면에 띄운다
#   ./scripts/run_panda_functionbay.sh fb_ready_timeout:=120
#   ./scripts/run_panda_functionbay.sh --show-args

ros2 launch robot_control panda_functionbay.launch.py "$@"

#!/usr/bin/env bash
# 펑션베이 시뮬레이터 백엔드로 Panda + MoveIt2 스택을 띄운다.
#
# 시뮬레이터(Unity + ros_tcp_endpoint)가 **먼저** 떠 있어야 한다. readiness_gate 가
# /output/panda_joint 의 첫 메시지를 기다리며, fb_ready_timeout(기본 60s) 안에
# 오지 않으면 launch 전체가 실패 종료한다.
#
# 시뮬레이터가 /camera_image 를 직접 발행하므로 자체 카메라 노드(mp4 재생)는 끈다.
# 시뮬레이터 영상을 파이프라인에 태우려면 camera_image_topic:=/camera_image 를 준다
# (단 camera_info 는 시뮬레이터가 제공하지 않는다 — 미해결. 문서 참고).
#
# 도메인/RMW 는 시뮬레이터 쪽과 반드시 일치시킨다. 어긋나면 게이트가 조용히
# 타임아웃될 뿐 원인이 로그에 드러나지 않는다.

ros2 launch robot_control panda_functionbay.launch.py \
  enable_camera_node:=false "$@"

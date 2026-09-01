#!/usr/bin/env bash
# 펑션베이 시뮬레이터 백엔드로 Panda + MoveIt2 스택을 띄운다.
#
# 시뮬레이터(Unity + ros_tcp_endpoint)가 **먼저** 떠 있어야 한다. readiness_gate 가
# /output/panda_joint 의 첫 메시지를 기다리며, fb_ready_timeout(기본 60s) 안에
# 오지 않으면 launch 전체가 실패 종료한다.
#
# 시뮬레이터가 /camera_image 를 직접 발행한다. OpenCV 카메라 노드는 기본 off 이므로
# 따로 끌 필요가 없다. 시뮬레이터 영상을 파이프라인에 태우려면
# camera_image_topic:=/camera_image 를 준다.
#
# 카메라 주기·해상도는 인자로 열지 않는다 — 시뮬레이터가 주는 대로 쓴다. camera_info
# 는 시뮬레이터가 제공하지 않으며, OpenCV 경로에서만 쓴다.
#
# 도메인/RMW 는 시뮬레이터 쪽과 반드시 일치시킨다. 어긋나면 게이트가 조용히
# 타임아웃될 뿐 원인이 로그에 드러나지 않는다.

ros2 launch robot_control panda_functionbay.launch.py "$@"

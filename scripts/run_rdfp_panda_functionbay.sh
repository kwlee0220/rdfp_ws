#!/usr/bin/env bash
# 펑션베이 시뮬레이터 백엔드 + **수집 계층**까지 함께 띄운다.
#
# 제어 계층(run_panda_functionbay.sh 가 띄우는 것) 전부에 더해:
#   session_control / target_joint_cmds_publisher / rdfp_image_viewer /
#   image_recorder
#
# **시뮬레이터(Unity + ros_tcp_endpoint)가 먼저 떠 있어야 한다.** 도메인/RMW 도
# 맞춘다 — 자세한 전제는 run_panda_functionbay.sh 의 주석을 본다.
#
# ⚠️ 레코더 FPS 는 기본 5 인데 **실측 소스는 9.606 Hz** 다 (2026-09-01).
# 레코더가 CFR 이라 선언한 값이 곧 영상의 시간축이 되고 파이프라인에 데시메이션이
# 없으므로, 그대로 쓰면 **영상이 약 1.9배 빠르게 재생된다.**
#
#   · 소스를 5 Hz 로 맞춘 뒤 기본값으로 쓴다        ← 목표 (벤더 요청 B-10)
#   · 그 전까지는 실측값에 맞춰 쓴다               ← 아래 예의 image_recorder_fps
#
# 기본값을 목표값으로 둔 이유는 소스가 맞춰지는 순간 바로 정합하기 위해서다.
# 자세한 근거는 rdfp_panda_functionbay.launch.py 의 docstring 에 있다.
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_rdfp_panda_functionbay.sh image_recorder_fps:=10   # 현재 소스에 맞춤
#   ./scripts/run_rdfp_panda_functionbay.sh image_recorder_auto_start:=true
#   ./scripts/run_rdfp_panda_functionbay.sh enable_image_recorder_node:=false
#   ./scripts/run_rdfp_panda_functionbay.sh --show-args

ros2 launch rdfp rdfp_panda_functionbay.launch.py "$@"

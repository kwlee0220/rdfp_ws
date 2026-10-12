#!/usr/bin/env bash
# 펑션베이 시뮬레이터 백엔드 위에 **수집 계층만** 얹는다.
#
#   session_control / target_joint_cmds_publisher / rdfp_image_viewer / 레코더
#
# **제어 계층을 먼저 띄운다** — `./scripts/run_panda_functionbay.sh`. 묶음 launch
# `rdfp_panda_functionbay` 은 2026-09-13 에 삭제했다.
#
# `backend:=functionbay` 한 인자가 arm 명령 채널과 **압축 카메라 배선**을 프로파일에서
# 가져온다 — `camera.source: compressed` 이므로 레코더가 `input_format=mjpeg` 로
# `/camera_image/compressed` 를 직접 구독한다 (raw 로 되살릴 필요가 없다).
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
# **해상도·FPS 는 백엔드 프로파일이 준다** (640x480 / 5) — 손으로 넘길 필요가 없고,
# 소스가 맞춰지기 전까지만 아래처럼 fps 를 덮는다.
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_rdfp_panda_functionbay.sh image_recorder_fps:=10   # 현재 소스에 맞춤
#   ./scripts/run_rdfp_panda_functionbay.sh enable_image_recorder:=false
#   ./scripts/run_rdfp_panda_functionbay.sh --show-args

ros2 launch rdfp rdfp_collect.launch.py backend:=functionbay "$@"

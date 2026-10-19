#!/usr/bin/env bash
# 펑션베이 시뮬레이터 백엔드 위에 **수집 계층만** 얹는다.
#
#   session_control / target_joint_cmds_publisher / rdfp_image_viewer / 레코더
#
# **제어 계층을 먼저 띄운다** — `./scripts/run_panda_functionbay.sh`. 묶음 launch
# `rdfp_panda_functionbay` 은 2026-09-13 에 삭제했다.
#
# **rosbag 은 띄우지 않는다.** 비영상 채널(관절 · 명령 · 그리퍼 · `/motion/*` …)은 rosbag
# 에서 적재하므로, 학습 데이터를 모으려면 녹화와 적재 설정까지 함께 맞춰야 한다 — 그것을
# 한 번에 하는 것이 `./scripts/run_collect_functionbay.sh` 다. 이 스크립트는 수집 계층만
# 따로 띄워 확인할 때 쓴다.
#
# `backend:=functionbay` 한 인자가 arm 명령 채널과 **압축 카메라 배선**을 프로파일에서
# 가져온다 — `camera.source: compressed` 이므로 세션 레코더가 `input_format=mjpeg` 로
# 압축 토픽을 직접 구독한다 (raw 로 되살릴 필요가 없다). 그 토픽은 시뮬레이터 원본
# `/camera_image/compressed` 가 아니라 **제어 계층 `camera_restamp` 의 출력
# `/camera_image_aligned/compressed`** 다(프로파일 `camera.compressed_topic`) — 원본 stamp 는
# 회차마다 벽시계와 경과시간을 오가므로 관절과 같은 벽시계 축으로 옮긴 것을 받는다.
#
# **시뮬레이터(Unity + ros_tcp_endpoint)가 먼저 떠 있어야 한다.** 도메인/RMW 도
# 맞춘다 — 자세한 전제는 run_panda_functionbay.sh 의 주석을 본다.
#
# **해상도·FPS 는 백엔드 프로파일이 준다** (640x480 / 10) — 손으로 넘길 필요가 없다.
# FPS 10 은 시뮬레이터 카메라 XML 의 `Publish_Frequency` 와 같은 값이다. 레코더는 CFR 이라
# 선언한 값이 곧 영상의 시간축이고, 실측 소스가 9.54~9.60 Hz 라 mp4 시간축은 약 4 %
# 짧다 — 프레임별 실제 시각은 sidecar(`.jsonl`) stamp 로 읽는다. **XML 주기를 바꾸면
# 프로파일 `camera.fps` 를 함께 바꾸고 `robot_control` 을 재빌드한다** (2026-09-17 까지
# 5 로 두었다가 영상이 1.9배 느리게 재생됐다).
#
# ⚠️ **수집 계층 뷰어(`enable_image_viewer`, 기본 켜짐)는 raw `/camera_image` 를 구독한다.**
# 펑션베이는 압축만 발행하므로 그 raw 는 제어 계층의 `camera_republish` 가 되살려야
# 생기는데, 그 노드는 `enable_camera`(또는 `enable_image_viewer`)로 켠다 — 기본은 꺼져 있다.
# 그대로 두면 **창은 뜨는데 영상이 비어 있다**(타입·토픽만 맞으면 오류가 안 난다). 화면이
# 필요하면 제어 계층을 `enable_camera:=true` 로 띄우고, 아니면 여기서 뷰어를 끈다.
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_rdfp_panda_functionbay.sh
#   ./scripts/run_rdfp_panda_functionbay.sh enable_image_viewer:=false   # 헤드리스
#   ./scripts/run_rdfp_panda_functionbay.sh enable_image_recorder:=false
#   ./scripts/run_rdfp_panda_functionbay.sh --show-args

ros2 launch rdfp rdfp_collect.launch.py backend:=functionbay "$@"

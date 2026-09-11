#!/usr/bin/env bash
# Panda + MoveIt2 **제어 계층** 스택을 Isaac Sim 백엔드로 띄운다.
#
# 뜨는 것: readiness_gate → ros2_control_node → joint_state_broadcaster →
#          panda_arm_controller(JTC) → panda_hand_controller → move_group /
#          servo / rviz2 / ee_pose / scene
#
# **mock 과 같은 그림이다.** `controller_manager` 가 ROS 쪽에서 돌고 하드웨어만
# `topic_based_ros2_control/TopicBasedSystem` 이 토픽으로 Isaac 과 말한다. 그래서
# `execute_trajectory()` 가 돌아오고, 트윈·teleop 이 백엔드를 몰라도 된다.
#
# ─── 먼저 할 일 ───────────────────────────────────────────────────────────
#
# **① Isaac 이 떠서 Play 중이어야 한다.**
#
#   ./scripts/run_isaac_sim.sh --gui        # 평소
#   ./scripts/run_isaac_sim.sh --headless   # 검사·CI
#
# readiness_gate 가 `/isaac_joint_states` 의 첫 메시지를 기다린다. isaac_ready_timeout
# (기본 60s) 안에 안 오면 launch 전체가 실패한다. 게이트를 CM 앞에 둔 이유는
# `use_sim_time` 인데 `/clock` 이 없으면 controller_manager 가 시각 0 에서 멈추기
# 때문이다 — 에러 없이 멈춘다.
#
# **② 도메인을 맞춘다. `run_isaac_sim.sh` 는 기본 31 이다.**
#
#   export ROS_DOMAIN_ID=31
#
# 이게 어긋나면 게이트가 조용히 타임아웃할 뿐 원인이 로그에 안 나온다. 증상이
# "Isaac 은 도는데 ROS 가 아무것도 못 본다"뿐이라 제일 먼저 의심할 것이다.
#
# ─── 알아 둘 것 ───────────────────────────────────────────────────────────
#
# **관절 상태 토픽만 갈린다.** Isaac 은 `/isaac_joint_states` 로 비켜서고 `/joint_states`
# 는 `joint_state_broadcaster` 가 갖는다. 안 비키면 `TopicBasedSystem` 이 자기 출력을
# 되읽는 고리가 생긴다.
#
# **타임라인 Stop/Play 와 프로세스 재시작은 다르다.** 전자는 시계가 안 되감겨 스택이
# 스스로 회복하지만, 후자는 시계가 0 으로 돌아가 `TF_OLD_DATA` 가 쏟아지고
# **컨트롤러는 `active` 라고 거짓 보고한다.** 그때는 이 스택을 다시 띄운다:
#
#   ./scripts/kill_stack.sh --ros
#
# 수집 노드(session_control · image_recorder · target_joint_cmds_publisher)는 뜨지
# 않는다. 세션/에피소드와 녹화까지 필요하면 수집 계열을 쓴다:
#
#   ./scripts/run_rdfp_panda_isaac.sh
#
# 사용 예 — 전체 인자는 `--show-args` 로 본다.
#
#   ./scripts/run_panda_isaac.sh
#   ./scripts/run_panda_isaac.sh enable_rviz:=true
#   ./scripts/run_panda_isaac.sh isaac_ready_timeout:=120    # Isaac 기동이 느릴 때
#   ./scripts/run_panda_isaac.sh enable_servo:=false         # teleop 안 쓸 때
#   ./scripts/run_panda_isaac.sh servo_linear_scale:=0.8     # teleop 이동 속도 (아래)
#
# **teleop 이 느리면 servo_linear_scale 이 아니라 teleop 의 linear_step 을 키운다.**
# 실측(2026-09-06) — scale 0.4 는 입력에 비례하고(1.0/0.5/0.25 → 47.5/24.5/11.5 mm/s),
# 0.8 은 최대가 1.2배 느는 대신 비례가 깨져(56.8/39.6/22.9) **미세 조작을 잃는다.**
# 그래서 기본은 원본값 0.4 다.
#   ./scripts/run_panda_isaac.sh --show-args

ros2 launch robot_control panda_isaac.launch.py "$@"

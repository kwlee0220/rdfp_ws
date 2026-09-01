#!/usr/bin/env bash
# 키보드 텔레오퍼레이션.
#
# **백엔드를 `backend` 하나로 고른다.** '/' (ready 자세로 이동) 가 쓰는 arm 명령
# 채널이 백엔드마다 다른데, 기본 `auto` 판별은 `/panda_arm_controller/commands`
# 토픽의 유무만 보므로 **mock 계열에서만 맞는다.**
#
#   ./scripts/run_teleop_keyboard.sh                                  # mock 계열
#   ./scripts/run_teleop_keyboard.sh --ros-args -p backend:=functionbay
#   ./scripts/run_teleop_keyboard.sh --ros-args -p backend:=isaac
#
# ⚠️ 펑션베이·Isaac 에서 프로파일을 빠뜨리면 auto 가 JTC 로 오판하고, 그 스택에는
# FollowJointTrajectory 액션 **서버가 없어** MoveGroup 이 CONTROL_FAILED(-4) 를
# 반환한다 — 계획은 되고 실행만 안 된다. 기동 시 그 조합이면 에러 로그가 남는다.
#
# 개별 값을 덮어쓰려면 함께 준다 (프로파일보다 우선한다):
#
#   ./scripts/run_teleop_keyboard.sh --ros-args \
#     -p backend:=functionbay -p arm_command_topic:=/other/cmd
#
# 세션·에피소드 키(< > [ ] 1-9 0)는 session_control 이 있어야 동작한다. 없으면
# 그 키만 비활성이 되고 나머지는 그대로 쓸 수 있다 (기동 로그에 표시된다).

ros2 run rdfp teleop_keyboard "$@"

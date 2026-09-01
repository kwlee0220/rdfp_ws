#!/usr/bin/env bash
# 키보드 텔레오퍼레이션. 기본값은 **mock 계열** 스택 기준이다.
#
# '/' (ready 자세로 이동) 는 MoveGroup 클라이언트를 쓰는데, 그 종류를 arm 명령
# 채널로 정한다. 기본 `arm_command_mode:=auto` 는 `/panda_arm_controller/commands`
# 토픽의 유무만 보므로 **mock 계열에서만 맞는다.**
#
# ⚠️ 펑션베이·Isaac 처럼 ros2_control 컨트롤러가 없는 스택에서는 auto 가 JTC 로
# 오판하고, 그 스택에는 FollowJointTrajectory 액션 **서버가 없어** 계획만 되고
# 실행이 일어나지 않는다 — '/' 를 눌러도 아무 반응이 없는 것처럼 보인다.
# 기동 시 그 조합이면 에러 로그가 남는다.
#
# 백엔드별로 아래처럼 넘긴다.
#
#   # 펑션베이
#   ./scripts/run_teleop_keyboard.sh \
#     --ros-args -p arm_command_mode:=jgpc \
#                -p arm_command_topic:=/input/panda_joint \
#                -p arm_command_format:=joint_state
#
#   # Isaac
#   ./scripts/run_teleop_keyboard.sh \
#     --ros-args -p arm_command_mode:=jgpc \
#                -p arm_command_topic:=/isaac/arm_command \
#                -p arm_command_format:=joint_state
#
# 세션·에피소드 키(< > [ ] 1-9 0)는 session_control 이 있어야 동작한다. 없으면
# 그 키만 비활성이 되고 나머지는 그대로 쓸 수 있다 (기동 로그에 표시된다).

ros2 run rdfp teleop_keyboard "$@"

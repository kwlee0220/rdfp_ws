#!/usr/bin/env bash
# replay mock 스택 컨테이너 실행 (X11 forwarding).
#
#   ros2 launch rdfp replay_panda_mock.launch.py 를 컨테이너에서 띄운다.
#   RViz2 / image_viewer 는 호스트 X 디스플레이에 표시된다.
#
# 환경변수(선택):
#   ROS_DOMAIN_ID   기본 31 (호스트/다른 컨테이너와 통신하려면 동일하게)
#   GPU=1           NVIDIA GPU 가속(--gpus all). nvidia-container-toolkit 필요.
set -euo pipefail

IMAGE="${IMAGE:-rdfp-replay-mock:latest}"
DOMAIN_ID="${ROS_DOMAIN_ID:-31}"

# X11 접근 허용(로컬 docker). 실패해도 계속 진행.
xhost +local:docker >/dev/null 2>&1 || true

# GPU 는 명시적으로 GPU=1 일 때만 사용한다(nvidia-container-toolkit/CDI 필요).
# 미설정 시 RViz 는 소프트웨어 렌더(llvmpipe)로 동작한다.
GPU_ARGS=()
if [[ "${GPU:-0}" == "1" ]]; then
  GPU_ARGS=(--gpus all)
fi

# launch 인자만 준 경우 기본 launch 뒤에 붙인다.
if [[ $# -gt 0 && "$1" != ros2 && "$1" != /* && "$1" != bash ]]; then
  set -- ros2 launch rdfp replay_panda_mock.launch.py "$@"
elif [[ $# -eq 0 ]]; then
  set -- ros2 launch rdfp replay_panda_mock.launch.py
fi

exec docker run --rm -it \
  --name rdfp-replay-mock \
  --network host \
  "${GPU_ARGS[@]}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
  "${IMAGE}" \
  "$@"

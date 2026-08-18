#!/usr/bin/env bash
# Gazebo(Fortress) 스택 컨테이너 실행 (X11 forwarding).
#
#   기본: ros2 launch rdfp rdfp_panda_gazebo.launch.py (풀 앱 gazebo)
#   추가 launch 인자는 그대로 뒤에 붙는다:
#     ./docker/run_gazebo.sh enable_rviz:=true simulate_camera:=true
#   백엔드만(앱 노드 없이) 띄우려면 전체 명령을 넘긴다:
#     ./docker/run_gazebo.sh ros2 launch rdfp panda_gazebo.launch.py
#
# 환경변수(선택):
#   ROS_DOMAIN_ID   기본 31 (호스트/다른 컨테이너와 통신하려면 동일하게)
#   GPU=1           NVIDIA GPU 가속(--gpus all). nvidia-container-toolkit/CDI 필요.
#
# GL 렌더: Intel/AMD 는 /dev/dri 를 자동 패스스루(하드웨어 가속), NVIDIA 는 GPU=1.
# 둘 다 없으면 소프트웨어 렌더(llvmpipe)로 동작한다(느림).
set -euo pipefail

IMAGE="${IMAGE:-rdfp-gazebo:latest}"
DOMAIN_ID="${ROS_DOMAIN_ID:-31}"

xhost +local:docker >/dev/null 2>&1 || true

GPU_ARGS=()
if [[ "${GPU:-0}" == "1" ]]; then
  GPU_ARGS=(--gpus all)
fi

# Intel/AMD GL 가속용 DRI 패스스루(있을 때만).
DRI_ARGS=()
if [[ -d /dev/dri ]]; then
  DRI_ARGS=(--device /dev/dri)
fi

# 첫 인자가 실행 파일이 아니면(= launch 인자만 준 경우) 기본 launch 뒤에 붙인다.
if [[ $# -gt 0 && "$1" != ros2 && "$1" != /* && "$1" != bash ]]; then
  set -- ros2 launch rdfp rdfp_panda_gazebo.launch.py "$@"
elif [[ $# -eq 0 ]]; then
  set -- ros2 launch rdfp rdfp_panda_gazebo.launch.py
fi

exec docker run --rm -it \
  --name rdfp-gazebo \
  --network host \
  "${GPU_ARGS[@]}" "${DRI_ARGS[@]}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
  "${IMAGE}" \
  "$@"

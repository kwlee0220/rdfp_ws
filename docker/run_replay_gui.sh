#!/usr/bin/env bash
# replay GUI 컨테이너 실행 (X11 forwarding).
#
#   ros2 run rdfp replay_gui (Tk GUI)를 컨테이너에서 띄운다.
#   실행 중인 replay mock 스택(MoveIt)과 ROS 로 통신하며, PostgreSQL + MP4 에서
#   에피소드를 읽어 재생한다.
#
# 필수/선택 환경변수:
#   RDFP_DB_DSN      PostgreSQL DSN (필수). 예: postgresql://user:pw@localhost:5432/rdfp
#                    --network host 이므로 호스트의 localhost DB 에 그대로 접속된다.
#   RDFP_DATA_DIR    MP4/데이터 루트 (기본 /data/rdfp). DB 에 저장된 파일 경로와
#                    동일하게 보이도록 같은 경로로 마운트한다.
#   ROS_DOMAIN_ID    기본 31 (replay mock 컨테이너와 동일해야 통신됨).
#   GPU=1            NVIDIA GPU 가속(--gpus all).
set -euo pipefail

IMAGE="${IMAGE:-rdfp-replay-gui:latest}"
DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
DATA_DIR="${RDFP_DATA_DIR:-/data/rdfp}"

if [[ -z "${RDFP_DB_DSN:-}" ]]; then
  echo "[run_replay_gui] WARNING: RDFP_DB_DSN 이 설정되지 않았습니다. DB 접속이 실패할 수 있습니다." >&2
fi

xhost +local:docker >/dev/null 2>&1 || true

# GPU 는 명시적으로 GPU=1 일 때만 사용한다(nvidia-container-toolkit/CDI 필요).
GPU_ARGS=()
if [[ "${GPU:-0}" == "1" ]]; then
  GPU_ARGS=(--gpus all)
fi

DATA_ARGS=()
if [[ -d "${DATA_DIR}" ]]; then
  DATA_ARGS=(-v "${DATA_DIR}:${DATA_DIR}:rw")
fi

# 인자만 준 경우(예: --config foo.yaml) 기본 명령 뒤에 붙인다.
if [[ $# -gt 0 && "$1" != ros2 && "$1" != /* && "$1" != bash ]]; then
  set -- ros2 run rdfp replay_gui "$@"
elif [[ $# -eq 0 ]]; then
  set -- ros2 run rdfp replay_gui
fi

exec docker run --rm -it \
  --name rdfp-replay-gui \
  --network host \
  "${GPU_ARGS[@]}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e RDFP_DB_DSN="${RDFP_DB_DSN:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
  "${DATA_ARGS[@]}" \
  "${IMAGE}" \
  "$@"

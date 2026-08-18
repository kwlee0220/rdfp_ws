#!/usr/bin/env bash
# panda JGPC mock 풀 앱 스택 컨테이너 실행 (X11 forwarding).
#
#   ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py 를 컨테이너에서 띄운다.
#   RViz2 / image_viewer 는 호스트 X 디스플레이에 표시된다.
#
# panda_mock 과 달리 arm 컨트롤러가 JointGroupPositionController 라
# move_group 의 arm plan & execute 가 동작하지 않는다. 궤적 실행은
# /panda_arm_controller/commands (std_msgs/Float64MultiArray) 스트리밍으로
# 수행한다 — create_move_group_client() 가 돌려주는 MoveGroupJgpcClient 참고.
#
# rdfp_panda_mock 과 달리 target_joint_states_publisher 는 포함되지 않으므로
# /target_joint_states 는 발행되지 않는다.
#
# 환경변수(선택):
#   ROS_DOMAIN_ID     기본 31
#   GPU=1             NVIDIA GPU 가속(--gpus all)
#   RDFP_REC_DIR      image_recorder MP4 출력 디렉터리 (기본 /tmp/recordings, 볼륨 마운트)
#   RDFP_CONFIG_DIR   지정 시 해당 호스트 디렉터리를 컨테이너 /mnt/config 로 마운트하고,
#                     그 안의 YAML 을 자동 사용한다(이미지에 구워진 기본 config 대신
#                     호스트 파일 사용). 설정은 두 파일로 나뉘며 각각 독립적으로 주입된다.
#                       panda_robot.yaml      -> config_file
#                       image_pipeline.yaml   -> image_pipeline_config_file
#                     해당 인자를 직접 준 경우 그 파일의 자동 주입은 생략한다.
#                     두 YAML 모두 rdfp_panda_mock.launch.py 와 그대로 공유한다.
#                     예: RDFP_CONFIG_DIR=$PWD/src/rdfp/config ./docker/run_panda_jgpc_mock.sh
#
# 웹캠이 없거나 카메라를 끄고 싶으면 launch 인자를 그대로 넘긴다:
#   ./docker/run_panda_jgpc_mock.sh enable_camera_node:=false
#   ./docker/run_panda_jgpc_mock.sh log_level:=debug
set -euo pipefail

IMAGE="${IMAGE:-rdfp-panda-jgpc-mock:latest}"
DOMAIN_ID="${ROS_DOMAIN_ID:-31}"
REC_DIR="${RDFP_REC_DIR:-/tmp/recordings}"

xhost +local:docker >/dev/null 2>&1 || true
mkdir -p "${REC_DIR}"

# GPU 는 명시적으로 GPU=1 일 때만 사용한다(nvidia-container-toolkit/CDI 필요).
# 미설정 시 RViz 는 소프트웨어 렌더(llvmpipe)로 동작한다.
GPU_ARGS=()
if [[ "${GPU:-0}" == "1" ]]; then
  GPU_ARGS=(--gpus all)
fi

# 존재하는 /dev/video* 를 컨테이너에 패스스루(웹캠). 없으면 건너뛴다.
CAM_ARGS=()
shopt -s nullglob
for dev in /dev/video*; do
  CAM_ARGS+=(--device "${dev}")
done
shopt -u nullglob

# config 디렉터리 볼륨 마운트(선택). RDFP_CONFIG_DIR 를 지정하면 그 호스트
# 디렉터리를 컨테이너 /mnt/config 로 read-only 마운트한다. docker -v 는
# 절대경로를 요구하므로 상대경로도 정규화한다.
CONFIG_MOUNT_TARGET="/mnt/config"
CONFIG_ARGS=()
CONFIG_DIR="${RDFP_CONFIG_DIR:-}"
if [[ -n "${CONFIG_DIR}" ]]; then
  if [[ ! -d "${CONFIG_DIR}" ]]; then
    echo "[ERROR] RDFP_CONFIG_DIR is not a directory: ${CONFIG_DIR}" >&2
    exit 1
  fi
  CONFIG_DIR="$(cd "${CONFIG_DIR}" && pwd)"
  CONFIG_ARGS=(-v "${CONFIG_DIR}:${CONFIG_MOUNT_TARGET}:ro")
fi

# 설정은 로봇 YAML 과 이미지 파이프라인 YAML 둘로 나뉜다. 각각 독립적으로 판정해
# 주입한다 — 한쪽만 주입하면 나머지는 이미지에 구워진 기본값을 읽어, 호스트 파일을
# 고쳐도 반영되지 않는 반쪽 상태가 된다.
#
#   $1 = launch argument 이름, 나머지 = 그 디렉터리에서 찾을 파일 이름 후보
#        (앞에서부터 먼저 존재하는 것을 쓴다 — 구 이름 하위호환용)
# 사용자가 그 argument 를 직접 준 경우 자동 주입을 생략한다.
AUTO_CONFIG_ARGS=()
add_auto_config() {
  local arg_name="$1"; shift
  local arg file_name
  for arg in "${USER_ARGS[@]}"; do
    [[ "${arg}" == "${arg_name}:="* ]] && return 0
  done
  [[ -n "${CONFIG_DIR}" ]] || return 0
  for file_name in "$@"; do
    [[ -f "${CONFIG_DIR}/${file_name}" ]] || continue
    if [[ "${file_name}" != "$1" ]]; then
      echo "[WARN] using legacy config name '${file_name}'; rename it to '$1'" >&2
    fi
    AUTO_CONFIG_ARGS+=("${arg_name}:=${CONFIG_MOUNT_TARGET}/${file_name}")
    return 0
  done
}

USER_ARGS=("$@")
# rdfp_panda_mock.yaml 은 panda_robot.yaml 의 구 이름이다. 마운트 디렉터리에 구
# 이름만 있는 사용자를 위해 폴백하되 경고를 남긴다.
add_auto_config config_file panda_robot.yaml rdfp_panda_mock.yaml
add_auto_config image_pipeline_config_file image_pipeline.yaml

# launch 인자만 준 경우(예: enable_camera_node:=false) 기본 launch 뒤에 붙인다.
if [[ $# -gt 0 && "$1" != ros2 && "$1" != /* && "$1" != bash ]]; then
  set -- ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py "$@" "${AUTO_CONFIG_ARGS[@]}"
elif [[ $# -eq 0 ]]; then
  set -- ros2 launch rdfp rdfp_panda_jgpc_mock.launch.py "${AUTO_CONFIG_ARGS[@]}"
fi

exec docker run --rm -it \
  --name rdfp-panda-jgpc-mock \
  --network host \
  "${GPU_ARGS[@]}" \
  "${CAM_ARGS[@]}" \
  "${CONFIG_ARGS[@]}" \
  -e DISPLAY="${DISPLAY}" \
  -e QT_X11_NO_MITSHM=1 \
  -e ROS_DOMAIN_ID="${DOMAIN_ID}" \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${HOME}/.Xauthority:/root/.Xauthority:rw" \
  -v "${REC_DIR}:${REC_DIR}:rw" \
  "${IMAGE}" \
  "$@"

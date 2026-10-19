#!/usr/bin/env bash
# rosbag2 녹화의 구현이다. 사용자 진입점은 `scripts/rosbag.sh record` 이며, 그쪽이
# 옵션을 아래 환경변수로 바꿔 `exec` 로 이 스크립트에 넘긴다.
#
#   RECORDING_TOPICS_FILE  녹화할 토픽 목록 (기본: config/recording_topics.list)
#   RDFP_ROSBAG_DIR        저장 루트 — **필수**, 기본값을 두지 않는다
#   SPLIT_DURATION_SEC     분할 간격(초, 기본 3600)
#   STORAGE_ID             저장 플러그인 (기본 mcap — 적재는 mcap 만 읽는다)
#
# 직접 실행해도 되지만 설정은 환경변수로만 받는다. 명령줄 인자는 거부한다 —
# 예전에는 조용히 무시해서 `-d 600` 을 붙여도 분할 간격이 그대로였다.
set -euo pipefail

if [ "$#" -gt 0 ]; then
  echo "[ERROR] record_rosbag.sh takes no arguments (got: $*)" >&2
  echo "[ERROR] Use: scripts/rosbag.sh record [--topics-file F] [--rosbag-dir D] [--split-duration S]" >&2
  exit 2
fi

########################################
# Configuration
########################################
# 저장 루트는 기본값을 두지 않는다. 이 스크립트(`~/rdfp_data/rosbag`)와 적재 설정
# (`dataset_config.yaml` 의 `${RDFP_ROSBAG_DIR:-$HOME/rosbag}`)이 서로 다른 기본값을
# 갖고 있어서, 환경변수 없이 녹화하면 적재가 녹화 결과를 조용히 못 찾았다.
RDFP_ROSBAG_DIR="${RDFP_ROSBAG_DIR:-}"
if [ -z "${RDFP_ROSBAG_DIR}" ]; then
  echo "[ERROR] RDFP_ROSBAG_DIR is not set. Run 'rdfp_env', or pass --rosbag-dir to scripts/rosbag.sh record." >&2
  exit 1
fi
SPLIT_DURATION_SEC="${SPLIT_DURATION_SEC:-3600}"
STORAGE_ID="${STORAGE_ID:-mcap}"

if ! [[ "${SPLIT_DURATION_SEC}" =~ ^[1-9][0-9]*$ ]]; then
  echo "[ERROR] SPLIT_DURATION_SEC must be a positive integer (seconds): '${SPLIT_DURATION_SEC}'" >&2
  exit 1
fi

# 저장 대상 토픽은 외부 텍스트 파일에서 읽는다. dataset_config.yaml 의
# `topics_file:` 와 동일한 파일을 공유한다.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RECORDING_TOPICS_FILE="${RECORDING_TOPICS_FILE:-${SCRIPT_DIR}/../config/recording_topics.list}"

if [ ! -f "${RECORDING_TOPICS_FILE}" ]; then
  echo "[ERROR] Recording topics file not found: ${RECORDING_TOPICS_FILE}" >&2
  exit 1
fi

# 앞뒤 공백을 모두 지운다. 적재 쪽(`rdfp.dataset.config._read_topics_file`)이 `strip()`
# 으로 읽으므로 같은 규칙이어야 두 쪽이 같은 토픽 이름을 본다.
mapfile -t TOPICS < <(grep -vE '^[[:space:]]*(#|$)' "${RECORDING_TOPICS_FILE}" \
                      | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')

if [ "${#TOPICS[@]}" -eq 0 ]; then
  echo "[ERROR] No topics declared in ${RECORDING_TOPICS_FILE}" >&2
  exit 1
fi


########################################
# Preconditions
########################################
if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Source ROS 2 environment first." >&2
  exit 1
fi

# 워크스페이스 overlay 가 없으면 rosbag2 는 `rdfp_msgs` 타입을 몰라 `/session` 등을
# **0 건**으로 기록하면서도 멈추지 않는다 — `has unknown type` 경고만 반복한다
# (2026-10-17 실측). `/session` 없이는 적재가 에피소드를 자를 수 없으므로 미리 막는다.
if ! ros2 pkg prefix rdfp_msgs >/dev/null 2>&1; then
  echo "[ERROR] Package 'rdfp_msgs' not found. Source the workspace overlay first ('rdfp_env' or 'source install/setup.bash')." >&2
  echo "[ERROR] Without it, rdfp_msgs topics such as /session are recorded with 0 messages." >&2
  exit 1
fi

# MCAP 저장소 사용 시 관련 패키지가 설치되어 있어야 한다.
if [ "${STORAGE_ID}" = "mcap" ] && ! dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
  echo "[ERROR] STORAGE_ID=mcap requires 'ros-humble-rosbag2-storage-mcap' package." >&2
  echo "[ERROR] Install with: sudo apt install ros-humble-rosbag2-storage-mcap" >&2
  exit 1
fi

# 세션 디렉터리 이름은 적재 쪽 탐색 규칙(`rdfp.rosbag.catalog.SESSION_DIR_RE`,
# `session_YYYY-MM-DD_HH-MM-SS`)을 벗어나면 안 된다 — 벗어난 디렉터리는 적재가 조용히
# 건너뛴다. 그래서 같은 초에 다시 시작해도 접미사(`_1`)를 붙이지 않고 다음 초까지
# 기다려 새 이름을 만든다. 동시에 둘을 띄우는 것은 지원하지 않는다 — 디렉터리는
# rosbag2 가 만들므로 이름을 미리 잡아 둘 수 없다.
while :; do
  RUN_TS="$(date +%F_%H-%M-%S)"
  DAY_DIR="${RDFP_ROSBAG_DIR}/${RUN_TS%%_*}"
  OUT_DIR="${DAY_DIR}/session_${RUN_TS}"
  [ -e "${OUT_DIR}" ] || break
  sleep 1
done

mkdir -p "${DAY_DIR}"

########################################
# Disk check (optional but recommended)
########################################
if command -v df >/dev/null 2>&1; then
  AVAIL_KB="$(df -Pk "${RDFP_ROSBAG_DIR}" 2>/dev/null | awk 'NR==2 {print $4}')"
  if [ -n "${AVAIL_KB:-}" ] && [ "${AVAIL_KB}" -lt 10485760 ]; then
    echo "[ERROR] Less than 10 GB free space available under ${RDFP_ROSBAG_DIR}" >&2
    exit 1
  fi
fi

########################################
# Logging
########################################
echo "[INFO] Starting rosbag2 recording"
echo "[INFO] Base dir           : ${RDFP_ROSBAG_DIR}"
echo "[INFO] Day dir            : ${DAY_DIR}"
echo "[INFO] Session dir        : ${OUT_DIR}"
echo "[INFO] Storage            : ${STORAGE_ID}"
echo "[INFO] Split duration     : ${SPLIT_DURATION_SEC} sec"
echo "[INFO] Topics file        : ${RECORDING_TOPICS_FILE}"
echo "[INFO] Topic count        : ${#TOPICS[@]}"
echo "[INFO] Topics             :"
printf '[INFO]   - %s\n' "${TOPICS[@]}"

########################################
# Start recording
########################################
# `exec` 로 자리를 넘겨 녹화 프로세스가 이 스크립트의 PID 를 이어받게 한다 —
# 그래야 그 PID 에 보낸 SIGINT 가 rosbag2 에 닿아 `metadata.yaml` 이 남는다.
exec ros2 bag record \
  -s "${STORAGE_ID}" \
  -o "${OUT_DIR}" \
  -d "${SPLIT_DURATION_SEC}" \
  "${TOPICS[@]}"

#!/usr/bin/env bash
set -euo pipefail

########################################
# Configuration
########################################
BASE_DIR="${BASE_DIR:-/data/rdfp/rosbag}"
SPLIT_DURATION_SEC="${SPLIT_DURATION_SEC:-3600}"
STORAGE_ID="${STORAGE_ID:-mcap}"

# 저장 대상 토픽
TOPICS=(
  /session

  # MoveIt arm관련 토픽 (joint 좌표)
  /servo_node/delta_twist_cmds
  /ee_pose

  # MoveIt arm관련 토픽 (joint delta)
  /servo_node/delta_joint_cmds

  # MoveIt arm관련 토픽 (end-effector pose)
  /target_joint_states
  /joint_states

  # MoveIt hand관련 토픽
  /gripper_control/gripper_cmds
  /gripper_control/gripper_states

  # Camera 관련 토픽
  /camera/image_raw
  /camera/image_compressed
)
# /servo_node/delta_joint_cmds


########################################
# Preconditions
########################################
if ! command -v ros2 >/dev/null 2>&1; then
  echo "[ERROR] ros2 command not found. Source ROS 2 environment first." >&2
  exit 1
fi

# MCAP 저장소 사용 시 관련 패키지가 설치되어 있어야 한다.
if [ "${STORAGE_ID}" = "mcap" ] && ! dpkg -s ros-humble-rosbag2-storage-mcap >/dev/null 2>&1; then
  echo "[ERROR] STORAGE_ID=mcap requires 'ros-humble-rosbag2-storage-mcap' package." >&2
  echo "[ERROR] Install with: sudo apt install ros-humble-rosbag2-storage-mcap" >&2
  exit 1
fi

RUN_TS="$(date +%F_%H-%M-%S)"
DAY_DIR="${BASE_DIR}/${RUN_TS%%_*}"
SESSION_NAME="session_${RUN_TS}"
OUT_DIR="${DAY_DIR}/${SESSION_NAME}"
SUFFIX=1

mkdir -p "${DAY_DIR}"

while [ -e "${OUT_DIR}" ]; do
  OUT_DIR="${DAY_DIR}/${SESSION_NAME}_${SUFFIX}"
  SUFFIX=$((SUFFIX + 1))
done

########################################
# Disk check (optional but recommended)
########################################
if command -v df >/dev/null 2>&1; then
  AVAIL_KB="$(df -Pk "${BASE_DIR}" 2>/dev/null | awk 'NR==2 {print $4}')"
  if [ -n "${AVAIL_KB:-}" ] && [ "${AVAIL_KB}" -lt 10485760 ]; then
    echo "[ERROR] Less than 10 GB free space available under ${BASE_DIR}" >&2
    exit 1
  fi
fi

########################################
# Logging
########################################
echo "[INFO] Starting rosbag2 recording"
echo "[INFO] Base dir           : ${BASE_DIR}"
echo "[INFO] Day dir            : ${DAY_DIR}"
echo "[INFO] Session dir        : ${OUT_DIR}"
echo "[INFO] Storage            : ${STORAGE_ID}"
echo "[INFO] Split duration     : ${SPLIT_DURATION_SEC} sec"
echo "[INFO] Topic count        : ${#TOPICS[@]}"
echo "[INFO] Topics             :"
printf '[INFO]   - %s\n' "${TOPICS[@]}"

########################################
# Start recording
########################################
exec ros2 bag record \
  -s "${STORAGE_ID}" \
  -o "${OUT_DIR}" \
  -d "${SPLIT_DURATION_SEC}" \
  "${TOPICS[@]}"


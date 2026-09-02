#!/usr/bin/env bash
# Isaac Sim 을 **ROS 브리지가 살아 있는 환경**으로 띄운다.
#
# 이 스크립트의 존재 이유는 환경 변수 세 줄이다. 그냥 `isaac-sim.sh` 를 실행하거나
# `~/isaac_ros2_env.sh` 를 소싱하면 **ROS 확장이 조용히 죽는다** (아래 §2).
#
#   ./scripts/run_isaac_sim.sh                 GUI. 스테이지 구성은 Script Editor 로
#   ./scripts/run_isaac_sim.sh --headless      GUI 없이 **전 과정 자동** (검사·CI 용)
#   ./scripts/run_isaac_sim.sh --headless --phase 3    카메라 없이 (조작 계열만)
#
# 뜨고 나면 ROS 쪽 스택을 따로 올린다:
#
#   ros2 launch robot_control panda_isaac.launch.py enable_gripper:=true enable_scene:=true
#
#
# ── 1. 왜 스크립트가 필요한가 ────────────────────────────────────────────────
#
# **시스템 ROS 를 소싱하면 안 된다.** Isaac 6.0 내장 파이썬은 3.12 이고 Humble 은
# 3.10 이다. `/opt/ros/humble/setup.bash` 를 소싱하면 3.10 `site-packages` 가
# PYTHONPATH 에 올라 Isaac 의 번들 3.12 ROS 를 **가린다.**
#
#   [Error] Failed to import ROS2 Python libraries: No module named 'rclpy._rclpy_pybind11'
#   [Error] Failed to initialize ROS2 service manager
#
# 확장은 "startup" 까지 찍고 죽으므로 **로그를 보지 않으면 성공한 것처럼 보인다.**
# 증상은 `/isaac_sim_control` 노드가 없는 것뿐이다.
#
# ⚠️ `~/isaac_ros2_env.sh` 가 정확히 이 문제를 만든다 — Windows + WSL2 구성에서
#    쓰던 방식이라 Ubuntu 에서는 반대로 작동한다. **쓰지 않는다.**
#
# `sim_control` 확장도 기본 비활성이라 `--enable` 로 켠다. 그것이 `/scene/reset` 이
# 쓰는 `simulation_interfaces` 서비스를 연다
# (docs/scene/isaac_scene_reset.md).
#
#
# ── 2. 도메인 ────────────────────────────────────────────────────────────────
#
# 그래프의 ROS2Context 가 `setup_graph.py` 의 상수(31)를 쓴다. 여기 값과 어긋나면
# 브리지 토픽만 다른 도메인으로 나가 **에러 없이 아무것도 안 보인다.**
#
#
# ── 3. 관련 문서 ─────────────────────────────────────────────────────────────
#
#   docs/simulation/isaac_backend_skeleton.md   Phase 0~10, 배포 구성, 6.0 재검증 결과
#   docs/scene/isaac_scene_reset.md             /scene/reset 경로와 선행 조건
set -euo pipefail

ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
ISAAC_ROS="$ISAAC_ROOT/exts/isaacsim.ros2.core/humble"
WORKSPACE="${RDFP_WORKSPACE:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
DOMAIN="${ROS_DOMAIN_ID:-31}"
PHASE="${ISAAC_PHASE:-4}"
HEADLESS=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) HEADLESS=1; shift ;;
        --phase)    PHASE="$2"; shift 2 ;;
        -h|--help)  sed -n '2,40p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

[[ -d "$ISAAC_ROOT" ]] || { echo "Isaac not found at $ISAAC_ROOT (set ISAAC_ROOT)" >&2; exit 1; }
[[ -d "$ISAAC_ROS" ]]  || { echo "bundled ROS not found at $ISAAC_ROS" >&2; exit 1; }

# `env -u` 로 시스템 ROS 의 흔적을 걷어낸다. 남아 있으면 3.10 이 3.12 를 가린다.
isaac_env=(
    env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH -u COLCON_PREFIX_PATH
    ROS_DISTRO=humble
    ROS_DOMAIN_ID="$DOMAIN"
    RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
    PYTHONPATH="$ISAAC_ROS/rclpy"
    LD_LIBRARY_PATH="$ISAAC_ROS/lib"
    RDFP_WORKSPACE="$WORKSPACE"
)

echo "[isaac] root=$ISAAC_ROOT  domain=$DOMAIN  workspace=$WORKSPACE"

if [[ "$HEADLESS" == "1" ]]; then
    echo "[isaac] headless bring-up (PHASE=$PHASE) — 로봇 로드부터 Play 까지 자동"
    exec "${isaac_env[@]}" ISAAC_PHASE="$PHASE" \
        "$ISAAC_ROOT/python.sh" "$WORKSPACE/scripts/isaac/sim_side/headless_bringup.py"
fi

cat <<EOF
[isaac] GUI 로 띄운다. 뜬 뒤 Script Editor 에서 아래를 순서대로 실행한다.

    path = "$WORKSPACE"
    for _s in ("setup_scene", "setup_graph", "place_robot",
               "set_home_pose", "tune_drive", "tune_grasp"):
        exec(open(path + "/scripts/isaac/sim_side/%s.py" % _s, encoding="utf-8").read())

  그다음 **Stop → Play**. 로그는 /tmp/isaac_*.log 에 남는다.

  ⚠️ 로봇이 스테이지에 없으면 setup_graph 가 'no articulation root' 로 멈춘다.
     asset browser 에서 Franka 를 올리거나 --headless 를 쓴다 (자동으로 불러온다).
EOF

exec "${isaac_env[@]}" "$ISAAC_ROOT/isaac-sim.sh" --enable isaacsim.ros2.sim_control

#!/usr/bin/env bash
#
# robot twin (HTTP 게이트웨이) 를 띄운다.
#
#   ./scripts/run_robot_twin.sh                        # mock 스택   (:8801)
#   ./scripts/run_robot_twin.sh --isaac                # Isaac 스택  (:8802)
#   ./scripts/run_robot_twin.sh --isaac --log-level debug   # 나머지 인자는 그대로 전달된다
#   RDFP_TWIN_CONFIG=<path> ./scripts/run_robot_twin.sh     # 임의의 설정 파일
#
# **로봇 스택이 먼저 떠 있어야 한다.** 트윈은 로봇을 제어하지 않고 중계만 한다.
#
#   mock :  ./scripts/run_panda_mock.sh                 # 팔·그리퍼·scene 연산까지
#           ros2 launch rdfp rdfp_panda_mock.launch.py  # + 세션/에피소드 연산
#   Isaac:  ./scripts/run_isaac_sim.sh --gui            # ① 시뮬레이터가 Play 중이어야 한다
#           ./scripts/run_panda_isaac.sh                # ② 제어 스택
#           ros2 launch rdfp rdfp_panda_isaac.launch.py # ②' + 세션/에피소드 연산
#
# 세션/에피소드 연산(start_session 등)은 rdfp 의 session_control_node 에 중계되므로
# 수집 계열 launch 가 필요하다. 없으면 그 연산만 PRECONDITION_FAILED 로 거부되고
# 나머지는 정상 동작한다.
#
#
# ── 백엔드 선택 ──────────────────────────────────────────────────────────────
#
# **설정 파일 하나로 갈린다.** 트윈은 백엔드를 모르고, YAML 이 토픽·시계·scene 레시피를
# 정한다. 두 shipped 설정의 실질적 차이는 셋뿐이다:
#
#   * `ros.use_sim_time` — Isaac 은 **true**. false 로 두면 변수 stamp 가 벽시계로 찍혀
#     에피소드 경계(sim time)와 어긋나고 데이터셋에서 짝이 안 맞는다.
#   * `reset_scene` 레시피 — mock 은 물체를 **만들 수** 있지만 Isaac 은 스테이지에 이미
#     있는 것을 옮길 수만 있다. 그래서 Isaac 쪽 이름·종류·크기는
#     `robot_control/config/isaac_scene.json` 과 같아야 하고, 어긋나면 런타임에
#     `isaac_scene_state_node` 가 거부한다 (`test_isaac_scene_recipe.py` 가 빌드에서 잡는다).
#     안착 z 도 다르다 — Isaac 에는 테이블(상단 0.4)이 있고 mock 에는 없다.
#   * `twin.id` / `ros.node_name` / `http.port` — 한 머신에서 둘을 함께 띄우기 위한 구분.
#
# `moveit.move_group_mode` 는 **둘 다 jtc** 다. Isaac 도 `topic_based_ros2_control` 로
# `controller_manager` 를 ROS 쪽에서 돌려 `panda_arm_controller` 가 JTC 이기 때문이다.
#
# **`ROS_DOMAIN_ID` 를 export 할 필요는 없다** — `main.py` 가 YAML 의 `ros.domain_id`(31)
# 와 `rmw` 를 rclpy 초기화 **전에** 프로세스 환경변수로 넣는다. 반대로 셸에 다른 값이
# 있어도 YAML 이 이긴다.
#
#
# ── 세 번째 트윈을 띄울 때 ───────────────────────────────────────────────────
#
# **`--port` 만 바꾸면 안 된다.** `twin.id` 와 `ros.node_name` 이 겹쳐 같은 이름의 노드가
# 둘이 된다. shipped YAML 을 복사해 셋을 모두 다르게 준 뒤 `RDFP_TWIN_CONFIG` 로 지정한다.
#
#   cp src/robot_twin/config/robot_twin_panda01.yaml /tmp/panda02.yaml
#   # twin.id: panda02 / ros.node_name: robot_twin_panda02 / http.port: 8803 로 수정
#   RDFP_TWIN_CONFIG=/tmp/panda02.yaml ./scripts/run_robot_twin.sh
set -euo pipefail

# 워크스페이스 루트는 **스크립트 위치**에서 구한다. RDFP_HOME 에 의존하면 rdfp_env 를
# 켜지 않은 셸에서 빈 값이 앞에 붙어 '/src/robot_twin/...' 로 조용히 깨진다.
ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
config_dir="$ws_root/src/robot_twin/config"

backend=""
args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --isaac) backend="robot_twin_panda_isaac"; shift ;;
        --mock)  backend="robot_twin_panda01"; shift ;;
        # 나머지는 트윈에 그대로 넘긴다 (`--log-level` 등). `--config` 를 직접 주는 것도
        # 막지 않는다 — 그때는 아래 기본 `--config` 뒤에 붙어 argparse 가 뒤를 택한다.
        *) args+=("$1"); shift ;;
    esac
done

# 우선순위: RDFP_TWIN_CONFIG > --isaac/--mock > 기본(mock). 환경변수를 위에 두는 것은
# 임의의 파일을 쓰려는 쪽이 더 구체적인 요구이기 때문이다.
if [[ -n "${RDFP_TWIN_CONFIG:-}" ]]; then
    config="$RDFP_TWIN_CONFIG"
else
    config="$config_dir/${backend:-robot_twin_panda01}.yaml"
fi

if [[ ! -f "$config" ]]; then
    echo "twin config not found: $config" >&2
    echo "shipped: $(cd "$config_dir" && ls robot_twin_*.yaml | tr '\n' ' ')" >&2
    exit 2
fi

echo "[twin] config: $config"

# --config 는 절대 경로를 넘긴다 — ros2 run 은 작업 디렉터리를 보장하지 않는다.
# exec 로 넘겨 Ctrl+C 가 트윈에 직접 전달되게 한다.
exec ros2 run robot_twin robot_twin --config "$config" "${args[@]+"${args[@]}"}"

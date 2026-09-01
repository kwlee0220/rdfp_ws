#!/usr/bin/env bash
#
# robot twin (HTTP 게이트웨이) 를 띄운다.
#
#   ./scripts/run_robot_twin.sh                        # 기본 설정
#   ./scripts/run_robot_twin.sh --log-level debug      # 인자는 그대로 전달된다
#   RDFP_TWIN_CONFIG=<path> ./scripts/run_robot_twin.sh
#
# **로봇 스택이 먼저 떠 있어야 한다.** 트윈은 로봇을 제어하지 않고 중계만 한다.
#
#   ./scripts/run_panda_mock.sh                        # 팔·그리퍼·scene 연산까지
#   ros2 launch rdfp rdfp_panda_mock.launch.py         # + 세션/에피소드 연산
#
# 세션/에피소드 연산(start_session 등)은 rdfp 의 session_control_node 에 중계되므로
# 수집 계열 launch 가 필요하다. 없으면 그 연산만 PRECONDITION_FAILED 로 거부되고
# 나머지는 정상 동작한다.
#
# **두 번째 트윈을 띄울 때 --port 만 바꾸면 안 된다.** twin.id 와 ros.node_name 이
# 겹쳐 같은 이름의 노드가 둘이 된다. YAML 을 복사해 셋을 모두 다르게 준 뒤
# RDFP_TWIN_CONFIG 로 지정한다.
#
#   cp src/robot_twin/config/robot_twin_panda01.yaml /tmp/panda02.yaml
#   # twin.id: panda02 / ros.node_name: robot_twin_panda02 / http.port: 8802 로 수정
#   RDFP_TWIN_CONFIG=/tmp/panda02.yaml ./scripts/run_robot_twin.sh
set -euo pipefail

# 워크스페이스 루트는 **스크립트 위치**에서 구한다. RDFP_HOME 에 의존하면 rdfp_env 를
# 켜지 않은 셸에서 빈 값이 앞에 붙어 '/src/robot_twin/...' 로 조용히 깨진다.
ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

config="${RDFP_TWIN_CONFIG:-$ws_root/src/robot_twin/config/robot_twin_panda01.yaml}"
if [[ ! -f "$config" ]]; then
    echo "twin config not found: $config" >&2
    exit 2
fi

# --config 는 절대 경로를 넘긴다 — ros2 run 은 작업 디렉터리를 보장하지 않는다.
# exec 로 넘겨 Ctrl+C 가 트윈에 직접 전달되게 한다.
exec ros2 run robot_twin robot_twin --config "$config" "$@"

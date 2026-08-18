#!/usr/bin/env bash
#
# robot twin (HTTP 게이트웨이) 를 띄운다.
#
#   ./scripts/run_robot_twin.sh                        # 기본 설정
#   ./scripts/run_robot_twin.sh --port 8802            # 인자는 그대로 전달된다
#   RDFP_TWIN_CONFIG=<path> ./scripts/run_robot_twin.sh
set -euo pipefail

# 워크스페이스 루트는 **스크립트 위치**에서 구한다. RDFP_HOME 에 의존하면 rdfpenv 를
# 켜지 않은 셸에서 빈 값이 앞에 붙어 '/src/rdfp/...' 로 조용히 깨진다.
ws_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

config="${RDFP_TWIN_CONFIG:-$ws_root/src/rdfp/config/robot_twin_panda01.yaml}"
if [[ ! -f "$config" ]]; then
    echo "twin config not found: $config" >&2
    exit 2
fi

# --config 는 절대 경로를 넘긴다 — ros2 run 은 작업 디렉터리를 보장하지 않는다.
# exec 로 넘겨 Ctrl+C 가 트윈에 직접 전달되게 한다.
exec ros2 run rdfp robot_twin --config "$config" "$@"

#!/usr/bin/env bash
#
# robot twin (HTTP 게이트웨이) 를 띄운다.
#
#   ./scripts/run_robot_twin.sh --twin panda01          # Isaac 스택
#   ./scripts/run_robot_twin.sh --twin rdfp01                 # 펑션베이
#   ./scripts/run_robot_twin.sh --twin mock01                 # mock 스택 (쓰지 않을 계획이다)
#   ./scripts/run_robot_twin.sh --twin panda01 --log-level debug   # 나머지 인자는 그대로 전달된다
#   ./scripts/run_robot_twin.sh --config <path>                # 임의의 설정 파일
#
# **`--twin <id>` 는 설치본의 `config/twins/<id>.yaml` 을 바로 연다** (2026-10-25). id 가 곧 파일
# 이름이고, 파일의 `twin.id` 와 다르면 트윈이 기동을 거부한다. 트윈을 더하거나 뺄 때 **이 스크립트는
# 고치지 않는다** — `src/robot_twin/config/twins/<id>.yaml` 을 넣거나 빼고 `colcon build` 한다. 없는
# id 를 주면 설치된 id 와 각 백엔드를 보여 준다.
#
# `--backend` 는 없앴다 (2026-10-25). 백엔드는 각 설정 파일의 `twin.backend` 가 정한다.
#
# 포트는 출하 설정 셋 모두 **:8801** 이다(아래 "환경마다 설정 파일이 하나씩이다").
#
# **`--twin` · `--config` 중 하나는 필수다 — 기본값이 없다** (2026-10-21). 예전에는 인자를 빠뜨리면
# mock 설정으로 떴는데, 펑션베이 · Isaac 스택 위에 mock 트윈이 뜨면 에러 없이 어긋난다 — JTC
# 클라이언트가 없는 액션 서버를 불러 "계획은 되고 실행만 안 되고", Isaac 에서는 시계가 달라 변수
# stamp 가 에피소드 경계와 어긋난다. `teleop_keyboard` · `replay_gui` 의 `backend` 에 기본값이 없는
# 것과 같은 이유다.
#
# **로봇 스택이 먼저 떠 있어야 한다.** 트윈은 로봇을 제어하지 않고 중계만 한다.
#
#   mock    : ./scripts/run_panda_mock.sh               # 팔·그리퍼·scene 연산까지
#   Isaac   : ./scripts/run_isaac_sim.sh --gui          # ① 시뮬레이터가 Play 중이어야 한다
#             ./scripts/run_panda_isaac.sh              # ② 제어 스택
#   펑션베이: 시뮬레이터 · endpoint · /scene/start 후  # docs/backends/functionbay/펑션베이_시뮬레이터_사용법.md
#             ./scripts/run_panda_functionbay.sh        #   팔 연산은 차트가 끝난 뒤(⑥ fb_wait_chart.py)
#   + 세션/에피소드 연산이 필요하면: ros2 launch rdfp rdfp_collect.launch.py backend:=<panda_mock|panda_isaac|panda_functionbay>
#     (수집까지 할 때는 ./scripts/run_collect_isaac.sh · run_collect_functionbay.sh 가 함께 띄운다)
#
# 세션/에피소드 연산(start_session 등)은 rdfp 의 session_control_node 에 중계되므로
# 수집 계층이 필요하다. 없으면 그 연산만 거절되고(503 PRECONDITION_FAILED,
# 'session_control_node is not available') 나머지는 정상 동작한다.
#
# **트윈은 기동할 때 팔 컨트롤러의 진행 중 궤적을 모두 취소한다** (JTC — mock · Isaac).
# 이전 트윈이 남긴 궤적을 끊기 위해서이며, RViz 등 다른 주체가 움직이던 것도 함께 멈춘다
# (docs/twin/로봇_트윈_사용자_가이드.md §9.7).
#
#
# ── 환경마다 설정 파일이 하나씩이다 ──────────────────────────────────────────
#
# 출하 설정은 환경마다 하나씩이다. 백엔드 **사실**은 그 파일의 `twin.backend` 한 줄이 백엔드
# 프로파일에서 채운다 —
# 팔 명령 채널 · 속도 · 프레임 · 고정물 · **시계
# (`ros.use_sim_time`)**. 그래서 시계는 트윈 YAML 에 적지 않는다(2026-10-21 부터 — 프로파일과 두 곳에
# 있으면 한쪽만 고쳤을 때 조용히 어긋난다. 적으면 그 값이 이긴다). 설정 파일마다 다른 것은 트윈 고유의
# 것뿐이다:
#
#   * `reset_scene` 레시피 — mock 은 물체를 **만들 수** 있지만 Isaac 은 스테이지에 이미
#     있는 것을 옮길 수만 있다. 그래서 Isaac 쪽 이름·종류·크기는
#     `robot_control/config/isaac_scene.json` 과 같아야 하고, 어긋나면 런타임에
#     `isaac_scene_state_node` 가 거부한다 (`test_isaac_scene_recipe.py` 가 빌드에서 잡는다).
#     안착 z 도 다르다 — Isaac 에는 테이블이 있고 mock 에는 없다.
#     **펑션베이에는 `reset_scene` 이 없다** (`/scene/reset` 을 제공하지 않는다). 대신
#     고정물 위치를 `fixtures` 변수로 낸다.
#
# 출하 트윈은 **환경마다 id 가 다르다** (2026-10-25) — `mock01` · `panda01` · `rdfp01`(펑션베이).
# 노드 이름은 id 를 따르고(`robot_twin_<id>`), 클라이언트는 붙을 트윈의 id 를 넘긴다(틀리면 404).
# `http.port`(8801) 는 셋이 같아 동시에 띄우지 않는다 — 실수로 둘을 띄우면 두 번째가 포트 충돌로
# 못 뜬다. 어느 환경인지는 `twin.backend` 가 말하고, 클라이언트는 트윈 메타
# (`GET /api/v1/robot_twins/<id>`)의 `backend` 로 확인한다.
#
# 모드는 mock · Isaac 이 JTC, 펑션베이가 JGPC(개루프 스트리밍)다. Isaac 도
# `topic_based_ros2_control` 로 `controller_manager` 를 ROS 쪽에서 돌려 JTC 다.
# `panda_mock_jgpc` 백엔드에는 출하 트윈 설정이 없다 — 필요하면 mock 설정을 복사해 `twin.backend:
# panda_mock_jgpc` 로 바꾸고 `--config` 로 준다.
#
# **트윈 쪽은 `ROS_DOMAIN_ID` 를 export 할 필요가 없다** — `main.py` 가 YAML 의 `ros.domain_id`(31)
# 와 `rmw` 를 rclpy 초기화 **전에** 프로세스 환경변수로 넣는다. 반대로 셸에 다른 값이
# 있어도 YAML 이 이긴다(기동 로그에 덮어쓴 사실이 남고, 셸 값이 다르면 WARN). **로봇 스택 ·
# 수집 계층은 셸 값을 그대로 쓰므로** 그쪽 터미널은 `rdfp_env` 로 같은 도메인을 켜야 한다 —
# 안 켜면 도메인 0 으로 떠서 트윈이 아무것도 못 받는다(증상은 변수의 `NO_DATA` 뿐).
#
#
# ── 다른 로봇의 트윈을 함께 띄울 때 ───────────────────────────────────────────
#
# 트윈 1개 = 로봇 1대다. 출하 설정을 복사해 **`twin.id` · `ros.node_name` · `http.port` ·
# `ros.domain_id` 넷을** 바꾼다. 출하 설정으로 둘 것이면 `src/robot_twin/config/twins/<id>.yaml` 로
# 넣고 빌드해 `--twin <id>` 로 고르고, 임시로 쓸 것이면 `--config` 로 지정한다. 그 로봇의 스택도 같은
# `ROS_DOMAIN_ID` 로 띄운다 — 토픽 이름이 절대 이름이라(로봇별 네임스페이스는 미구현) 같은
# 도메인에 두 스택을 띄우면 섞인다. **`--port` 만 바꾸면 같은 로봇에 트윈이 둘 붙는다**(같은
# 컨트롤러에 명령이 겹친다). `--twin` 과 `--config` 는 함께 주지 않는다(주면 거부한다).
#
#   cp src/robot_twin/config/twins/panda01.yaml src/robot_twin/config/twins/panda02.yaml
#   # twin.id: panda02 / ros.node_name: robot_twin_panda02 / http.port: 8802 / ros.domain_id: 32 로 수정
#   colcon build --packages-select robot_twin
#   ./scripts/run_robot_twin.sh --twin panda02
set -euo pipefail

# **설정 파일은 트윈(`robot_twin.shipped_configs`)이 찾는다** (2026-10-25). `--twin <id>` 는
# 설치본의 `install/robot_twin/share/robot_twin/config/twins/<id>.yaml` 이다. 예전에는 이 스크립트가
# 백엔드 → 파일 이름 표를 들고 있어 트윈을 더하거나 뺄 때마다 셸을 고쳐야 했고, YAML 의
# `twin.backend` 와 표가 어긋나면 엉뚱한 파일을 조용히 골랐다.
#
# **설치본을 읽는다 — `src/` 가 아니다** (2026-09-21). 트윈의 `reset_scene` 레시피와 scene 노드의
# `isaac_scene.json` 은 같은 장면을 두 곳에 적은 것이라 같은 시점의 값이어야 하는데, 트윈만
# `src/` 를 읽던 시절 실린더 반지름을 바꾼 뒤 수집이 `dimensions [0.06, 0.03] != [0.06, 0.025]` 로
# 매 회 거부됐다(탁자 높이만 바뀌었다면 위치는 검사하지 않으므로 조용히 틀린 높이로 수집했을
# 것이다). 대가: **설정을 고치면 `colcon build` 를 해야 반영된다.** 설치본은 소싱한 환경의
# `robot_twin` 패키지에서 찾는다.

die() {
    echo "run_robot_twin: $*" >&2
    exit 2
}

args=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        # 없앤 인자 — 트윈 id 로 고른다. 없는 id 를 주면 트윈이 설치된 id 와 백엔드를 보여 준다.
        --backend|--backend=*|--mock|--isaac|--functionbay|--fb)
            die "'${1%%=*}' was removed — pick the twin by id: --twin <id> (e.g. --twin panda01; an unknown id lists the installed ones)" ;;
        # --config 는 절대 경로로 넘긴다 — ros2 run 은 작업 디렉터리를 보장하지 않는다.
        --config)
            [[ $# -ge 2 ]] || die "--config needs a path"
            args+=(--config "$(readlink -f -- "$2" || echo "$2")"); shift 2 ;;
        --config=*)
            args+=(--config "$(readlink -f -- "${1#--config=}" || echo "${1#--config=}")"); shift ;;
        # 나머지(`--twin` · `--log-level` …)는 트윈에 그대로 넘긴다.
        *) args+=("$1"); shift ;;
    esac
done

# exec 로 넘겨 Ctrl+C 가 트윈에 직접 전달되게 한다.
exec ros2 run robot_twin robot_twin "${args[@]+"${args[@]}"}"

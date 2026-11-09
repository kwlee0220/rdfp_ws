#!/usr/bin/env bash
# 펑션베이 백엔드에서 **학습 데이터 수집을 받을 준비**를 한다 — 수집 계층과 rosbag 을 띄운다.
#
#   ./scripts/run_collect_functionbay.sh                     ~/rdfp_data 아래에 모은다
#   ./scripts/run_collect_functionbay.sh --data-root DIR     다른 곳에 모은다
#   ./scripts/run_collect_functionbay.sh --with-stack        제어 스택·트윈까지 띄운다
#   ./scripts/run_collect_functionbay.sh --viewer            카메라 창을 띄운다 (기본은 안 띄운다)
#   ./scripts/run_collect_functionbay.sh --shutdown rdfp     수집이 끝나면 ROS 쪽을 내린다
#   ./scripts/run_collect_functionbay.sh --skip-chart-wait   SignalChart 종료 확인을 건너뛴다
#   ./scripts/run_collect_functionbay.sh --check-only        선행 조건만 확인하고 끝낸다
#
# `run_collect_isaac.sh` 의 펑션베이 판이다. **종료 처리 · 수집 계층 · rosbag · 자동 종료는
# 그쪽과 같은 코드다** — 한쪽을 고치면 다른 쪽도 본다. 왜 그렇게 하는지(rosbag 을 SIGINT 로
# 먼저 끝내는 이유, 자식 노드를 직접 거두는 이유, 뷰어를 argv 로 고르는 이유 …)는 그쪽
# 주석이 정본이다.
#
# **시뮬레이터 쪽은 띄우지 않는다.** 솔버 · 시뮬레이터(Windows) · `ros_tcp_endpoint` ·
# `/scene/start` 는 이 스크립트보다 먼저 끝나 있어야 한다
# (docs/backends/functionbay/펑션베이_시뮬레이터_사용법.md §2 ①~④). `--with-stack` 은 그
# 다음 단계인 제어 스택(⑤)과 트윈만 띄운다.
#
# 뜬 뒤에 **에피소드를 구동하는 것은 따로**다 (이 스크립트는 담기만 한다). 펑션베이에는
# `robot-twin-collect` 같은 구동기가 아직 없다 — 그것은 `reset_scene` 을 부르는데 펑션베이는
# `/scene/reset` 이 없다. 경계는 트윈 REST 로 찍는다 — 준비되면 명령을 출력한다.
#
# `--shutdown`, Ctrl-C, 띄운 것을 거두는 규칙은 Isaac 판과 같다. 단 **`--shutdown` 은
# `rdfp` 만 받는다** — 시뮬레이터가 Windows 쪽이라 여기서 내릴 수 없다(아래 ⑤).
#
#
# ── 1. Isaac 판과 다른 것 ────────────────────────────────────────────────────
#
# **① SignalChart 가 끝나야 준비된 것이다.** `/scene/start` 와 함께 차트가 자동 재생되어
# 팔·그리퍼가 움직이는데, 그 도중 `/input/*` 로 명령이 하나라도 가면 시뮬레이터가 토픽
# 모드로 바꾸고 차트를 버린다 — 팔·peg 가 중간 상태에 남는다. 그래서 "준비됐다"를 내기
# 전에 `fb_wait_chart.py` 로 차트 종료를 확인한다. 이미 끝났으면 몇 초(멈춤 3 s, 팔이 끝
# 자세에 없으면 차트 길이만큼) 안에 통과한다. 설정 DB(`FB_MMR_DATABASE`)를 못 읽는 등 판정을
# 못 하면 멈춘다 — 끝난 것을 알면 `--skip-chart-wait`.
#
# **② `/joint_states` stamp 가 벽시계여야 한다** (체크리스트 C-4). 시뮬레이터 원 stamp 는
# 회차마다 벽시계와 경과시간을 오가고 `joint_state_fusion` 이 벽시계로 정렬한다. 정렬이
# 꺼져 경과시간이면 적재가 에피소드 창으로 걸러 **rosbag 에는 남고 DB 에만 안 들어간다** —
# 다 모은 뒤에도 오류 없이. 그래서 표본 하나를 받아 지금 시각과 대조한다.
#
# **③ 레코더가 받을 카메라가 와야 한다.** 세션 레코더는 `camera_restamp` 의 출력(프로파일
# `camera.compressed_topic`)을 구독한다. 안 오면 mp4 가 0 프레임이고 frame clock(E1)이 서지
# 않아 export 가 통째로 막힌다 — 이것도 적재까지 가서야 드러난다.
#
# **④ 옛 bag 경고가 없다.** Isaac 은 sim 시계가 재기동마다 0 으로 되감겨 에피소드 키가 회차
# 간에 겹치지만, 펑션베이는 `use_sim_time: false` 이고 녹화 토픽이 전부 벽시계 축이라 키가
# 안 겹친다. 적재가 끝난 bag 을 `rosbag_dir` 밖으로 빼는 습관은 그대로 권한다.
#
# **⑤ 시뮬레이터는 여기서 못 내린다.** Windows 쪽 프로세스라 `--shutdown` 은 `rdfp`(ROS 쪽)만
# 받는다. `ros_tcp_endpoint` 는 다른 워크스페이스(tcp_ws)라 `kill_stack.sh --ros` 에 안 걸려
# 남는다 — 이어서 다시 모을 때 그대로 쓴다.
#
# **⑥ `--viewer` 는 제어 계층의 raw 디코더가 있어야 보인다.** 수집 계층 뷰어는 raw
# `/camera_image` 를 구독하는데, 펑션베이는 압축만 발행해 제어 계층이 `enable_camera:=true`
# 로 떠 있어야 그 raw 가 생긴다. `--with-stack` 은 `--viewer` 일 때 그렇게 띄우고, 이미 떠
# 있는 스택에 raw 발행자가 없으면 경고만 한다 — 수집에는 영향이 없다(레코더는 압축을 받는다).
#
#
# ── 2. 관련 문서 ─────────────────────────────────────────────────────────────
#
#   docs/backends/functionbay/펑션베이_시뮬레이터_사용법.md   **띄우는 순서** (§2) — 솔버부터
#                                                         차트 종료 대기까지
#   docs/backends/functionbay/펑션베이_인수_체크리스트.md     C-4 (stamp) 등 수집 판정
#   docs/collect/수집_계층_얹기_절차서.md                    rdfp_collect 의 백엔드 중립 설명
#   docs/collect/rosbag_명령_가이드.md                       녹화 · 적재 · 정리
#   scripts/run_collect_isaac.sh                            같은 일의 Isaac 판 (공통 부분의 근거)
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${RDFP_DATA_ROOT:-$HOME/rdfp_data}"
TWIN_HOST="${RDFP_TWIN_HOST:-127.0.0.1}"
TWIN_PORT="${RDFP_TWIN_PORT:-8801}"
TWIN_ID="${RDFP_TWIN_ID:-rdfp01}"   # 펑션베이 트윈의 id (2026-10-25)
CHECK_ONLY=0
VIEWER=0
WITH_STACK=0
SKIP_CHART=0
STARTED_STACK=0    # 우리가 띄웠으면 1 — 그때만 우리가 거둔다
SHUTDOWN=""          # "" | rdfp — 비면 자동 종료를 안 한다
SHUTDOWN_ARMED=0     # 정상 종료 경로를 탔을 때만 1 (Ctrl-C 에서는 스택을 안 내린다)
# 표본 stamp 와 지금 시각의 차이가 이보다 크면 벽시계 축이 아니다. 정렬된 stamp 는 수신보다
# 수십 ms 앞서고 `ros2 topic echo` 기동이 1~2 s 걸리므로 넉넉히 잡는다 — 경과시간이면 차이가
# 수십 년이라 경계값에 민감하지 않다.
STAMP_LAG_LIMIT_SEC=10

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data-root) DATA_ROOT="$2"; shift 2 ;;
        --viewer) VIEWER=1; shift ;;
        --with-stack) WITH_STACK=1; shift ;;
        --shutdown) SHUTDOWN="$2"; shift 2 ;;
        --skip-chart-wait) SKIP_CHART=1; shift ;;
        --check-only) CHECK_ONLY=1; shift ;;
        -h|--help) sed -n '2,27p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

case "$SHUTDOWN" in
    ""|rdfp) ;;
    all) echo "--shutdown all 은 펑션베이에서 못 쓴다 — 시뮬레이터가 Windows 쪽이다 (rdfp 만 받는다)" >&2
         exit 2 ;;
    *) echo "unknown --shutdown: $SHUTDOWN (rdfp)" >&2; exit 2 ;;
esac

TOPICS_FILE="$WS_ROOT/config/recording_topics.list"
ROSBAG_DIR="$DATA_ROOT/rosbag"
RECORDINGS_DIR="$DATA_ROOT/recordings"
VIDEOS_DIR="$DATA_ROOT/videos"
LOG_DIR="$DATA_ROOT/logs"
DATASET_YAML="$DATA_ROOT/dataset.yaml"

fail() { echo "[collect] 실패: $*" >&2; exit 1; }

# ── 종료 처리 ────────────────────────────────────────────────────────────────
#
# **rosbag 을 먼저, SIGINT 로 끝낸다** — 그래야 `metadata.yaml` 이 남는다. 수집 계층은 그
# 뒤, 스택은 가장 마지막이다. 근거는 run_collect_isaac.sh 의 같은 절.
COLLECT_PID=""
ROSBAG_PID=""

# 프로세스의 자손 PID 를 전부 모은다 (깊이 무관). 이름(`pkill -f`)이 아니라 부모-자식
# 관계로 고른다 — 이름으로 고르면 다른 스택이나 자기 자신까지 잡는다.
_descendants() {
    local pid="$1" child
    for child in $(pgrep -P "$pid" 2>/dev/null || true); do
        echo "$child"
        _descendants "$child"
    done
}

cleanup() {
    trap - INT TERM EXIT
    # **정리는 끝까지 간다.** 트랩 안에서도 `set -e` 가 살아 있어, 한 줄이 실패하면 거기서
    # 끝나 버린다 — 실제로 선행 확인 단계(아직 `rosbag/` 이 없다)에서 아래 `find` 파이프가
    # 실패해 **우리가 띄운 스택을 안 거두고** 성공도 종료 코드 1 로 바뀌었다. 끄면 트랩이
    # 끝까지 돌고 종료 코드는 `exit` 에 준 값 그대로다.
    set +e
    echo
    if [[ -n "$ROSBAG_PID" ]] && kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "[collect] rosbag 종료 (SIGINT — metadata.yaml 을 남기려면 이것이어야 한다)"
        kill -INT "$ROSBAG_PID" 2>/dev/null || true
        for _ in $(seq 1 100); do kill -0 "$ROSBAG_PID" 2>/dev/null || break; sleep 0.2; done
        kill -0 "$ROSBAG_PID" 2>/dev/null && { echo "[collect] 20초 내 미종료 — SIGKILL"; \
            kill -9 "$ROSBAG_PID" 2>/dev/null || true; }
    fi
    if [[ -n "$COLLECT_PID" ]] && kill -0 "$COLLECT_PID" 2>/dev/null; then
        echo "[collect] 수집 계층 종료"
        # 자식 목록을 신호 *전에* 찍어 둔다 — 부모가 죽으면 자식이 init 으로 재부모화된다.
        local kids
        kids="$(_descendants "$COLLECT_PID")"
        kill -INT "$COLLECT_PID" 2>/dev/null || true
        for _ in $(seq 1 50); do kill -0 "$COLLECT_PID" 2>/dev/null || break; sleep 0.2; done
        # 백그라운드 `ros2 launch` 는 SIGINT 를 받아도 자식 노드가 살아남는다 — 직접 거둔다.
        local survivors=""
        for p in $kids; do kill -0 "$p" 2>/dev/null && survivors+=" $p"; done
        if [[ -n "$survivors" ]]; then
            echo "[collect] 남은 자식 노드 정리:$survivors"
            # shellcheck disable=SC2086
            kill -TERM $survivors 2>/dev/null || true
            for _ in $(seq 1 25); do
                local still=""
                for p in $survivors; do kill -0 "$p" 2>/dev/null && still+=" $p"; done
                [[ -z "$still" ]] && break
                sleep 0.2
            done
            for p in $survivors; do
                kill -0 "$p" 2>/dev/null && { echo "[collect] 미종료 → SIGKILL $p"; \
                    kill -9 "$p" 2>/dev/null || true; }
            done
        fi
        kill -TERM "$COLLECT_PID" 2>/dev/null || true
    fi
    # `metadata.yaml` 이 실제로 생겼는지 본다 — 없으면 그 세션은 적재에서 조용히 빠진다.
    local session_dir
    session_dir="$(find "$ROSBAG_DIR" -mindepth 2 -maxdepth 2 -type d -name 'session_*' \
                   -newermt '-1 day' 2>/dev/null | sort | tail -1)"
    if [[ -n "$session_dir" ]]; then
        if [[ -f "$session_dir/metadata.yaml" ]]; then
            echo "[collect] bag OK: $session_dir"
        else
            echo "[collect] ⚠️ metadata.yaml 이 없다 — 이대로면 import 가 조용히 건너뛴다"
            echo "[collect]    복구: ros2 bag reindex -s mcap '$session_dir'"
        fi
    fi
    echo "[collect] 적재:  ./scripts/rosbag.sh import --config $DATASET_YAML"

    # 스택은 가장 마지막에, `--shutdown rdfp` 로 정상 종료했거나 **우리가 띄웠을 때만**
    # 내린다. 사람이 따로 띄운 스택은 계속 쓸 수 있어야 한다. 시뮬레이터와 endpoint 는
    # 어느 경우에도 남는다(위 ⑤).
    if [[ "$SHUTDOWN_ARMED" == "1" || "$STARTED_STACK" == "1" ]]; then
        echo "[collect] 로봇 스택 종료 — 시뮬레이터 · endpoint 는 남긴다"
        "$WS_ROOT/scripts/kill_stack.sh" --ros 2>&1 | sed 's/^/[collect]   /' || true
    fi
}
trap cleanup INT TERM EXIT

# ── 선행 조건 ────────────────────────────────────────────────────────────────

command -v ros2 >/dev/null 2>&1 || fail "ros2 가 없다 — install/setup.bash 를 소싱한다"

# **카메라 토픽이 녹화 목록에 살아 있으면 막는다** — 레코더 mp4 등록과 겹쳐 import 가
# UNIQUE 제약으로 죽는다. 펑션베이 카메라는 압축이라 bag 경로로는 애초에 적재되지도 않는다.
if grep -qE '^[[:space:]]*/.*camera.*image' "$TOPICS_FILE"; then
    fail "$TOPICS_FILE 에 카메라 토픽이 살아 있다 — 레코더 mp4 등록과 겹쳐 import 가 죽는다"
fi

# 레코더가 구독할 압축 토픽은 **프로파일에서 읽는다** — 여기 적어 두면 프로파일과 갈린다.
CAMERA_TOPIC="$(python3 -c "from robot_control.backends import get_backend
print(get_backend('panda_functionbay').value('camera', 'compressed_topic'))" 2>/dev/null)" \
    || fail "백엔드 프로파일을 못 읽었다 — colcon build 후 install/setup.bash 를 소싱한다"

TWIN_URL="http://$TWIN_HOST:$TWIN_PORT/api/v1/robot_twins/$TWIN_ID"

# **`joint_state_fusion` 과 `move_group` 이 둘 다 있어야 스택이 선 것이다.** 앞은 펑션베이
# 전용이라 다른 백엔드 스택과 구분되고, 뒤는 `readiness_gate` 가 첫 관절 보고를 받은 뒤에야
# 뜨므로 시뮬레이터와 한 번은 이어졌다는 뜻이다.
_stack_up() {
    local nodes
    nodes="$(timeout 20 ros2 node list 2>/dev/null || true)"
    grep -qx '/joint_state_fusion' <<<"$nodes" && grep -qx '/move_group' <<<"$nodes"
}
_twin_health() { curl -sf --max-time 5 "$TWIN_URL/health" 2>/dev/null || true; }

# 토픽 표본 하나의 `header.stamp` 가 지금보다 몇 초 뒤처졌는지 출력한다. 못 받으면 빈 출력.
_stamp_lag() {
    local topic="$1" out sec nsec
    out="$(timeout 10 ros2 topic echo "$topic" --once --field header.stamp 2>/dev/null || true)"
    sec="$(awk '/^sec:/ {print $2; exit}' <<<"$out")"
    nsec="$(awk '/^nanosec:/ {print $2; exit}' <<<"$out")"
    [[ -n "$sec" && -n "$nsec" ]] || return 0
    python3 -c 'import sys, time; print(f"{time.time() - int(sys.argv[1]) - int(sys.argv[2]) * 1e-9:.3f}")' \
        "$sec" "$nsec"
}

_lag_ok() { python3 -c 'import sys; sys.exit(0 if abs(float(sys.argv[1])) <= float(sys.argv[2]) else 1)' "$1" "$2"; }

# ── `--with-stack`: 제어 스택과 트윈을 직접 띄운다 ──────────────────────────
#
# **시뮬레이터 · endpoint · `/scene/start` 는 여기 없다** (위 머리말). 스택은 시작만으로는
# `/input/*` 에 아무것도 안 내므로 차트 도중에 띄워도 된다 — 팔을 움직이는 일만 ① 뒤로 미룬다.
#
# ⚠️ **이미 떠 있으면 띄우지 않는다.** 둘씩 띄우면 노드가 두 벌이 된다.
if [[ "$WITH_STACK" == "1" ]]; then
    mkdir -p "$LOG_DIR"
    if _stack_up; then
        echo "[collect] 제어 스택이 이미 떠 있다 — 그대로 쓴다"
    else
        stack_args=()
        # 수집 계층 뷰어가 볼 raw 를 제어 계층이 되살린다 (위 ⑥).
        [[ "$VIEWER" == "1" ]] && stack_args+=("enable_camera:=true")
        echo "[collect] 제어 스택 기동 (로그: $LOG_DIR/panda_functionbay.log)"
        "$WS_ROOT/scripts/run_panda_functionbay.sh" "${stack_args[@]+"${stack_args[@]}"}" \
            > "$LOG_DIR/panda_functionbay.log" 2>&1 &
        STACK_PID=$!
        STARTED_STACK=1
        # `readiness_gate` 가 첫 관절 보고를 기다린다(기본 60 s) — 못 받으면 launch 가 통째로 죽는다.
        echo -n "[collect] 시뮬레이터 연결 · move_group 대기 "
        for _ in $(seq 1 90); do
            _stack_up && { echo "— 준비됨"; break; }
            kill -0 "$STACK_PID" 2>/dev/null \
                || fail "제어 스택이 죽었다 — $LOG_DIR/panda_functionbay.log (readiness_gate 타임아웃이면 endpoint · /scene/start · ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>} 을 본다)"
            echo -n '.'; sleep 2
        done
        _stack_up || fail "move_group 이 180초 안에 안 섰다 — $LOG_DIR/panda_functionbay.log"
    fi

    if [[ -z "$(_twin_health)" ]]; then
        echo "[collect] 트윈 기동 (로그: $LOG_DIR/robot_twin.log)"
        "$WS_ROOT/scripts/run_robot_twin.sh" --twin "$TWIN_ID" > "$LOG_DIR/robot_twin.log" 2>&1 &
        TWIN_PID=$!
        STARTED_STACK=1
        echo -n "[collect] 트윈 대기 "
        for _ in $(seq 1 45); do
            [[ -n "$(_twin_health)" ]] && { echo "— 준비됨"; break; }
            kill -0 "$TWIN_PID" 2>/dev/null || fail "트윈이 죽었다 — $LOG_DIR/robot_twin.log"
            echo -n '.'; sleep 2
        done
    fi
fi

echo "[collect] 제어 스택 확인 …"
_stack_up || fail "제어 스택이 없다 — ./scripts/run_panda_functionbay.sh 를 먼저 띄우거나 --with-stack 을 준다 (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>})"

echo "[collect] 트윈 확인 …"
HEALTH="$(_twin_health)"
[[ -n "$HEALTH" ]] || fail "트윈이 없다 ($TWIN_URL) — ./scripts/run_robot_twin.sh --twin $TWIN_ID 를 띄우거나 --with-stack 을 준다"
grep -q '"move_group":"READY"' <<<"$HEALTH" \
    || fail "트윈의 move_group 이 준비되지 않았다: $HEALTH"
# 트윈이 이 환경의 것인가. 트윈은 환경마다 id 가 달라(`rdfp01` · `panda01` · `mock01`) 다른
# 환경의 트윈이 떠 있으면 대개 위 health 가 404 로 비어 걸리지만, **`RDFP_TWIN_ID` 를 바꿔 주면
# 통과한다** — 그대로 가면 이 스택 위에서 엉뚱한 설정(명령 채널 ·
# 시계)의 트윈이 에러 없이 명령을 낸다. 트윈 메타의 `backend` 로 가른다.
TWIN_BACKEND="$(curl -sf --max-time 5 "$TWIN_URL" 2>/dev/null \
    | python3 -c 'import json, sys; print(json.load(sys.stdin).get("backend") or "")' 2>/dev/null || true)"
[[ "$TWIN_BACKEND" == panda_functionbay ]] \
    || fail "떠 있는 트윈이 이 환경의 것이 아니다 (backend: ${TWIN_BACKEND:-<없음>}, 기대: panda_functionbay) — 그 트윈을 내리고 ./scripts/run_robot_twin.sh --twin $TWIN_ID 를 띄운다"

# ② `/joint_states` 가 오고, 그 stamp 가 벽시계 축인가 (C-4).
echo "[collect] /joint_states stamp 확인 (C-4) …"
lag="$(_stamp_lag /joint_states)"
[[ -n "$lag" ]] || fail "/joint_states 가 10초 안에 안 온다 — 시뮬레이션이 돌고 있나(/scene/start), endpoint 가 떠 있나"
_lag_ok "$lag" "$STAMP_LAG_LIMIT_SEC" \
    || fail "/joint_states stamp 가 벽시계가 아니다 (지금 − stamp = ${lag} s) — 적재에서 조용히 빠진다. joint_state_fusion 의 stamp_source 와 기동 로그 'stamp offset established' 를 본다"
echo "[collect]   벽시계 축 (지금 − stamp = ${lag} s)"

# ③ 레코더가 구독할 카메라가 오는가.
echo "[collect] 카메라 확인 ($CAMERA_TOPIC) …"
lag="$(_stamp_lag "$CAMERA_TOPIC")"
[[ -n "$lag" ]] || fail "$CAMERA_TOPIC 가 10초 안에 안 온다 — 이대로면 mp4 가 0 프레임이고 export 가 막힌다. 제어 스택의 camera_restamp 와 원본 /camera_image/compressed 를 본다"
_lag_ok "$lag" "$STAMP_LAG_LIMIT_SEC" \
    || fail "$CAMERA_TOPIC stamp 가 벽시계가 아니다 (지금 − stamp = ${lag} s) — camera_restamp 를 본다"
echo "[collect]   수신 중 (지금 − stamp = ${lag} s)"

# ⑥ `--viewer` 인데 raw 발행자가 없으면 창이 비어 있게 된다 — 경고만 한다.
if [[ "$VIEWER" == "1" ]]; then
    raw_pubs="$(timeout 10 ros2 topic info /camera_image 2>/dev/null \
                | awk '/Publisher count/ {print $3}' || true)"
    if [[ "${raw_pubs:-0}" == "0" ]]; then
        echo "[collect] ⚠️ /camera_image 발행자가 없다 — 카메라 창이 비어 있게 된다"
        echo "[collect]    제어 스택을 enable_camera:=true 로 다시 띄우면 보인다 (수집에는 영향 없음)"
    fi
fi

# ① SignalChart 가 끝났는가.
if [[ "$SKIP_CHART" == "1" ]]; then
    echo "[collect] SignalChart 종료 확인을 건너뛴다 (--skip-chart-wait)"
else
    echo "[collect] SignalChart 종료 확인 (fb_wait_chart.py) …"
    set +e
    PYTHONUNBUFFERED=1 "$WS_ROOT/scripts/functionbay/fb_wait_chart.py" 2>&1 | sed 's/^/[collect]   /'
    chart_code=${PIPESTATUS[0]}
    set -e
    case "$chart_code" in
        0) ;;
        1) fail "차트가 시간 안에 안 끝났다 — 팔이 계속 움직이고 있다. 시뮬레이터 화면을 본다" ;;
        2) fail "/output/panda_joint 보고가 안 온다 — 시뮬레이션을 시작했나(/scene/start), endpoint 가 떠 있나" ;;
        3) fail "차트를 못 읽어 판정하지 못했다 — FB_MMR_DATABASE 를 확인하거나, 끝난 것을 알면 --skip-chart-wait" ;;
        *) fail "fb_wait_chart.py 가 코드 $chart_code 로 끝났다" ;;
    esac
fi

if [[ "$CHECK_ONLY" == "1" ]]; then
    echo "[collect] 선행 조건 OK (--check-only 라 여기서 끝낸다)"
    exit 0
fi

# ── 제어 계층 뷰어를 내린다 ──────────────────────────────────────────────────
#
# 제어 스택을 `enable_image_viewer:=true` 로 띄웠다면 창이 둘이 된다 — 수집 계층 것만 남긴다.
# argv 앞 두 원소가 **그 실행 파일 자체**인 프로세스만 고른다 (run_collect_isaac.sh 의 같은 절).
# `camera_republish` 는 건드리지 않는다 — 수집 계층 뷰어가 그 raw 를 본다.
_viewer_pids() {
    local target="$1" pid argv0 i
    for pid in /proc/[0-9]*; do
        pid="${pid#/proc/}"
        [[ "$pid" == "$$" ]] && continue
        i=0
        while IFS= read -r -d '' argv0; do
            [[ "$argv0" == *"$target" ]] && { echo "$pid"; break; }
            i=$((i + 1)); [[ "$i" -ge 2 ]] && break
        done < "/proc/$pid/cmdline" 2>/dev/null
    done
}

viewer_pids="$(_viewer_pids 'robot_control/lib/robot_control/image_viewer_node' || true)"
if [[ -n "$viewer_pids" ]]; then
    echo "[collect] 제어 계층 뷰어 종료: $(echo "$viewer_pids" | tr '\n' ' ')"
    # shellcheck disable=SC2086
    kill -INT $viewer_pids 2>/dev/null || true
fi
echo "[collect] 카메라 창: $([[ "$VIEWER" == "1" ]] && echo '켬 (--viewer)' || echo '끔')"

mkdir -p "$ROSBAG_DIR" "$RECORDINGS_DIR" "$VIDEOS_DIR" "$LOG_DIR"


# ── 수집 계층 ────────────────────────────────────────────────────────────────

echo "[collect] 수집 계층 기동 (로그: $LOG_DIR/rdfp_collect.log)"
collect_viewer_arg="false"
[[ "$VIEWER" == "1" ]] && collect_viewer_arg="true"

ros2 launch rdfp rdfp_collect.launch.py backend:=panda_functionbay \
    image_recorder_output_dir:="$RECORDINGS_DIR" \
    enable_image_viewer:="$collect_viewer_arg" \
    > "$LOG_DIR/rdfp_collect.log" 2>&1 &
COLLECT_PID=$!

# 서비스가 설 때까지 기다린다 — 먼저 rosbag 을 띄우면 첫 `/session` 전이를 놓칠 수 있다.
echo -n "[collect] session_control 대기 "
for _ in $(seq 1 60); do
    if timeout 5 ros2 service list 2>/dev/null | grep -q '^/session_control/start_episode$'; then
        echo "— 준비됨"; break
    fi
    kill -0 "$COLLECT_PID" 2>/dev/null || fail "수집 계층이 죽었다 — $LOG_DIR/rdfp_collect.log"
    echo -n '.'; sleep 1
done
timeout 5 ros2 service list 2>/dev/null | grep -q '^/session_control/start_episode$' \
    || fail "session_control 이 60초 안에 서지 않았다 — $LOG_DIR/rdfp_collect.log"

# ── 데이터셋 설정 ────────────────────────────────────────────────────────────
#
# 네 경로(rosbag · mp4 · 레코더 출력 · 녹화 목록)를 여기서 한 번에 정해 적어 둔다 — 손으로
# 옮겨 적다 `recordings_dir` 이 갈리면 `image_frames` 가 0 이 되어 export 가 막힌다.
# rosbag 보다 먼저 쓴다 — `rosbag.sh record --config` 가 이 설정과 녹화 목록을 대조한다.
cat > "$DATASET_YAML" <<EOF
# ./scripts/run_collect_functionbay.sh 가 생성했다 ($(date '+%F %T')). 손으로 고쳐도 되지만
# 다음 실행이 덮어쓴다.
rosbag_dir:     $ROSBAG_DIR
output_mp4_dir: $VIDEOS_DIR
recordings_dir: $RECORDINGS_DIR
topics_file:    $TOPICS_FILE
db:
  dsn_env: RDFP_DB_DSN
  schema: public
EOF

# ── rosbag ───────────────────────────────────────────────────────────────────
#
# `rosbag.sh` 는 녹화까지 `exec` 로 넘기므로 `$!` 가 곧 `ros2 bag record` 의 PID 다.
echo "[collect] rosbag 기동 (로그: $LOG_DIR/rosbag.log)"
"$WS_ROOT/scripts/rosbag.sh" record \
    --topics-file "$TOPICS_FILE" --rosbag-dir "$ROSBAG_DIR" --config "$DATASET_YAML" \
    > "$LOG_DIR/rosbag.log" 2>&1 &
ROSBAG_PID=$!
sleep 3
kill -0 "$ROSBAG_PID" 2>/dev/null || fail "rosbag 이 뜨지 못했다 — $LOG_DIR/rosbag.log"

cat <<EOF

[collect] 준비됐다. 다른 터미널에서 에피소드 경계를 찍고 작업을 돌린다.
  펑션베이용 구동기(robot-twin-collect 같은)는 아직 없어 경계를 트윈 REST 로 직접 찍는다:

    T=$TWIN_URL/operations
    H='Content-Type: application/json'
    curl -sX POST \$T/start_session -H "\$H" -d '{"inputs":{"task_label":"<작업 이름>"}}'
    curl -sX POST \$T/start_episode -H "\$H" -d '{"inputs":{}}'
    robot-twin-peg --no-home        # 작업 (robot-twin-client). ready 복귀는 경계 밖에서 한다
    curl -sX POST \$T/stop_episode  -H "\$H" -d '{"inputs":{"outcome":"success"}}'   # 실패도 닫는다
    curl -sX POST \$T/stop_session  -H "\$H" -d '{"inputs":{}}'

  ⚠️ 펑션베이는 /scene/reset 이 없다 — peg 이 씬을 벗어나면 복구는 시뮬레이터 재시작뿐이다.

  데이터:   $DATA_ROOT
  설정:     $DATASET_YAML
  끝낼 때:  $([[ -n "$SHUTDOWN" ]] \
        && echo "세션이 닫히면 자동 종료 (--shutdown $SHUTDOWN). Ctrl-C 도 된다" \
        || echo '이 터미널에서 Ctrl-C (rosbag 을 안전하게 닫는다)')

EOF

if [[ -z "$SHUTDOWN" ]]; then
    wait "$ROSBAG_PID"
    exit 0
fi

# ── 자동 종료 ────────────────────────────────────────────────────────────────
#
# **세션이 열렸다가 `IDLE` 로 돌아오면** 끝난 것으로 본다 — 바로 `IDLE` 을 기다리면 수집
# 계층이 막 떴을 때의 `IDLE` 에서 즉시 끝난다. 상태는 트윈 변수로 읽는다(`/session` 은
# TRANSIENT_LOCAL 이라 CLI 로 읽으려면 QoS 를 맞춰야 한다).
session_state() {
    curl -s --max-time 5 "$TWIN_URL/variables/session_state" 2>/dev/null \
        | sed -n 's/.*"state"[[:space:]]*:[[:space:]]*"\([A-Z_]*\)".*/\1/p'
}

echo "[collect] 자동 종료 대기 — 세션이 열렸다 닫히면 정리한다"
opened=0
while :; do
    if ! kill -0 "$ROSBAG_PID" 2>/dev/null; then
        echo "[collect] rosbag 이 멈췄다 — 정리한다"
        break
    fi
    state="$(session_state)"
    if [[ "$opened" == "0" ]]; then
        if [[ "$state" == "IN_SESSION" || "$state" == "IN_EPISODE" ]]; then
            opened=1
            echo "[collect] 세션 열림 — 닫히면 종료한다"
        fi
    elif [[ "$state" == "IDLE" ]]; then
        echo "[collect] 세션 닫힘 — 자동 종료"
        break
    fi
    sleep 2
done

# `cleanup` 이 rosbag·수집 계층을 정리한다 (EXIT trap). 로봇 스택은 그 뒤에 내린다.
SHUTDOWN_ARMED=1
exit 0

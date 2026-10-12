#!/usr/bin/env bash
# Isaac 백엔드에서 **학습 데이터 수집을 받을 준비**를 한다 — 수집 계층과 rosbag 을 띄운다.
#
#   ./scripts/run_collect_isaac.sh                    ~/rdfp_data 아래에 모은다
#   ./scripts/run_collect_isaac.sh --data-root DIR    다른 곳에 모은다
#   ./scripts/run_collect_isaac.sh --with-stack       로봇 스택·트윈까지 띄운다
#   ./scripts/run_collect_isaac.sh --viewer           카메라 창을 띄운다 (기본은 안 띄운다)
#   ./scripts/run_collect_isaac.sh --shutdown rdfp    수집이 끝나면 ROS 쪽만 내린다
#   ./scripts/run_collect_isaac.sh --shutdown all     시뮬레이터까지 내린다
#   ./scripts/run_collect_isaac.sh --check-only       선행 조건만 확인하고 끝낸다
#
# **`--with-stack` 은 터미널을 하나로 줄인다.** 없으면 제어 스택(`panda_isaac`)과
# 트윈이 **먼저 떠 있어야 하고**, 없으면 여기서 멈춘다. 주면 그 둘을 직접 띄우고
# 컨트롤러가 활성될 때까지 기다린다. **시뮬레이터는 포함하지 않는다** — 기동에 2분이
# 걸리고, 재기동하면 sim 시계가 0 으로 되감겨 에피소드 키가 겹치기 때문이다(§6).
#
# ⚠️ **띄운 것은 끝낼 때 함께 내린다.** `--shutdown` 을 안 줘도, Ctrl-C 로 끝내도
# 그렇다 — **이 스크립트가 띄운 것은 이 스크립트가 거둔다.** 스택을 남기고 싶으면
# `--with-stack` 을 쓰지 말고 터미널을 나눈다.
#
# **`--shutdown` 은 무인 수집용이다.** `/session` 이 열렸다가 `IDLE` 로 돌아오면
# rosbag·수집 계층을 정리하고 스스로 끝난다. 어디까지 내릴지는 값이 정한다.
#
#   --shutdown rdfp   수집 계층 + 로봇 스택(ROS 노드). **시뮬레이터는 남긴다** —
#                     이어서 더 모을 때 쓴다. Isaac 재기동은 2분이 걸리고 sim 시계가
#                     0 으로 되감겨 에피소드 키가 겹친다.
#   --shutdown all    거기에 시뮬레이터까지. 그날 작업을 끝낼 때 쓴다.
#
# 끝났다는 신호를 세션으로 잡는 이유는 이 스크립트가 에피소드를 구동하지 않기
# 때문이고, `robot-twin-collect` 가 `stop_session` 을 `finally` 로 부르므로 중간에
# 실패해도 신호가 온다.
#
# 뜬 뒤에 **에피소드를 구동하는 것은 따로**다 (이 스크립트는 담기만 한다):
#
#   robot-twin-collect --repeat 20
#
# Ctrl-C 로 이 스크립트를 끝내면 rosbag 과 수집 계층이 함께 정리된다.
#
# **기본은 카메라 창을 안 띄운다.** 뷰어는 매 프레임을 디코드해 창에 그리므로
# 그만큼 부하이고, `DISPLAY` 가 없는 환경에서는 켜 두면 노드가 못 뜬다.
# 눈으로 보려면 `--viewer` 를 준다 — 수집 계층 뷰어(`/rdfp_image_viewer_node`)가
# 뜨고, 프레임에 세션 상태가 겹쳐 그려져 지금이 `IN_EPISODE` 인지 확인된다.
#
# **제어 계층 뷰어(`/image_viewer`)는 어느 경우에도 내린다.** 두 launch 가 같은
# 이름의 `enable_image_viewer` 인자를 각자 갖고 있어, 쪼개서 띄우면 창이 둘이
# 되기 때문이다. 되살리려면 제어 스택을 다시 띄운다.
#
#
# ── 1. 왜 스크립트가 필요한가 — 조용히 실패하는 자리가 넷이다 ────────────────
#
# **① rosbag 을 SIGKILL 로 죽이면 `metadata.yaml` 이 안 써진다.** 그러면
# `discover_splits` 가 그 세션을 "녹화 중이거나 비정상 종료"로 보고 **조용히 건너뛴다** —
# `import` 는 성공으로 끝나고 요약만 비어 있다. 그래서 종료 처리가 **SIGINT 먼저**이고,
# 끝난 뒤 `metadata.yaml` 이 실제로 생겼는지 확인한다.
#
# **② 네 경로가 서로 맞아야 한다.** 레코더의 `output_dir` 과 데이터셋의
# `recordings_dir` 이 갈리면 mp4 를 못 찾아 `image_frames` 가 0 이 되고, 그러면
# frame clock(E1)이 서지 않아 export 가 통째로 막힌다. 그래서 여기서 한 번에 정하고
# `dataset.yaml` 까지 써 둔다.
#
# **③ 수집 계층 없이 에피소드를 돌리면 경계만 찍히고 아무것도 안 담긴다.** 오류가
# 아니라 빈 결과로 나타난다 — 2026-09-20 에 실제로 9회를 그렇게 돌렸다.
#
# **④ 제어 스택·트윈이 먼저 떠 있어야 한다.** 기본은 그 둘을 띄우지 않고 **없으면
# 여기서 멈춘다** (`run_panda_isaac.sh` · `run_robot_twin.sh --isaac` 가 각자 띄운다).
# 순서가 어긋나면 증상이 "에피소드는 도는데 데이터가 없다"라서 늦게 드러난다.
# `--with-stack` 을 주면 이 스크립트가 직접 띄우고 기다린다.
#
#
# ── 2. 관련 문서 ─────────────────────────────────────────────────────────────
#
#   docs/backends/isaac/isaac_bringup_runbook.md    **전 과정 명령 순서** (§2) — 시뮬레이터
#                                               부터 작업 수행·적재까지
#   docs/collect/수집_계층_얹기_절차서.md       rdfp_collect 의 백엔드 중립 설명
#   docs/learning/저장_형식.md           저장 형식과 export
#   docs/collect/session_control_guide.md       세션 상태 기계
#   src/rdfp/launch/README.md                   rdfp_collect 의 인자표
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATA_ROOT="${RDFP_DATA_ROOT:-$HOME/rdfp_data}"
TWIN_HOST="${RDFP_TWIN_HOST:-127.0.0.1}"
TWIN_PORT="${RDFP_TWIN_PORT:-8802}"
TWIN_ID="${RDFP_TWIN_ID:-panda_isaac}"
CHECK_ONLY=0
VIEWER=0
WITH_STACK=0
STARTED_STACK=0    # 우리가 띄웠으면 1 — 그때만 우리가 거둔다
SHUTDOWN=""          # "" | rdfp | all — 비면 자동 종료를 안 한다
SHUTDOWN_ARMED=0     # 정상 종료 경로를 탔을 때만 1 (Ctrl-C 에서는 스택을 안 내린다)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --data-root) DATA_ROOT="$2"; shift 2 ;;
        --viewer) VIEWER=1; shift ;;
        --with-stack) WITH_STACK=1; shift ;;
        --shutdown) SHUTDOWN="$2"; shift 2 ;;
        --check-only) CHECK_ONLY=1; shift ;;
        -h|--help) sed -n '2,32p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

case "$SHUTDOWN" in
    ""|rdfp|all) ;;
    *) echo "unknown --shutdown: $SHUTDOWN (rdfp | all)" >&2; exit 2 ;;
esac

TOPICS_FILE="$WS_ROOT/config/recording_topics_isaac.list"
ROSBAG_DIR="$DATA_ROOT/rosbag"
RECORDINGS_DIR="$DATA_ROOT/recordings"
VIDEOS_DIR="$DATA_ROOT/videos"
LOG_DIR="$DATA_ROOT/logs"
DATASET_YAML="$DATA_ROOT/dataset.yaml"

fail() { echo "[collect] 실패: $*" >&2; exit 1; }

# ── 종료 처리 ────────────────────────────────────────────────────────────────
#
# **rosbag 을 먼저, SIGINT 로 끝낸다** (위 ①). 수집 계층은 그 뒤다 — 먼저 내리면
# 마지막 에피소드의 `/session` 전이가 bag 에 안 남는다.
COLLECT_PID=""
ROSBAG_PID=""

# 프로세스의 자손 PID 를 전부 모은다 (깊이 무관).
#
# **`pkill -f <패턴>` 을 쓰지 않는 이유**: 이름으로 고르면 이 워크스페이스의 다른
# 스택이나 **자기 자신**까지 잡는다(실제로 겪었다). 부모-자식 관계로 좁히면 우리가
# 띄운 것만 정확히 거둔다.
_descendants() {
    local pid="$1" child
    for child in $(pgrep -P "$pid" 2>/dev/null || true); do
        echo "$child"
        _descendants "$child"
    done
}

cleanup() {
    trap - INT TERM EXIT
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
        # **자식 목록을 신호 *전에* 찍어 둔다.** 부모가 죽으면 자식은 init 으로
        # 재부모화되어 `pgrep -P` 로 더는 못 찾는다.
        local kids
        kids="$(_descendants "$COLLECT_PID")"
        kill -INT "$COLLECT_PID" 2>/dev/null || true
        for _ in $(seq 1 50); do kill -0 "$COLLECT_PID" 2>/dev/null || break; sleep 0.2; done
        # ⚠️ **`ros2 launch` 에 SIGINT 를 줘도 자식 노드는 살아남는다** (2026-09-20
        # 실측: 두 회차의 `session_control` · `image_recorder` · 뷰어가 그대로 남아
        # 노드가 두 벌이 됐다). 터미널 Ctrl-C 로는 정리되지만 백그라운드로 띄우면
        # 제어 터미널이 없어 전달되지 않는다. 그래서 **자식을 직접 거둔다.**
        # 고아를 남기면 다음 회차에 노드가 겹치고, 카메라 주기가 1 Hz 대로 떨어진다.
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
    # **`metadata.yaml` 이 실제로 생겼는지 본다** — 없으면 그 세션은 적재에서 조용히
    # 빠진다. 복구법까지 여기서 알려 준다.
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
    echo "[collect] 적재:  ros2 run rdfp import --config $DATASET_YAML"

    # **스택은 가장 마지막에 내린다.** 순서가 중요하다 — 먼저 내리면 마지막 에피소드의
    # `/session` 전이와 관절 상태가 bag 에 안 남는다.
    #
    # 내리는 경우가 둘이다:
    #   ① `--shutdown` 으로 정상 종료 — 값이 범위를 정한다 (`rdfp` / `all`)
    #   ② `--with-stack` 으로 **우리가 띄웠다** — 그러면 Ctrl-C 여도 우리가 거둔다.
    #      띄운 쪽이 거두지 않으면 고아가 되고, 다음 회차에 노드가 두 벌이 된다.
    #
    # 둘 다 아니면 건드리지 않는다 — 사람이 따로 띄운 스택은 계속 쓸 수 있어야 한다.
    local scope=""
    if [[ "$SHUTDOWN_ARMED" == "1" ]]; then
        scope="$SHUTDOWN"
    elif [[ "$STARTED_STACK" == "1" ]]; then
        scope="rdfp"                      # 우리가 띄운 것은 ROS 쪽뿐이다
    fi
    case "$scope" in
        rdfp)
            echo "[collect] 로봇 스택 종료 — 시뮬레이터는 남긴다"
            "$WS_ROOT/scripts/kill_stack.sh" --ros 2>&1 \
                | sed 's/^/[collect]   /' || true ;;
        all)
            echo "[collect] 로봇 스택 + 시뮬레이터 종료"
            "$WS_ROOT/scripts/kill_stack.sh" 2>&1 \
                | sed 's/^/[collect]   /' || true ;;
    esac
}
trap cleanup INT TERM EXIT

# ── 선행 조건 ────────────────────────────────────────────────────────────────

command -v ros2 >/dev/null 2>&1 || fail "ros2 가 없다 — install/setup.bash 를 소싱한다"

# **카메라 토픽이 녹화 목록에 살아 있으면 여기서 막는다** (위 ②와 같은 종류인데
# 이쪽은 `import` 가 UNIQUE 제약으로 **죽는다** — 조용하지 않지만 다 모은 뒤에 터진다).
if grep -qE '^[[:space:]]*/.*camera.*image' "$TOPICS_FILE"; then
    fail "$TOPICS_FILE 에 카메라 토픽이 살아 있다 — 레코더 mp4 등록과 겹쳐 import 가 죽는다"
fi

# **이미 bag 이 있으면 경고한다** (2026-09-20). sim 시계는 시뮬레이터 재기동마다 0 으로
# 되감기므로 에피소드 키 `(start_sec, start_nanosec)` 가 회차 간에 유일하지 않다 —
# 옛 bag 과 새 bag 을 함께 적재하면 **새 에피소드의 관절·연산 데이터가 0 건이 된다.**
# 이미지는 레코더 sidecar 로 붙어 정상이라 **겉보기에 멀쩡하다.**
if [[ -d "$ROSBAG_DIR" ]] && find "$ROSBAG_DIR" -name '*.mcap' -print -quit 2>/dev/null | grep -q .; then
    echo "[collect] ⚠️ $ROSBAG_DIR 에 이미 bag 이 있다 — 적재 전에 빼낸다"
    echo "[collect]    함께 적재하면 새 에피소드의 관절 데이터가 조용히 0 건이 된다"
    echo "[collect]    (sim 시계가 재기동마다 되감겨 에피소드 키가 겹친다)"
fi

TWIN_URL="http://$TWIN_HOST:$TWIN_PORT/api/v1/robot_twins/$TWIN_ID"

_stack_up()  { timeout 20 ros2 node list 2>/dev/null | grep -q '^/controller_manager$'; }
_twin_health() { curl -sf --max-time 5 "$TWIN_URL/health" 2>/dev/null || true; }

# ── `--with-stack`: 제어 스택과 트윈을 직접 띄운다 ──────────────────────────
#
# **시뮬레이터는 여기 없다.** Isaac 은 기동에 2분이 걸리고, 재기동하면 sim 시계가
# 0 으로 되감겨 **에피소드 키가 회차 간에 겹친다**(관절 데이터가 조용히 0 건이 되는
# 그 문제다). 그래서 Isaac 은 바깥에 두고 오래 살린다.
#
# ⚠️ **이미 떠 있으면 띄우지 않는다.** 둘씩 띄우면 액션 서버가 둘이 되어 클라이언트가
# 엉뚱한 쪽 응답을 받는다.
if [[ "$WITH_STACK" == "1" ]]; then
    mkdir -p "$LOG_DIR"
    if _stack_up; then
        echo "[collect] 제어 스택이 이미 떠 있다 — 그대로 쓴다"
    else
        echo "[collect] 제어 스택 기동 (로그: $LOG_DIR/panda_isaac.log)"
        "$WS_ROOT/scripts/run_panda_isaac.sh" > "$LOG_DIR/panda_isaac.log" 2>&1 &
        STACK_PID=$!
        STARTED_STACK=1
        echo -n "[collect] 컨트롤러 대기 "
        for _ in $(seq 1 90); do
            _stack_up && { echo "— 준비됨"; break; }
            kill -0 "$STACK_PID" 2>/dev/null \
                || fail "제어 스택이 죽었다 — $LOG_DIR/panda_isaac.log (Isaac 의 /clock 을 먼저 본다)"
            echo -n '.'; sleep 2
        done
        _stack_up || fail "컨트롤러가 180초 안에 안 섰다 — $LOG_DIR/panda_isaac.log"
    fi

    if [[ -z "$(_twin_health)" ]]; then
        echo "[collect] 트윈 기동 (로그: $LOG_DIR/robot_twin.log)"
        "$WS_ROOT/scripts/run_robot_twin.sh" --isaac > "$LOG_DIR/robot_twin.log" 2>&1 &
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
_stack_up || fail "제어 스택이 없다 — ./scripts/run_panda_isaac.sh 를 먼저 띄우거나 --with-stack 을 준다"

echo "[collect] 트윈 확인 …"
HEALTH="$(_twin_health)"
[[ -n "$HEALTH" ]] || fail "트윈이 없다 ($TWIN_URL) — ./scripts/run_robot_twin.sh --isaac 를 띄우거나 --with-stack 을 준다"
grep -q '"move_group":"READY"' <<<"$HEALTH" \
    || fail "트윈의 move_group 이 준비되지 않았다: $HEALTH"

if [[ "$CHECK_ONLY" == "1" ]]; then
    echo "[collect] 선행 조건 OK (--check-only 라 여기서 끝낸다)"
    exit 0
fi

# ── 제어 계층 뷰어를 내린다 ──────────────────────────────────────────────────
#
# **창이 두 개 뜨는 것을 막는다.** 제어 계층(`panda_isaac`)과 수집 계층
# (`rdfp_collect`)이 **같은 이름의 `enable_image_viewer` 인자**를 각자 갖고 있어,
# 둘을 따로 띄우면 뷰어도 둘이 된다 (`/image_viewer` + `/rdfp_image_viewer_node`).
# 묶음 launch(`rdfp_panda_isaac`)는 백엔드에 `enable_image_viewer:=false` 를 넘겨
# 이것을 피하는데, **쪼개서 띄우면 그 조정자가 없다.**
#
# 남기는 쪽은 **수집 계층**이다 — 프레임에 세션 상태를 겹쳐 그려, 지금이
# `IN_EPISODE` 인지 눈으로 확인된다. 제어 계층 것은 그냥 영상이다.
#
# 죽여도 안전하다: `panda_isaac.launch.py` 의 `OnProcessExit` 는 readiness gate
# 에만 걸려 있어 **연쇄 종료가 없다.** 되살리려면 제어 스택을 다시 띄운다.
#
# ⚠️ **명령줄 문자열 검색으로 고르면 안 된다.** 두 번 데였다.
#   ① `pkill -f image_viewer` 는 `rdfp_image_viewer_node` 도 잡아 **남기려던 쪽까지
#      죽인다** (부분 문자열이다).
#   ② 경로로 좁혀도 **그 경로를 '언급만' 하는 프로세스까지 잡는다** — 실제로
#      진단용 `grep` 이 든 셸이 잡혀 죽었다 (2026-09-20).
# 그래서 argv 원소 하나가 **그 실행 파일 자체**인지를 본다. `/proc/<pid>/cmdline`
# 은 NUL 로 구분되므로 원소 단위로 정확히 비교할 수 있다.
# **argv 앞 두 원소만 본다.** 노드는 `python3 <경로> --ros-args …` 꼴이라 경로가
# argv[1] 이다. 뒤쪽 원소까지 보면 경로를 인자로 넘긴 남의 프로세스가 걸린다.
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
# `enable_image_viewer` 는 `rdfp_collect` 의 `--show-args` 에 **안 나온다** —
# `_build_actions`(OpaqueFunction) 안에서 늦게 선언되기 때문이다. 그래도 인자로
# 넘기면 그쪽이 이긴다(명령줄이 DeclareLaunchArgument 기본값을 덮는다).
collect_viewer_arg="false"
[[ "$VIEWER" == "1" ]] && collect_viewer_arg="true"

ros2 launch rdfp rdfp_collect.launch.py backend:=isaac \
    image_recorder_output_dir:="$RECORDINGS_DIR" \
    enable_image_viewer:="$collect_viewer_arg" \
    > "$LOG_DIR/rdfp_collect.log" 2>&1 &
COLLECT_PID=$!

# **서비스가 설 때까지 기다린다.** 노드 목록에 이름이 뜨는 것과 서비스가 받을 준비가
# 된 것은 다르다 — 먼저 rosbag 을 띄우면 첫 `/session` 전이를 놓칠 수 있다.
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

# ── rosbag ───────────────────────────────────────────────────────────────────

echo "[collect] rosbag 기동 (로그: $LOG_DIR/rosbag.log)"
RECORDING_TOPICS_FILE="$TOPICS_FILE" RDFP_ROSBAG_DIR="$ROSBAG_DIR" \
    "$WS_ROOT/scripts/record_rosbag.sh" > "$LOG_DIR/rosbag.log" 2>&1 &
ROSBAG_PID=$!
sleep 3
kill -0 "$ROSBAG_PID" 2>/dev/null || fail "rosbag 이 뜨지 못했다 — $LOG_DIR/rosbag.log"

# ── 데이터셋 설정 ────────────────────────────────────────────────────────────
#
# 적재할 때 쓸 설정을 **여기서 써 둔다** — 위 ②의 네 경로가 손으로 옮겨 적는 순간
# 어긋날 수 있어서다. `db.dsn_env` 는 값이 아니라 **환경변수 이름**이다.
cat > "$DATASET_YAML" <<EOF
# ./scripts/run_collect_isaac.sh 가 생성했다 ($(date '+%F %T')). 손으로 고쳐도 되지만
# 다음 실행이 덮어쓴다.
rosbag_dir:     $ROSBAG_DIR
output_mp4_dir: $VIDEOS_DIR
recordings_dir: $RECORDINGS_DIR
topics_file:    $TOPICS_FILE
db:
  dsn_env: RDFP_DB_DSN
  schema: public
EOF

cat <<EOF

[collect] 준비됐다. 다른 터미널에서 에피소드를 돌린다:

    robot-twin-collect --repeat 20

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
# **끝났다는 신호는 `/session` 이 `IDLE` 로 돌아오는 것이다.** 이 스크립트는
# 에피소드를 구동하지 않으므로(그건 `robot-twin-collect` 가 한다) 스스로는 진행을
# 모른다. `robot-twin-collect` 는 마지막에 `stop_session` 을 **`finally` 로** 부르므로,
# 중간에 실패해도 세션은 닫힌다 — 그래서 이 신호가 믿을 만하다.
#
# ⚠️ **바로 `IDLE` 을 기다리면 안 된다** — 수집 계층이 막 떴을 때도 `IDLE` 이라
# 한 바퀴도 안 돌고 즉시 종료한다. 그래서 **열렸다가 닫히는 것**을 본다.
#
# 상태는 `/session` 토픽 대신 **트윈 변수**로 읽는다. 그 토픽은 TRANSIENT_LOCAL 이라
# `ros2 topic echo` 에 QoS 를 맞춰 줘야 하는데, 트윈은 이미 구독해 두고 REST 로 낸다.
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

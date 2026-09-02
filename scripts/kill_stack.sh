#!/usr/bin/env bash
# 로봇 스택과 시뮬레이터의 **잔여물을 정리한다.**
#
#   ./scripts/kill_stack.sh --dry-run     무엇을 죽일지만 본다 (먼저 이것부터)
#   ./scripts/kill_stack.sh               전부 정리
#   ./scripts/kill_stack.sh --isaac       Isaac 만
#   ./scripts/kill_stack.sh --ros         ROS 노드만 (시뮬레이터는 둔다)
#
#
# ── 왜 필요한가 — 잔여물이 네 종류다 ────────────────────────────────────────
#
# **① `ros2 launch` 를 죽여도 자식 노드는 살아남는다.** 터미널에서 Ctrl-C 로 끝내면
# 정리되지만, 프로세스를 `kill` 하거나 SSH 세션이 끊기면 `move_group`,
# `robot_state_publisher`, `isaac_scene_state` … 가 부모 없이 계속 돈다. 스택을 다시
# 띄우면 **노드가 둘씩** 되고, 액션 서버가 둘이면 클라이언트가 엉뚱한 쪽 응답을 받는다
# (`There may be more than one action server`).
#
# **② 고아 노드는 오염된 상태를 계속 발행한다.** 시뮬레이터를 재시작하면 sim 시계가
# 0 으로 돌아가는데, 살아남은 노드의 TF 버퍼에는 **더 큰 타임스탬프의 옛 데이터**가
# 남아 새 데이터가 `TF_OLD_DATA` 로 버려진다. `/scene/objects` 가 **직전 실행에서
# 물체를 놓았던 자리**를 계속 말하게 되고, 그것을 믿은 파지가 빗나간다.
#
# **③ Isaac 은 래퍼를 죽여도 자식이 남는다.** `python.sh` 는 껍데기이고 실제 프로세스는
# `kit/python/bin/python3` 다. 래퍼만 죽이면 시뮬레이터가 그대로 돌면서 GPU 를 잡고
# 있고, 다음에 띄운 인스턴스와 **둘 다 `/clock` 을 발행**해 배속이 음수로 나온다.
#
# **④ `ros2` CLI 데몬이 죽은 노드를 계속 보여준다.** 프로세스가 전부 사라져도
# `ros2 node list` 에 남는다 — 캐시다. 이것 때문에 "왜 안 죽지" 로 시간을 버린다.
#
#
# ── 무엇을 죽이나 ────────────────────────────────────────────────────────────
#
# 이 저장소의 스택과 시뮬레이터로 **한정**한다. 다른 ROS 작업은 건드리지 않는다.
#
#   ros2 launch <이 워크스페이스의 런치>       Isaac / 펑션베이 / mock 계열
#   이 워크스페이스 install/ 아래의 노드 실행 파일
#   런치가 띄우는 표준 노드                     move_group · robot_state_publisher ·
#                                              static_transform_publisher · spawner · rviz2
#   Isaac Sim                                  kit · python.sh
#   ros2 CLI 데몬
#
# ⚠️ **다른 사람과 도메인을 공유하는 기계에서는 쓰지 않는다.** 프로세스 단위로 죽이므로
#    같은 기계의 다른 ROS 작업이 위 패턴에 걸리면 함께 내려간다.
set -uo pipefail

WORKSPACE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DRY_RUN=0
DO_ROS=1
DO_ISAAC=1

while [[ $# -gt 0 ]]; do
    case "$1" in
        --dry-run|-n) DRY_RUN=1; shift ;;
        --isaac)      DO_ROS=0; shift ;;
        --ros)        DO_ISAAC=0; shift ;;
        -h|--help)    sed -n '2,50p' "${BASH_SOURCE[0]}"; exit 0 ;;
        *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
    esac
done

# 이 스크립트 자신과 그 부모 셸은 절대 죽이지 않는다. 패턴에 스크립트 이름이 들어가
# 자기 자신을 잡는 사고가 실제로 있었다.
SELF=$$
readonly SELF

_pids() {   # _pids <설명> <정규식>
    local label="$1" pattern="$2" pids=()
    while read -r pid cmd; do
        [[ "$pid" == "$SELF" || "$pid" == "$PPID" ]] && continue
        [[ "$cmd" == *kill_stack* ]] && continue
        pids+=("$pid")
    done < <(ps -eo pid=,cmd= | grep -E "$pattern" | grep -v grep)
    [[ ${#pids[@]} -eq 0 ]] && return 0
    printf '%-22s %s\n' "$label" "${pids[*]}" >&2
    printf '%s\n' "${pids[@]}"
}

collect() {
    {
        if [[ "$DO_ROS" == "1" ]]; then
            _pids "ros2 launch"    "ros2 launch (robot_control|rdfp) "
            _pids "워크스페이스 노드" "${WORKSPACE}/install/[a-z_]+/lib/"
            _pids "MoveIt/TF/rviz" "/opt/ros/humble/lib/(moveit_ros_move_group|robot_state_publisher|tf2_ros|controller_manager|rviz2)/"
        fi
        if [[ "$DO_ISAAC" == "1" ]]; then
            _pids "Isaac"          "isaacsim/(kit/|python\.sh)"
        fi
    } | sort -un
}

mapfile -t targets < <(collect)

if [[ ${#targets[@]} -eq 0 ]]; then
    echo "정리할 프로세스가 없다."
else
    echo "대상 ${#targets[@]}개: ${targets[*]}"
    if [[ "$DRY_RUN" == "1" ]]; then
        echo "(--dry-run — 아무것도 죽이지 않았다)"
        exit 0
    fi
    # TERM 을 먼저 준다. 런치는 이때 자식을 정리하므로 대부분 여기서 끝난다.
    kill "${targets[@]}" 2>/dev/null
    # 대기 중에는 라벨을 찍지 않는다 — 매 초 같은 목록이 흘러 실제 결과를 덮는다.
    for _ in $(seq 1 10); do
        mapfile -t alive < <(collect 2>/dev/null)
        [[ ${#alive[@]} -eq 0 ]] && break
        sleep 1
    done
    mapfile -t alive < <(collect 2>/dev/null)
    if [[ ${#alive[@]} -gt 0 ]]; then
        echo "SIGTERM 후에도 남음 → SIGKILL: ${alive[*]}"
        kill -9 "${alive[@]}" 2>/dev/null
        sleep 2
    fi
fi

# **데몬을 마지막에 죽인다.** 먼저 죽이면 위에서 `ros2` 를 부를 때 되살아난다.
if [[ "$DRY_RUN" == "0" ]]; then
    pkill -f 'ros2cli\.daemon' 2>/dev/null
    sleep 1
fi

# **`collect` 를 다시 부른다.** 다른 정규식으로 세면 대상이 아닌 것을 '남았다'고
# 보고한다 — rviz2 가 `-d <workspace>/install/...` 인자 때문에 그렇게 걸린 적이 있다.
mapfile -t leftover < <(collect 2>/dev/null)
if [[ ${#leftover[@]} -eq 0 ]]; then
    echo "남은 프로세스: 없음"
else
    echo "남은 프로세스 ${#leftover[@]}개: ${leftover[*]}"
fi
echo
echo "그래프 확인 — 데몬을 지웠으므로 첫 조회는 몇 초 걸린다:"
echo "  ros2 node list"

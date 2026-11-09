#!/usr/bin/env bash
# 키보드 텔레오퍼레이션.
#
# **백엔드를 `backend` 하나로 고른다.** '/' (ready 자세로 이동) 가 쓰는 arm 명령
# 채널이 백엔드마다 다르기 때문이다. 노드에는 기본값이 없어 안 주면 뜨지 않고,
# 이 래퍼가 `TELEOP_BACKEND`(기본 `panda_isaac`)를 넘긴다 (맨 아래).
#
#   ./scripts/run_teleop_keyboard.sh                                  # isaac
#   TELEOP_BACKEND=panda_functionbay ./scripts/run_teleop_keyboard.sh
#   ./scripts/run_teleop_keyboard.sh --ros-args -p backend:=panda_mock      # 인자가 환경변수를 이긴다
#
# ⚠️ 붙은 스택과 다른 백엔드를 주면 계획은 되고 실행만 안 된다 — 펑션베이 스택에 JTC
# 백엔드(mock · isaac)를 주면 FollowJointTrajectory 액션 **서버가 없어** MoveGroup 이
# CONTROL_FAILED(-4) 를 반환한다. 기동 시 JTC 인데 서버가 없으면 에러 로그가 남는다.
#
# ⚠️ 기동하면서 servo 를 시작하므로 **트윈으로 작업하는 동안에는 띄워 두지 않는다** —
# 명령이 없어도 팔이 흐르고 트윈의 MoveIt 실행과 컨트롤러를 두고 싸운다
# (docs/teleop/servo_vs_planned_motion.md §5).
#
# 개별 값을 덮어쓰려면 함께 준다 (프로파일보다 우선한다):
#
#   ./scripts/run_teleop_keyboard.sh --ros-args \
#     -p backend:=panda_functionbay -p arm_command_topic:=/other/cmd
#
# 세션·에피소드 키(< > [ ] 1-9 0)는 session_control 이 있어야 동작한다. 없으면
# 그 키만 비활성이 되고 나머지는 그대로 쓸 수 있다 (기동 로그에 표시된다).
#
# ─── 키보드 자동반복 ──────────────────────────────────────────────────────
#
# **터미널은 '키를 뗐다'를 알려 주지 않는다.** 모션 키는 자동반복으로 들어오는
# 문자가 끊기는 것을 뗀 것으로 읽고 deadman TTL(0.06s)이 만료되면 멈춘다. 그래서
# 자동반복 설정이 조작감을 그대로 결정한다:
#
#   · 자동반복이 **꺼져 있으면** 문자가 한 번만 와서 **누르고 있어도 곧 멈춘다**
#   · 초기 지연이 길면(기본 ~660ms) **홀드 초반이 한 번 끊긴다**
#
# 그래서 실행 동안만 빠른 반복으로 바꾸고 **끝나면 되돌린다.** `xset` 은 X 서버
# 전역 설정이라 그냥 바꿔 두면 에디터·브라우저까지 영향을 받는다.
#
# 되돌릴 때 **두 가지를 다 복원해야 한다** — `xset r off` 는 on/off 만 되돌리고
# delay/rate 는 바뀐 채 남는다 (실측). rate 를 먼저 주고 off 를 나중에 준다.
#
# X 가 없으면(Wayland·순수 SSH) 조용히 건너뛴다 — 그 환경에서는 evdev 기반 입력을
# 봐야 한다 (docs/teleop/clutch_pedal_guide.md 가 선례다).
#
#   TELEOP_KB_RATE=0 ./scripts/run_teleop_keyboard.sh    # 자동반복을 건드리지 않는다

TELEOP_KB_RATE="${TELEOP_KB_RATE:-30}"          # 반복 Hz (0 이면 손대지 않는다)
TELEOP_KB_DELAY="${TELEOP_KB_DELAY:-30}"        # 초기 지연 ms

if [[ "$TELEOP_KB_RATE" != "0" ]] && [[ -n "${DISPLAY:-}" ]] && command -v xset >/dev/null; then
    _old_state=$(xset q 2>/dev/null | awk '/^[[:space:]]*auto repeat:/{print $3}')
    _old_vals=$(xset q 2>/dev/null | awk '/auto repeat delay/{print $4, $7}')

    if [[ -n "$_old_vals" ]]; then
        restore_autorepeat() {
            xset r rate $_old_vals 2>/dev/null          # delay/rate 를 먼저 되돌린다
            [[ "$_old_state" == "off" ]] && xset r off 2>/dev/null   # 그 다음 on/off
            echo "[teleop] 자동반복 복원: $_old_vals (state=$_old_state)"
        }
        trap restore_autorepeat EXIT INT TERM

        xset r rate "$TELEOP_KB_DELAY" "$TELEOP_KB_RATE" 2>/dev/null
        echo "[teleop] 자동반복: delay=${TELEOP_KB_DELAY}ms rate=${TELEOP_KB_RATE}Hz" \
             "(이전 $_old_vals, state=$_old_state — 종료 시 복원)"
    fi
fi

# **`backend` 는 노드에 기본값이 없다** — 어느 스택에 붙는지는 사람이 안다. 짐작하게
# 두면 틀린 짐작이 조용히 통과하고 증상은 "'/' 키가 안 먹는다" 로만 보인다.
# 여기서 기본을 들되 환경변수로 바꾼다. 인자로 직접 준 것이 있으면 그것이 이긴다.
#
#   TELEOP_BACKEND=panda_functionbay ./scripts/run_teleop_keyboard.sh
#   ./scripts/run_teleop_keyboard.sh --ros-args -p backend:=panda_mock_jgpc
TELEOP_BACKEND="${TELEOP_BACKEND:-panda_isaac}"
if [[ "$*" == *"backend:="* ]]; then
    ros2 run rdfp teleop_keyboard "$@"
else
    ros2 run rdfp teleop_keyboard --ros-args -p backend:="$TELEOP_BACKEND" "$@"
fi

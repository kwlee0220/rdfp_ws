#!/usr/bin/env bash
# Isaac 로봇 스택을 띄우고 robot-twin-client 로 조작하기 위한 **선행 조건**을 확인한다.
# 아무것도 띄우지 않고 아무것도 고치지 않는다 — 확인만 하고, 실패마다 고치는 방법을 적는다.
#
#   ./scripts/isaac/is_check_prereq.sh
#
# 확인하는 것 (docs/backends/isaac/isaac_bringup_runbook.md §1 「선행 조건」):
#
#   1. Isaac Sim 6.0      — $ISAAC_ROOT (기본 ~/isaacsim), GPU 드라이버, 내장 ROS 확장
#   2. .rdfprc            — 있는가, 소싱하면 도메인 · RMW 가 Isaac · 트윈과 맞는가
#   3. ROS 패키지         — topic_based_ros2_control · simulation_interfaces · MoveIt panda 자원
#   4. 워크스페이스 빌드  — install/ 이 있는가, **src 와 같은가** (낡은 설치본)
#   5. robot-twin-client  — 체크아웃 · venv · uv.lock 동기화
#
# 4번에서 src 와 install 을 파일 단위로 대조하는 이유: 각 패키지가 소스를 **복사해** 설치하므로
# src 만 고치고 빌드하지 않으면 옛 코드가 돈다. 특히 `isaac_scene.json` 은 Isaac 쪽이 src 를,
# `isaac_scene_state_node` 가 install 을 읽어 **둘이 다른 물체 목록을 본다** — 오류는 안 난다.
#
# 환경변수: ISAAC_ROOT (기본 ~/isaacsim), RDFP_TWIN_CLIENT_DIR
#           (기본 ~/development/mdtpy/robot-twin-client).
#
# 종료 코드: 실패가 없으면 0, 하나라도 있으면 1. 경고는 종료 코드에 넣지 않는다.

# `-e` 는 쓰지 않는다 — 실패해도 나머지 항목을 계속 확인해야 한다. `-u` 는 ROS 의 setup
# 스크립트를 소싱하는 동안만 끈다 (그 스크립트들이 미정의 변수를 참조한다).
set -uo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
ISAAC_ROOT="${ISAAC_ROOT:-$HOME/isaacsim}"
CLIENT_DIR="${RDFP_TWIN_CLIENT_DIR:-$HOME/development/mdtpy/robot-twin-client}"
TWIN_CONFIG="$WS_ROOT/src/robot_twin/config/twins/panda01.yaml"
# Isaac 쪽 RMW — `run_isaac_sim.sh` 가 환경에 값이 없을 때 쓰는 기본값과 같아야 한다.
ISAAC_DEFAULT_RMW="rmw_fastrtps_cpp"

case "${1:-}" in
    -h|--help) sed -n '2,24p' "${BASH_SOURCE[0]}"; exit 0 ;;
    "") ;;
    *) echo "unknown option: $1 (try --help)" >&2; exit 2 ;;
esac

FAILS=0
WARNS=0

section() { printf '\n[%s]\n' "$1"; }
ok()      { printf '  통과  %s\n' "$1"; }
# 첫 인자가 무엇이 틀렸는지, 나머지가 고치는 방법이다.
bad() {
    printf '  실패  %s\n' "$1"; shift
    local line; for line in "$@"; do printf '        → %s\n' "$line"; done
    FAILS=$((FAILS + 1))
}
warn() {
    printf '  경고  %s\n' "$1"; shift
    local line; for line in "$@"; do printf '        → %s\n' "$line"; done
    WARNS=$((WARNS + 1))
}
note()    { printf '  참고  %s\n' "$1"; }

# ── 1. Isaac Sim ──────────────────────────────────────────────────────────────

section "1. Isaac Sim ($ISAAC_ROOT)"
if [[ -x "$ISAAC_ROOT/isaac-sim.sh" && -x "$ISAAC_ROOT/python.sh" ]]; then
    version="$(head -n 1 "$ISAAC_ROOT/VERSION" 2>/dev/null || true)"
    if [[ "$version" == 6.0* ]]; then
        ok "Isaac Sim $version"
    elif [[ -z "$version" ]]; then
        warn "버전을 알 수 없다 (VERSION 파일이 없다)" "이 워크스페이스는 Isaac Sim 6.0 기준으로 검증했다"
    else
        warn "Isaac Sim $version" "이 워크스페이스는 6.0 기준으로 검증했다 — 다른 버전은 sim_side 스크립트가 어긋날 수 있다"
    fi
else
    bad "isaac-sim.sh · python.sh 가 없다" \
        "Isaac Sim 6.0 을 설치하거나, 다른 곳에 있으면 ISAAC_ROOT 를 맞춘다"
fi
# GPU 드라이버. 드라이버 패키지가 갱신되고 재부팅하지 않으면 커널 모듈과 사용자 영역 라이브러리의
# 버전이 갈려 RTX 가 장치를 하나도 못 만든다. 그래도 Isaac 은 `[bringup] PLAY` 까지 찍고 나서
# 죽으므로 증상이 "창이 안 뜬다"뿐이고, 원인은 Kit 로그(`~/isaacsim/kit/logs/`)의
# `NVML_ERROR_LIB_RM_VERSION_MISMATCH` 에만 있다. 2026-10-19 에 실제로 그랬다 — unattended-upgrade 가
# 10-11 에 595.91 → 595.99 로 올렸고 커널 모듈은 9-17 부팅 때의 595.91 이었다.
if ! command -v nvidia-smi >/dev/null 2>&1; then
    bad "nvidia-smi 가 없다 — NVIDIA 드라이버가 설치되지 않았다" "sudo ubuntu-drivers install"
elif gpu="$(timeout 20 nvidia-smi --query-gpu=name,driver_version --format=csv,noheader 2>&1)"; then
    ok "GPU 드라이버 ($(head -n 1 <<<"$gpu"))"
elif grep -qi 'version mismatch' <<<"$gpu"; then
    kmod="$(sed -nE 's/.*Kernel Module +([0-9.]+).*/\1/p' /proc/driver/nvidia/version 2>/dev/null | head -n 1)"
    lib="$(sed -nE 's/.*NVML library version: *([0-9.]+).*/\1/p' <<<"$gpu" | head -n 1)"
    bad "NVIDIA 커널 모듈(${kmod:-?})과 라이브러리(${lib:-?})의 버전이 다르다 — 드라이버가 갱신된 뒤 재부팅하지 않았다" \
        "재부팅한다 (Isaac 의 RTX 가 GPU 장치를 못 만들어 창이 안 뜬다)" \
        "재발 방지: unattended-upgrade 에서 드라이버를 뺀다 — /etc/apt/apt.conf.d/50unattended-upgrades 의" \
        "  Unattended-Upgrade::Package-Blacklist 에 \"nvidia-\"; \"libnvidia-\"; 를 넣는다"
else
    bad "nvidia-smi 가 실패했다: $(head -n 1 <<<"$gpu")" "드라이버 상태를 확인한다 (dmesg | grep -i nvidia)"
fi
# `run_isaac_sim.sh` 가 이 확장의 rclpy 로 ROS 를 띄운다. 없으면 ROS 확장이 조용히 죽는다.
if [[ -d "$ISAAC_ROOT/exts/isaacsim.ros2.core/humble" ]]; then
    ok "내장 ROS 확장 (exts/isaacsim.ros2.core/humble)"
else
    bad "내장 ROS 확장이 없다: $ISAAC_ROOT/exts/isaacsim.ros2.core/humble" \
        "Isaac Sim 설치가 온전한지 확인한다 (ROS 2 브리지 확장이 포함된 배포판이어야 한다)"
fi

# ── 2. .rdfprc ────────────────────────────────────────────────────────────────

section "2. .rdfprc (ROS 환경)"
ros_ready=0
if [[ -f "$WS_ROOT/.rdfprc" ]]; then
    set +u
    # shellcheck source=/dev/null
    source "$WS_ROOT/.rdfprc" >/dev/null 2>&1
    rc=$?
    set -u
    if [[ "$rc" -eq 0 && -n "${ROS_DISTRO:-}" ]]; then
        ok ".rdfprc 소싱 (ROS_DISTRO=$ROS_DISTRO)"
        ros_ready=1
    else
        bad ".rdfprc 를 소싱하지 못했다 (rc=$rc)" \
            "직접 실행해 오류를 본다: source $WS_ROOT/.rdfprc" \
            "~/development/ros/.ros2rc 가 있는지도 본다 — .rdfprc 가 그것을 먼저 소싱한다"
    fi
else
    bad ".rdfprc 가 없다 ($WS_ROOT/.rdfprc)" \
        "git 에 없는 파일이다(평문 DSN 을 담아 .gitignore 로 제외) — 새 머신에서는 손으로 만든다" \
        "형식: docs/python_env_guide.md"
    # 나머지 항목을 확인할 수 있게 ROS 만이라도 올린다.
    for distro in humble jazzy; do
        if [[ -f "/opt/ros/$distro/setup.bash" ]]; then
            set +u
            # shellcheck source=/dev/null
            source "/opt/ros/$distro/setup.bash" >/dev/null 2>&1
            # shellcheck source=/dev/null
            [[ -f "$WS_ROOT/install/setup.bash" ]] && source "$WS_ROOT/install/setup.bash" >/dev/null 2>&1
            set -u
            note "아래 항목은 /opt/ros/$distro 로 확인한다"
            ros_ready=1
            break
        fi
    done
fi

if [[ -f "$WS_ROOT/.rdfprc" && "$ros_ready" == "1" ]]; then
    if [[ "${RDFP_HOME:-}" == "$WS_ROOT" ]]; then
        ok "RDFP_HOME=$RDFP_HOME"
    else
        warn "RDFP_HOME 이 이 워크스페이스가 아니다: ${RDFP_HOME:-<비어 있음>}" \
            ".rdfprc 의 RDFP_HOME 을 $WS_ROOT 로 고친다 — 다르면 다른 워크스페이스의 install/ 을 얹는다"
    fi

    # 트윈 설정의 도메인과 같아야 한다. Isaac 은 터미널 1 의 ROS_DOMAIN_ID(없으면 31)를 그래프에
    # 박으므로, 어긋나면 브리지 토픽이 다른 도메인으로 나가 **에러 없이 아무것도 안 보인다.**
    twin_domain="$(grep -E '^[[:space:]]*domain_id:' "$TWIN_CONFIG" 2>/dev/null \
                   | head -n 1 | sed -E 's/.*domain_id:[[:space:]]*([0-9]+).*/\1/')"
    if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
        bad "ROS_DOMAIN_ID 가 없다" \
            "기본 0 이 되어 Isaac(${twin_domain:-31}) · 트윈과 갈린다 — .ros2rc 가 배포판별로 정한다"
    elif [[ -n "$twin_domain" && "$ROS_DOMAIN_ID" != "$twin_domain" ]]; then
        bad "ROS_DOMAIN_ID=$ROS_DOMAIN_ID — 트윈 설정(twins/panda01.yaml)은 $twin_domain" \
            "같은 값으로 맞춘다. 터미널 1(시뮬레이터)에도 같은 값이 가야 한다"
    else
        ok "ROS_DOMAIN_ID=$ROS_DOMAIN_ID (트윈 설정과 같다)"
    fi

    rmw="${RMW_IMPLEMENTATION:-}"
    if [[ -z "$rmw" || "$rmw" == "$ISAAC_DEFAULT_RMW" ]]; then
        ok "RMW_IMPLEMENTATION=${rmw:-<기본>} (Isaac 쪽 기본 $ISAAC_DEFAULT_RMW 와 같다)"
    else
        warn "RMW_IMPLEMENTATION=$rmw — Isaac 은 따로 주지 않으면 $ISAAC_DEFAULT_RMW 로 뜬다" \
            "터미널 1 에서도 같은 RMW_IMPLEMENTATION 을 export 하고 run_isaac_sim.sh 를 띄운다"
    fi

    if [[ ":${AMENT_PREFIX_PATH:-}:" == *":$WS_ROOT/install/"* ]]; then
        ok "워크스페이스 overlay 가 얹혔다 ($WS_ROOT/install)"
    else
        bad "워크스페이스 overlay 가 안 얹혔다" \
            "colcon build 를 먼저 한다 — install/setup.bash 가 없으면 .rdfprc 가 조용히 건너뛴다"
    fi
fi

# ── 3. ROS 패키지 ─────────────────────────────────────────────────────────────

section "3. ROS 패키지"
if [[ "$ros_ready" != "1" ]] || ! command -v ros2 >/dev/null 2>&1; then
    bad "ros2 명령이 없어 확인하지 못했다" "ROS 2 를 설치하고 .rdfprc 를 맞춘 뒤 다시 돌린다"
else
    # 패키지 이름 · 없을 때의 증상. apt 이름은 `ros-<distro>-` + 밑줄을 하이픈으로 바꾼 것이다.
    pkgs=(
        "topic_based_ros2_control|ros2_control_node 가 하드웨어 플러그인을 못 찾아 컨트롤러 사슬이 서지 않는다"
        "simulation_interfaces|/scene/reset 이 열리지 않는다 — 스택은 정상 기동하므로 reset_scene 을 부를 때야 드러난다"
        "moveit_resources_panda_moveit_config|panda_isaac.launch.py 가 MoveIt 설정을 못 찾아 뜨지 않는다"
    )
    for entry in "${pkgs[@]}"; do
        pkg="${entry%%|*}"; why="${entry#*|}"
        if prefix="$(ros2 pkg prefix "$pkg" 2>/dev/null)"; then
            ok "$pkg ($prefix)"
        else
            bad "$pkg 가 없다 — $why" "sudo apt install ros-${ROS_DISTRO:-humble}-${pkg//_/-}"
        fi
    done
fi

# ── 4. 워크스페이스 빌드 ──────────────────────────────────────────────────────

section "4. 워크스페이스 빌드 ($WS_ROOT/install)"

# 두 디렉터리 아래 파일을 대조해 src 쪽 기준으로 다르거나 없는 파일 이름을 출력한다.
#   $1 src 디렉터리   $2 설치 디렉터리   $3 이름 패턴(find -name)
_diff_tree() {
    local src="$1" dst="$2" pattern="$3" f rel
    [[ -d "$src" ]] || return 0
    while IFS= read -r -d '' f; do
        rel="${f#"$src"/}"
        cmp -s "$f" "$dst/$rel" || printf '%s\n' "$rel"
    done < <(find "$src" -name "$pattern" -type f -not -path '*/__pycache__/*' -print0)
}

# 설치본에만 남은 파일 — colcon 은 src 에서 지운 파일을 install 에서 치우지 않는다.
# `__pycache__` 는 뺀다 — `ros2 launch` 가 설치본의 launch 파일을 import 하며 그 옆에 만든다.
_leftovers() {
    local src="$1" dst="$2" f rel
    [[ -d "$dst" ]] || return 0
    while IFS= read -r -d '' f; do
        rel="${f#"$dst"/}"
        [[ -e "$src/$rel" ]] || printf '%s\n' "$rel"
    done < <(find "$dst" -type f -not -path '*/__pycache__/*' -print0)
}

# 두 JSON 이 주석 키(`_` 로 시작)를 빼고도 다른가. 다르면 0, 같으면 1 을 돌려준다.
_scene_differs() {
    [[ -f "$2" ]] || return 0     # 설치본에 없으면 다른 것이다
    python3 - "$1" "$2" <<'EOF'
import json
import sys


def strip(value):
    if isinstance(value, dict):
        return {k: strip(v) for k, v in value.items() if not k.startswith('_')}
    if isinstance(value, list):
        return [strip(v) for v in value]
    return value


with open(sys.argv[1], encoding='utf-8') as a, open(sys.argv[2], encoding='utf-8') as b:
    sys.exit(0 if strip(json.load(a)) != strip(json.load(b)) else 1)
EOF
}

if [[ ! -f "$WS_ROOT/install/setup.bash" ]]; then
    bad "install/setup.bash 가 없다 — 빌드한 적이 없다" \
        "cd $WS_ROOT && colcon build   (src/ 가 아니라 워크스페이스 루트에서)"
else
    ok "install/setup.bash"
    stale_pkgs=()
    for pkg in rdfp_msgs panda_ftsensor_robotiq robot_control robot_twin rdfp; do
        if [[ ! -d "$WS_ROOT/install/$pkg" ]]; then
            bad "$pkg 가 설치되지 않았다" "colcon build --packages-select $pkg"
            continue
        fi
        share="$WS_ROOT/install/$pkg/share/$pkg"
        changed=()
        case "$pkg" in
            rdfp_msgs)
                for sub in msg srv; do
                    while IFS= read -r rel; do changed+=("$sub/$rel"); done \
                        < <(_diff_tree "$WS_ROOT/src/$pkg/$sub" "$share/$sub" '*.*')
                done ;;
            panda_ftsensor_robotiq)
                # meshes 는 뺀다 — 크고, 고칠 일이 없는 벤더 자산이다.
                for sub in urdf srdf config; do
                    while IFS= read -r rel; do changed+=("$sub/$rel"); done \
                        < <(_diff_tree "$WS_ROOT/src/$pkg/$sub" "$share/$sub" '*')
                done ;;
            *)
                site="$(compgen -G "$WS_ROOT/install/$pkg/lib/python3*/site-packages/$pkg" | head -n 1)"
                if [[ -z "$site" ]]; then
                    bad "$pkg 의 파이썬 모듈이 설치되지 않았다" "colcon build --packages-select $pkg"
                    continue
                fi
                while IFS= read -r rel; do changed+=("$rel"); done \
                    < <(_diff_tree "$WS_ROOT/src/$pkg/$pkg" "$site" '*.py')
                # 대조 범위는 각 setup.py 의 data_files 와 같다 — launch 는 `*.py` 만 설치된다
                # (README 등은 설치되지 않으므로 대조하면 늘 '다르다'로 나온다).
                while IFS= read -r rel; do changed+=("config/$rel"); done \
                    < <(_diff_tree "$WS_ROOT/src/$pkg/config" "$share/config" '*')
                while IFS= read -r rel; do changed+=("launch/$rel"); done \
                    < <(_diff_tree "$WS_ROOT/src/$pkg/launch" "$share/launch" '*.py')
                left=()
                for sub in config launch; do
                    while IFS= read -r rel; do left+=("$sub/$rel"); done \
                        < <(_leftovers "$WS_ROOT/src/$pkg/$sub" "$share/$sub")
                done
                if [[ "${#left[@]}" -gt 0 ]]; then
                    warn "$pkg: 설치본에만 남은 파일 ${#left[@]}개 (${left[*]:0:3})" \
                        "src 에서 지운 파일이다 — 재빌드로는 안 치워진다: rm -rf install/$pkg build/$pkg 후 다시 빌드한다"
                fi ;;
        esac
        if [[ "${#changed[@]}" -eq 0 ]]; then
            ok "$pkg — 설치본이 src 와 같다"
        else
            stale_pkgs+=("$pkg")
            hint=()
            for rel in "${changed[@]}"; do
                [[ "$rel" == "config/isaac_scene.json" ]] || continue
                # 주석 키(`_` 로 시작)만 다르면 동작은 같다 — 내용이 다를 때만 크게 알린다.
                if _scene_differs "$WS_ROOT/src/$pkg/$rel" "$share/$rel"; then
                    hint=("isaac_scene.json 내용이 다르다 — Isaac 은 src 를, isaac_scene_state_node 는 install 을 읽어 서로 다른 물체 목록을 본다")
                else
                    hint=("isaac_scene.json 은 주석(_ 키)만 다르다 — 물체 목록은 같다")
                fi
            done
            bad "$pkg — src 와 다른 파일 ${#changed[@]}개 (예: ${changed[*]:0:3})" \
                "${hint[@]}" "빌드하지 않으면 옛 코드가 돈다"
        fi
    done
    if [[ "${#stale_pkgs[@]}" -gt 0 ]]; then
        note "한 번에: cd $WS_ROOT && colcon build --packages-select ${stale_pkgs[*]}"
    fi
fi

# ── 5. robot-twin-client ──────────────────────────────────────────────────────

section "5. robot-twin-client ($CLIENT_DIR)"
if [[ ! -d "$CLIENT_DIR" ]]; then
    bad "체크아웃이 없다" \
        "git clone https://github.com/kwlee0220/robot-twin-client.git $CLIENT_DIR" \
        "다른 곳에 있으면 RDFP_TWIN_CLIENT_DIR 로 알려 준다"
else
    ok "체크아웃"
    if [[ ! -x "$CLIENT_DIR/.venv/bin/python" ]]; then
        bad "venv 가 없다" "cd $CLIENT_DIR && uv sync"
    elif ! "$CLIENT_DIR/.venv/bin/python" -c 'import robot_twin_client' >/dev/null 2>&1; then
        bad "venv 에서 robot_twin_client 를 import 하지 못한다" "cd $CLIENT_DIR && uv sync"
    else
        ok "venv 에서 robot_twin_client import"
        missing=()
        for script in robot-twin-pick robot-twin-stack; do
            [[ -x "$CLIENT_DIR/.venv/bin/$script" ]] || missing+=("$script")
        done
        if [[ "${#missing[@]}" -eq 0 ]]; then
            ok "콘솔 스크립트 (robot-twin-pick · robot-twin-stack)"
        else
            bad "콘솔 스크립트가 없다: ${missing[*]}" "cd $CLIENT_DIR && uv sync"
        fi
    fi
    if command -v uv >/dev/null 2>&1; then
        # `--inexact` 는 venv 에 더 있는 것(mcp extra 등)을 문제 삼지 않는다 — 고치지 않고 보기만 한다.
        if (cd "$CLIENT_DIR" && uv sync --check --inexact >/dev/null 2>&1); then
            ok "venv 가 uv.lock 과 맞다"
        else
            warn "venv 가 uv.lock 과 다르다 — 의존성이 바뀐 뒤 sync 하지 않았다" "cd $CLIENT_DIR && uv sync"
        fi
    else
        warn "uv 가 없다 — README 의 'uv run …' 대신 .venv/bin/<스크립트> 로 실행한다" \
            "설치: https://docs.astral.sh/uv/"
    fi
fi

# ── 참고: 이미 떠 있는 Isaac ──────────────────────────────────────────────────
#
# 선행 조건은 아니지만, 다음 단계가 `run_isaac_sim.sh` 이므로 두 번 띄우는 실수를 미리 알린다.
# 판별은 `run_isaac_sim.sh` 와 같다 — argv 앞 세 원소만 보고 경로를 '언급만' 하는 프로세스는 안 잡는다.
running=()
for dir in /proc/[0-9]*; do
    pid="${dir#/proc/}"
    [[ "$pid" == "$$" ]] && continue
    i=0
    while IFS= read -r -d '' arg; do
        if [[ "$arg" == *"/sim_side/headless_bringup.py" || "$arg" == *"/isaac-sim.sh" ]]; then
            running+=("$pid"); break
        fi
        i=$((i + 1)); [[ "$i" -ge 3 ]] && break
    # `2>/dev/null` 을 `<` 앞에 둔다 — 순서가 반대면 그 사이 끝난 프로세스의 cmdline 을 못 연
    # 오류가 숨겨지기 전에 찍힌다.
    done 2>/dev/null < "$dir/cmdline"
done
if [[ "${#running[@]}" -gt 0 ]]; then
    section "참고"
    note "Isaac 이 이미 떠 있다 (PID ${running[*]}) — run_isaac_sim.sh 를 다시 띄우지 않는다"
fi

# ── 요약 ──────────────────────────────────────────────────────────────────────

echo
if [[ "$FAILS" -eq 0 ]]; then
    echo "선행 조건 통과 (경고 $WARNS개) — isaac_bringup_runbook.md §2 터미널 1 부터 진행한다"
    exit 0
fi
echo "선행 조건 실패 ${FAILS}개 · 경고 ${WARNS}개 — 위의 → 를 따라 고친 뒤 다시 돌린다"
exit 1

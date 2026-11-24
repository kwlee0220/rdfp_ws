#!/usr/bin/env bash
# 데이터셋 관리 GUI (`rdfp_gui`) 를 띄운다.
#
#   ./scripts/run_dataset_gui.sh                       # config/dataset_config.yaml
#   ./scripts/run_dataset_gui.sh --config <yaml>
#
# 설치본이 있으면 `ros2 run rdfp_gui dataset_gui` 로, 없으면 소스 경로를 PYTHONPATH 에 얹어 모듈로
# 띄운다. 적재(`import`)는 bag 에 이미지 토픽이 없으면 ROS 없이 돈다(2026-11-23). 서브프로세스가
# `ros2` 를 못 찾으면 `python -m rdfp.dataset.*` 로 대신 부른다. PySide6 는 pip 전용이다:
# pip install --user PySide6-Essentials
#
# 다른 파이썬(별도 venv)으로 띄우려면 RDFP_GUI_PYTHON 을 준다 — 그때는 소스 경로로 띄운다:
#   RDFP_GUI_PYTHON=~/.venvs/rdfp-gui/bin/python ./scripts/run_dataset_gui.sh
# LeRobot v3 export 는 lerobot 이 있는 파이썬이 따로 필요하다 — RDFP_LEROBOT_PYTHON (rdfp_gui README).
set -euo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
export RDFP_HOME="${RDFP_HOME:-$root}"
cd "$root"
# Qt 6.5+ 의 xcb(X11) 플러그인은 libxcb-cursor0 가 없으면 로드되지 않는다 — 오류가 길고 원인이 묻힌다.
# ldconfig 는 /usr/sbin 에 있어 일반 사용자 PATH 에 없을 수 있으므로 경로를 직접 쓴다.
_has_xcb_cursor() {
    local ldc
    for ldc in /sbin/ldconfig /usr/sbin/ldconfig; do
        [ -x "$ldc" ] && "$ldc" -p 2>/dev/null | grep -q 'libxcb-cursor\.so\.0' && return 0
    done
    compgen -G '/usr/lib/*/libxcb-cursor.so.0' >/dev/null || compgen -G '/lib/*/libxcb-cursor.so.0' >/dev/null
}
if [ "${XDG_SESSION_TYPE:-}" != "wayland" ] && [ -n "${DISPLAY:-}" ]; then
    if ! _has_xcb_cursor; then
        echo "run_dataset_gui: libxcb-cursor0 is missing (Qt xcb plugin needs it): sudo apt install libxcb-cursor0" >&2
        exit 1
    fi
    # X11 세션에서는 wayland 시도(실패 경고)를 건너뛰고 xcb 로 바로 간다. 사용자가 정한 값은 그대로 둔다.
    export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
fi
if [ -z "${RDFP_GUI_PYTHON:-}" ] && command -v ros2 >/dev/null 2>&1 \
        && [ -x "$root/install/rdfp_gui/lib/rdfp_gui/dataset_gui" ]; then
    exec ros2 run rdfp_gui dataset_gui "$@"
fi
python="${RDFP_GUI_PYTHON:-python3}"
if ! command -v "$python" >/dev/null 2>&1 && [ ! -x "$python" ]; then
    echo "run_dataset_gui: python not found: $python (RDFP_GUI_PYTHON)" >&2
    exit 1
fi
export PYTHONPATH="$root/src/rdfp_gui:$root/src/rdfp${PYTHONPATH:+:$PYTHONPATH}"
exec "$python" -m rdfp_gui.main "$@"

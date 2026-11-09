#!/usr/bin/env bash
# 데이터셋 관리 GUI (`rdfp_gui`) 를 띄운다.
#
#   ./scripts/run_dataset_gui.sh                       # config/dataset_config.yaml
#   ./scripts/run_dataset_gui.sh --config <yaml>
#
# 설치본이 있으면 `ros2 run rdfp_gui dataset_gui` 로, 없으면 소스 경로를 PYTHONPATH 에 얹어 모듈로
# 띄운다 (조회만 할 때 — 적재 · export 는 서브프로세스가 `ros2 run rdfp …` 를 부르므로 ROS 를 소싱한
# 셸이어야 한다). PySide6 는 pip 전용이다: pip install --user PySide6-Essentials
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
if command -v ros2 >/dev/null 2>&1 && [ -x "$root/install/rdfp_gui/lib/rdfp_gui/dataset_gui" ]; then
    exec ros2 run rdfp_gui dataset_gui "$@"
fi
export PYTHONPATH="$root/src/rdfp_gui:$root/src/rdfp${PYTHONPATH:+:$PYTHONPATH}"
exec python3 -m rdfp_gui.main "$@"

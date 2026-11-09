#!/usr/bin/env bash
set -euo pipefail
manager_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
manager_python="${RDFP_MANAGER_PYTHON:-${manager_root}/.venv-dataset-manager/bin/python}"
if [[ ! -x "$manager_python" ]]; then
    manager_python="${RDFP_MANAGER_PYTHON:-python3}"
fi
# X11 sessions should not try an unavailable Wayland compositor. Preserve an
# explicitly selected Qt platform (including offscreen for tests).
if [[ -z "${QT_QPA_PLATFORM:-}" && -n "${DISPLAY:-}" &&
      ( "${XDG_SESSION_TYPE:-}" == "x11" || -z "${WAYLAND_DISPLAY:-}" ) ]]; then
    export QT_QPA_PLATFORM=xcb
fi
export PYTHONPATH="${manager_root}/src/rdfp_gui_codex:${manager_root}/src/rdfp${PYTHONPATH:+:${PYTHONPATH}}"
exec "$manager_python" -m rdfp_gui_codex.manager_cmd --config "$manager_root/config/dataset_config.yaml" "$@"

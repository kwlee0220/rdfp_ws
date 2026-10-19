#!/usr/bin/env bash
# rosbag2 관련 명령의 통합 진입점이다.
#
#   ./scripts/rosbag.sh <command> [options]
#
# 처리 로직은 다시 쓰지 않고 기존 구현에 넘긴다:
#   record                        → scripts/record_rosbag.sh (옵션을 환경변수로 바꿔 넘긴다)
#   import                        → ros2 run rdfp import
#   list-topics · list-episodes · export · delete · clear
#                                 → ros2 run rdfp rosbag <command>
#
# **모든 위임은 `exec` 로 한다.** 녹화 프로세스가 이 스크립트의 PID 를 그대로 이어받아야
# `kill -INT <PID>` 가 `ros2 bag record` 에 닿아 `metadata.yaml` 이 남는다. 중간에 bash 가
# 남아 있으면 신호가 거기서 멈추고, 결국 SIGKILL 로 끝난 세션은 적재에서 조용히 빠진다
# (`run_collect_isaac.sh` · `run_collect_functionbay.sh` 가 이 PID 로 녹화를 끈다).
#
# **넘기는 명령의 옵션은 여기서 검사하지 않는다.** 옵션 목록을 bash 와 argparse 두 곳에
# 두면 언젠가 어긋나므로, 이 스크립트는 명령 이름과 `record` 의 옵션만 보고 나머지는
# 받는 쪽에 그대로 넘긴다.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

err() { echo "[ERROR] $*" >&2; }

usage_error() {
  err "$*"
  echo "Run 'scripts/rosbag.sh help' for usage." >&2
  exit 2
}

require_ros2() {
  if ! command -v ros2 >/dev/null 2>&1; then
    err "ros2 command not found. Source ROS 2 first ('rdfp_env')."
    exit 1
  fi
}

usage() {
  cat <<'EOF'
Usage: scripts/rosbag.sh <command> [options]

rosbag2 commands for the RDFP collection workflow.

Commands:
  record          Record the listed topics to MCAP. Stop with Ctrl+C.
  list-topics     Summarize the topics recorded under the bag root.
  list-episodes   List the episodes found in the bags.
  import          Load bags into the DB over the whole range of a dataset config.
  export          Load only the selected episodes into the DB.
  delete          Remove the selected episodes from the bags (asks first).
  clear           Delete all session data under the bag root (asks first).
  help [command]  Show this help, or the options of a command.

Bag root: --rosbag-dir DIR or the RDFP_ROSBAG_DIR environment variable.
There is no built-in default; 'rdfp_env' sets RDFP_ROSBAG_DIR.
Options of commands other than 'record' are passed through unchanged.
EOF
}

record_usage() {
  cat <<'EOF'
Usage: scripts/rosbag.sh record [options]

Record the topics listed in a topics file with 'ros2 bag record' (MCAP).
Each run creates <root>/<YYYY-MM-DD>/session_<YYYY-MM-DD_HH-MM-SS>/.
Stop with Ctrl+C (SIGINT) so that metadata.yaml is written.

Options:
  --topics-file FILE     Topics to record, one per line.
                         Default: config/recording_topics.list (env RECORDING_TOPICS_FILE).
                         A relative path is taken from the current directory.
  --rosbag-dir DIR       Bag root. Required: this or env RDFP_ROSBAG_DIR.
  --split-duration SEC   Start a new file every SEC seconds (default 3600,
                         env SPLIT_DURATION_SEC). This is not a total time limit.
  --config FILE          Dataset config that 'import' will use later. Only used
                         for the check below.
                         Default: $RDFP_HOME/config/dataset_config.yaml
  -h, --help             Show this help.

Before recording, warns when that dataset config would not import some of the
recorded topics, or would look for bags under a different root.
EOF
}

# 녹화를 시작하기 전에, 나중에 적재할 설정이 이 녹화를 제대로 읽을지 본다.
# 어긋나면 오류가 아니라 '조용히 빠짐'으로 나타나는 두 가지만 경고한다:
#   ① 녹화할 토픽 중 적재 설정의 토픽 목록에 없는 것 — 녹화는 되지만 DB 에 안 들어간다
#   ② 적재 설정의 rosbag_dir 이 녹화 루트와 다르다 — 적재가 이 녹화를 찾지 못한다
# 설정 해석은 적재와 같은 코드(`rdfp.dataset.config`)로 한다 — `${VAR:-기본값}` 확장을
# 여기서 따로 흉내 내면 두 구현이 어긋난다. 적재 때와 같은 환경으로 평가해야 하므로
# `--rosbag-dir` 를 환경변수로 내보내기 **전에** 부른다. 점검은 녹화를 막지 않는다.
check_against_import_config() {
  local config="$1" topics_file="$2" rosbag_dir="$3"
  if [ ! -f "${config}" ]; then
    echo "[WARN] Dataset config not found, skipping the import check: ${config}" >&2
    return 0
  fi
  # 목록 파일이 없으면 record_rosbag.sh 가 오류로 알린다.
  [ -f "${topics_file}" ] || return 0

  python3 - "${config}" "${topics_file}" "${rosbag_dir}" <<'PY' || true
import os
import sys

cfg_path, topics_path, root = sys.argv[1:4]
try:
    from rdfp.dataset.config import load_dataset_config
    cfg = load_dataset_config(cfg_path)
except Exception as e:  # 점검은 녹화를 막지 않는다
    print(f'[WARN] Import check skipped ({cfg_path}): {e}', file=sys.stderr)
    sys.exit(0)

# record_rosbag.sh 와 같은 규칙: 앞뒤 공백 제거, 빈 줄 · `#` 주석 줄 무시.
with open(topics_path, encoding='utf-8') as f:
    recorded = [s for s in (line.strip() for line in f) if s and not s.startswith('#')]

wanted = set(cfg.effective_topics())
if wanted:
    dropped = [t for t in recorded if t not in wanted]
    if dropped:
        print(f'[WARN] Recorded but NOT imported by {cfg_path}:', file=sys.stderr)
        for t in dropped:
            print(f'[WARN]   - {t}', file=sys.stderr)

if os.path.realpath(cfg.rosbag_dir) != os.path.realpath(root):
    print(f'[WARN] {cfg_path} reads bags from {cfg.rosbag_dir}, '
          f'but this run records to {root}.', file=sys.stderr)
    print('[WARN] Importing with that config will not find this recording.', file=sys.stderr)
PY
}

cmd_record() {
  # 우선순위: 옵션 > 환경변수 > 기본값. 환경변수로 쓰던 옛 사용법도 그대로 통한다.
  local topics_file="${RECORDING_TOPICS_FILE:-${SCRIPT_DIR}/../config/recording_topics.list}"
  local rosbag_dir="${RDFP_ROSBAG_DIR:-}"
  local split="${SPLIT_DURATION_SEC:-}"
  local config="" config_given=0

  while [ "$#" -gt 0 ]; do
    case "$1" in
      -h|--help)
        record_usage
        exit 0 ;;
      --topics-file|--rosbag-dir|--split-duration|--config)
        if [ "$#" -lt 2 ] || [[ "$2" == -* ]]; then
          usage_error "record: $1 requires a value"
        fi
        case "$1" in
          --topics-file)    topics_file="$2" ;;
          --rosbag-dir)     rosbag_dir="$2" ;;
          --split-duration) split="$2" ;;
          --config)         config="$2"; config_given=1 ;;
        esac
        shift 2 ;;
      --topics-file=*)    topics_file="${1#*=}"; shift ;;
      --rosbag-dir=*)     rosbag_dir="${1#*=}"; shift ;;
      --split-duration=*) split="${1#*=}"; shift ;;
      --config=*)         config="${1#*=}"; config_given=1; shift ;;
      *)
        usage_error "record: unknown option '$1'" ;;
    esac
  done

  if [ -z "${rosbag_dir}" ]; then
    usage_error "record: bag root is not set. Pass --rosbag-dir or set RDFP_ROSBAG_DIR ('rdfp_env' sets it)."
  fi
  if [ -n "${split}" ] && ! [[ "${split}" =~ ^[1-9][0-9]*$ ]]; then
    usage_error "record: --split-duration must be a positive integer (seconds): '${split}'"
  fi
  # 기본 설정 경로는 `import` 와 같은 규칙이다 (`rdfp.dataset.cli_common.DEFAULT_CONFIG_FILE_PATH`).
  if [ -z "${config}" ]; then
    config="${RDFP_HOME:-.}/config/dataset_config.yaml"
  elif [ "${config_given}" = 1 ] && [ ! -f "${config}" ]; then
    usage_error "record: dataset config not found: ${config}"
  fi

  check_against_import_config "${config}" "${topics_file}" "${rosbag_dir}"

  export RECORDING_TOPICS_FILE="${topics_file}"
  export RDFP_ROSBAG_DIR="${rosbag_dir}"
  if [ -n "${split}" ]; then
    export SPLIT_DURATION_SEC="${split}"
  fi
  exec "${SCRIPT_DIR}/record_rosbag.sh"
}

# 적재는 bag 위치를 설정 파일의 `rosbag_dir` 에서 읽는다. 기본 설정은
# `${RDFP_ROSBAG_DIR:-$HOME/rosbag}` 이라 환경변수가 없으면 녹화와 다른 곳을 조용히 본다.
# 그래서 `--config` 없이(기본 설정으로) 실행할 때는 RDFP_ROSBAG_DIR 을 요구한다.
# `--config` 를 주면 그 설정을 고른 것으로 보고 묻지 않는다.
cmd_import() {
  local a config_given=0
  for a in "$@"; do
    case "$a" in
      -h|--help)
        require_ros2
        exec ros2 run rdfp import --help ;;
      --config|--config=*)
        config_given=1 ;;
    esac
  done
  if [ "${config_given}" = 0 ] && [ -z "${RDFP_ROSBAG_DIR:-}" ]; then
    usage_error "import: RDFP_ROSBAG_DIR is not set, so the default dataset config would look for bags under its own fallback (\$HOME/rosbag). Run 'rdfp_env', or pass --config."
  fi
  require_ros2
  exec ros2 run rdfp import "$@"
}

if [ "$#" -eq 0 ]; then
  usage >&2
  exit 2
fi

cmd="$1"
shift

case "${cmd}" in
  help|-h|--help)
    # `help record` 는 `record --help` 와 같다. 인자가 또 help 류면 그냥 전체 도움말 —
    # 그대로 넘기면 자기 자신을 끝없이 다시 부른다.
    case "${1:-}" in
      ""|help|-h|--help) usage ;;
      *) exec "${BASH_SOURCE[0]}" "$1" --help ;;
    esac ;;
  record)
    cmd_record "$@" ;;
  import)
    cmd_import "$@" ;;
  list-topics|list-episodes|export|delete|clear)
    require_ros2
    exec ros2 run rdfp rosbag "${cmd}" "$@" ;;
  *)
    usage_error "unknown command '${cmd}'" ;;
esac

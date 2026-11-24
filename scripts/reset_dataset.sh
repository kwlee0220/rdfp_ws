#!/usr/bin/env bash
# 수집 데이터를 **전부** 지운다 — 운영 가이드 §8.6.
#
# 지우는 것: DB(대상 스키마의 적재 테이블 삭제 후 재생성) · 적재 대기 bag(`rosbag_dir`) ·
# 보관 bag(`$RDFP_DATA_ROOT/rosbag_imported`) · 레코더 출력(`recordings_dir`, 설정했을 때만) ·
# 적재가 만든 영상 · 로그(`output_mp4_dir`) · 학습 샘플(`$RDFP_DATA_ROOT/datasets`).
#
# `scripts/clear_dataset.sh` 와 달리 **적재에 쓴 설정(--config)** 의 DB · 스키마 · 경로를 쓴다.
#
# 순서: 경로 검사 → 녹화 종료 확인 → 삭제 대상 표시(서버 · DB · 스키마 · 실제 경로) →
#       확인 입력(`delete`) → 삭제 → 삭제 확인(에피소드 0 건 · 남은 파일 0).
# 한 단계라도 실패하면 거기서 멈춘다.
#
# 사용법 (`.rdfprc` 를 소싱한 셸, 수집 스택과 `ros2 bag record` 를 먼저 내린다):
#   ./scripts/reset_dataset.sh --config <적재에 쓴 설정> --check-only   # 대상만 본다
#   ./scripts/reset_dataset.sh --config <적재에 쓴 설정>                # 확인 후 삭제
#   ./scripts/reset_dataset.sh --config <적재에 쓴 설정> --yes          # 비대화형
set -uo pipefail

usage() {
    sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'
    exit "${1:-0}"
}

CFG=""
CHECK_ONLY=0
YES=0
while [[ $# -gt 0 ]]; do
    case "$1" in
        --config) CFG="${2:-}"; shift 2 ;;
        --check-only) CHECK_ONLY=1; shift ;;
        --yes) YES=1; shift ;;
        -h|--help) usage 0 ;;
        *) echo "[ERROR] unknown option: $1" >&2; usage 2 ;;
    esac
done
[[ -n "$CFG" ]] || { echo "[ERROR] --config is required (the dataset config used for import)" >&2; exit 2; }
[[ -n "${RDFP_DATA_ROOT:-}" ]] || { echo "[ERROR] RDFP_DATA_ROOT is not set - source .rdfprc first" >&2; exit 2; }

# 설정에서 경로를 읽는다. 심볼릭 링크를 풀어 검사 · 삭제 · 확인이 같은 경로를 쓰게 하고,
# / · $HOME · 데이터 루트 자체나 그 상위를 가리키면 거부한다. 실패하면 아무 변수도 바꾸지 않는다.
load_paths() {
    local out
    out="$(python3 - "$CFG" "$RDFP_DATA_ROOT" <<'EOF'
import os, shlex, sys
from rdfp.dataset.config import load_dataset_config
cfg = load_dataset_config(sys.argv[1])
home = os.path.realpath(os.path.expanduser('~'))
root = sys.argv[2]
if not os.path.isabs(root) or os.path.realpath(root) in {'/', home}:
    sys.exit(f'RDFP_DATA_ROOT={root!r}: must be absolute and must not be / or $HOME')
root = os.path.realpath(root)
paths = {'BAG_DIR': cfg.rosbag_dir, 'VID_DIR': cfg.output_mp4_dir, 'REC_DIR': cfg.recordings_dir,
         'IMP_DIR': os.path.join(root, 'rosbag_imported'), 'DS_DIR': os.path.join(root, 'datasets')}
out = {}
for k, v in paths.items():
    if not v:
        if k == 'REC_DIR':                       # recordings_dir 은 생략할 수 있다 — 삭제 목록에서 뺀다
            out[k] = ''
            continue
        sys.exit(f'{k}: path is not set in the config')
    real = os.path.realpath(v)
    # 내용 전체를 지우는 대상이므로 / · $HOME · 데이터 루트 자체와 그 상위 경로를 거부한다
    if not os.path.isabs(v) or any(real == p or p.startswith(real.rstrip('/') + '/')
                                   for p in ('/', root, home)):
        sys.exit(f'{k}={v!r} (-> {real}): must be absolute and must not be /, $HOME, '
                 f'the data root or one of their parents')
    out[k] = real
for k, v in out.items():
    print(f'{k}={shlex.quote(v)}')
EOF
)" || return 1
    eval "$out"
    DIRS=("$BAG_DIR" "$IMP_DIR" "$VID_DIR" "$DS_DIR")
    [[ -n "$REC_DIR" ]] && DIRS+=("$REC_DIR")
    return 0
}

# 접속한 서버 · DB · 스키마와 에피소드 수. after 는 0 건이 아니면 1 을 돌려준다.
show_db() {
    python3 - "$CFG" "$1" <<'EOF'
import sys
from rdfp.dataset.config import load_dataset_config
from rdfp.dataset.db.connection import open_connection
with open_connection(load_dataset_config(sys.argv[1]).db) as c:
    schema = c.execute('select current_schema()').fetchone()[0]
    n = c.execute('select count(*) from sessions').fetchone()[0]
    print(f'DB {c.info.host}:{c.info.port}/{c.info.dbname} - schema {schema} - episodes {n}')
sys.exit(1 if sys.argv[2] == 'after' and n != 0 else 0)
EOF
}

# 디렉터리마다 실제 경로 · 상태 · 크기(before) 또는 남은 항목 수(after).
# 없는 경로(FileNotFoundError)만 부재로 치고, 권한 오류 등은 'unreadable' 로 1 을 돌려준다.
# 디렉터리를 가리키는 심볼릭 링크도 남은 항목으로 센다(링크 대상은 따라가지 않는다).
inspect_dirs() {
    python3 - "$1" "${DIRS[@]}" <<'EOF'
import os, stat, sys
mode, dirs, failed = sys.argv[1], sys.argv[2:], False
for d in dirs:
    try:
        st = os.stat(d)
    except FileNotFoundError:
        print(f'{"absent" if mode == "before" else "deleted":<16}{d}')
        continue
    except OSError as e:
        print(f'{"unreadable":<16}{d} - {e}'); failed = True
        continue
    if not stat.S_ISDIR(st.st_mode):
        print(f'{"not a directory":<16}{d}'); failed = True
        continue
    errors, files, size = [], 0, 0
    for top, subdirs, names in os.walk(d, onerror=errors.append):
        for n in names + [sd for sd in subdirs if os.path.islink(os.path.join(top, sd))]:
            try:
                size += os.lstat(os.path.join(top, n)).st_size; files += 1
            except OSError as e:
                errors.append(e)
    if errors:                                   # 일부를 읽지 못하면 크기 · 개수를 믿을 수 없다
        print(f'{"unreadable":<16}{d} - {errors[0]}'); failed = True
    elif mode == 'before':
        print(f'{"present":<16}{d}  ({files} entries, {size / 2**20:.1f} MiB)')
    else:
        print(f'{"left " + str(files):<16}{d}'); failed |= files > 0
sys.exit(1 if failed else 0)
EOF
}

recording_running() { pgrep -f 'ros2 bag record' >/dev/null; }

# 숨김 파일까지 비운다. 없는 디렉터리는 오류로 치지 않는다.
empty_dir() { [[ -d "$1" ]] || return 0; find "$1" -mindepth 1 -delete; }

# ---- 확인 ------------------------------------------------------------------
load_paths || { echo "[ERROR] cannot read the config or a path is invalid - aborting" >&2; exit 1; }
if recording_running; then
    echo "[ERROR] ros2 bag record is still running - stop the collection wrapper or recorder first:" >&2
    pgrep -af 'ros2 bag record' >&2
    exit 1
fi
echo "== targets"
show_db before && inspect_dirs before || { echo "[ERROR] pre-check failed - nothing was deleted" >&2; exit 1; }
[[ -d "$BAG_DIR" ]] || echo "note: rosbag_dir does not exist - rosbag clear will be skipped (fix the config if this is a typo)"
[[ "$CHECK_ONLY" == 1 ]] && { echo "pre-check passed (--check-only: nothing was deleted)"; exit 0; }

if [[ "$YES" != 1 ]]; then
    if [[ ! -t 0 ]]; then
        echo "[ERROR] not a terminal - pass --yes to delete without a prompt" >&2
        exit 2
    fi
    read -r -p "Type 'delete' to delete everything listed above: " answer
    [[ "$answer" == delete ]] || { echo "aborted - nothing was deleted"; exit 1; }
fi

# ---- 삭제 -------------------------------------------------------------------
# 확인 입력을 받는 사이에 녹화가 다시 시작됐을 수 있다 — 한 번 더 본다.
recording_running && { echo "[ERROR] ros2 bag record started - aborting" >&2; exit 1; }
ros2 run rdfp init-db --config "$CFG" --drop --yes          || exit 1   # DB
if [[ -d "$BAG_DIR" ]]; then
    ros2 run rdfp rosbag clear --rosbag-dir "$BAG_DIR" --yes || exit 1   # 적재 대기 bag
fi
rm -rf "${IMP_DIR:?}"                                        || exit 1   # 보관 bag
[[ -z "$REC_DIR" ]] || empty_dir "${REC_DIR:?}"              || exit 1   # 레코더 mp4 + sidecar + metadata
empty_dir "${VID_DIR:?}"                                     || exit 1   # bag 이미지 인코딩 mp4 + 실행 로그
empty_dir "${DS_DIR:?}"                                      || exit 1   # 학습 샘플

# ---- 삭제 확인 --------------------------------------------------------------
echo "== after"
show_db after && inspect_dirs after || { echo "[ERROR] something is left - see above" >&2; exit 1; }
echo "deletion verified"

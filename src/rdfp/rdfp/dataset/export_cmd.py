"""`export` 콘솔 스크립트 — 적재된 에피소드를 학습 샘플 트리로 내보낸다 (E7).

설계는 [E7_저장_형식_설계.md](../../../docs/learning/저장_형식.md) 다.

```bash
ros2 run rdfp export --config dataset_config.yaml --out /data/rdfp/datasets/v1
ros2 run rdfp export --config dataset_config.yaml --out /tmp/cmp --label joint_command
```

**ROS 없이도 돈다** — DB 와 numpy 만 쓴다. 다른 dataset CLI 와 같은 `*_cmd.py`
형제 모듈이며 `dataset <sub>` 디스패처를 되살리지 않는다.
"""

from __future__ import annotations

from typing import Optional, Sequence

import argparse
import logging
import sys

from .cli_common import (DEFAULT_CONFIG_FILE_PATH, add_common_args, add_config_arg,
                         configure_logging, load_dataset_or_fail, resolve_config_path)
from .db.connection import open_connection
from .export.writer import (DEFAULT_COMMAND_TOPIC, DEFAULT_JOINT_TOPIC, LABEL_COMMAND,
                            LABEL_PLAN, ExportError, export_dataset)

LOG = logging.getLogger('rdfp.dataset.export')


def _parse_episodes(spec: Optional[str]) -> Optional[list]:
    """`1-3,7` 을 `[1, 2, 3, 7]` 로. `None` 이면 전부다."""
    if not spec:
        return None
    out = []
    for piece in spec.split(','):
        piece = piece.strip()
        if not piece:
            continue
        if '-' in piece.lstrip('-'):
            lo, _, hi = piece.partition('-')
            out.extend(range(int(lo), int(hi) + 1))
        else:
            out.append(int(piece))
    return out


def _all_episodes(conn) -> list:
    """카메라 프레임이 있는 에피소드만 고른다 — 없으면 frame clock 이 안 선다."""
    with conn.cursor() as cur:
        cur.execute('SELECT DISTINCT s.id FROM sessions s '
                    'JOIN image_streams i ON i.episode_id = s.id ORDER BY s.id')
        return [r[0] for r in cur.fetchall()]


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog='export', description='적재된 에피소드를 ACT 학습 샘플 트리로 내보낸다 (E7).')
    add_common_args(p)
    add_config_arg(p, required=False)
    p.add_argument('--out', required=True, help='데이터셋 루트 (없으면 만든다)')
    p.add_argument('--episodes', help='예: 1-3,7. 생략하면 영상이 있는 에피소드 전부')
    p.add_argument('--label', choices=(LABEL_PLAN, LABEL_COMMAND), default=LABEL_PLAN,
                   help='관절-계획(기본) 또는 관절-명령(비교 기준, 단일 백엔드만)')
    p.add_argument('--chunk-len', type=int, default=20,
                   help='chunk 길이 k. **고려 중인 최댓값으로 둔다** — 짧은 k 는 '
                        '앞에서 잘라 쓰면 되고 재export 가 필요 없다 (설계 §5)')
    p.add_argument('--delta-sec', type=float, default=0.1, help='첫 스텝 오프셋 Δ')
    p.add_argument('--step-sec', type=float, default=0.1, help='행동 스텝 T_a')
    p.add_argument('--arm-joints', help='쉼표로 구분한 관절 순서 (생략하면 DB 의 순서)')
    p.add_argument('--camera-topic', default='', help='시계로 쓸 카메라 (여럿이면 필수)')
    p.add_argument('--joint-topic', default=DEFAULT_JOINT_TOPIC)
    p.add_argument('--command-topic', default=DEFAULT_COMMAND_TOPIC)
    p.add_argument('--keep-untrustworthy', action='store_true',
                   help='이벤트가 없어 정상 종료로 **가정한** 에피소드도 통계에 넣는다')
    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    configure_logging(args.log_level)
    config_path = resolve_config_path(args)
    if config_path is None:
        LOG.error('no --config given and default config not found: %s',
                  DEFAULT_CONFIG_FILE_PATH)
        return 2
    return cmd_export(args, config_path)


def cmd_export(args: argparse.Namespace, config_path: str) -> int:
    """`export` 명령: 에피소드를 학습 샘플 트리로 내보낸다."""
    cfg = load_dataset_or_fail(config_path)
    if cfg is None:
        return 2
    try:
        with open_connection(cfg.db) as conn:
            ids = _parse_episodes(args.episodes) or _all_episodes(conn)
            if not ids:
                LOG.error('no episodes with camera frames — nothing to export')
                return 1
            LOG.info('exporting %d episode(s) as %s', len(ids), args.label)
            meta, lines = export_dataset(
                conn, ids, out_dir=args.out, label=args.label,
                delta_sec=args.delta_sec, step_sec=args.step_sec,
                chunk_len=args.chunk_len,
                arm_joints=tuple(args.arm_joints.split(',')) if args.arm_joints else (),
                camera_topic=args.camera_topic, joint_topic=args.joint_topic,
                command_topic=args.command_topic,
                recordings_dir=cfg.recordings_dir,
                skip_untrustworthy=not args.keep_untrustworthy)
            conn.rollback()   # 읽기만 했다
    except ExportError as exc:
        LOG.error('%s', exc)
        return 1
    except Exception:
        logging.exception('export failed')
        return 1

    print(f'{len(meta.episodes)} episode(s), {meta.samples} sample(s) -> {args.out}')
    print(f'  camera {meta.spec.camera_hz:.2f} Hz (measured) '
          f'· chunk_len {meta.spec.chunk_len} · label {meta.spec.label}')
    dropped = [e.episode_id for e in lines if not e.trustworthy]
    if dropped:
        # **조용히 빼지 않는다** — 가정으로 만든 레이블이 섞였는지는 사람이 알아야 한다.
        print(f'  excluded (assumed full duration): {dropped}')
    for note in meta.notes:
        print(f'  note: {note}')
    return 0


__all__ = ['build_parser', 'cmd_export', 'main']


if __name__ == '__main__':
    sys.exit(main())

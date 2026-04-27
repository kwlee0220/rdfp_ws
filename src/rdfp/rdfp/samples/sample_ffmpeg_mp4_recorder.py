#!/usr/bin/env python3
"""FFMpegMp4Recorder 사용 예제.

그라디언트 패턴을 움직이는 150 프레임(5 초 @ 30fps) 테스트 영상을 생성한다.
ROS2 없이 독립적으로 실행할 수 있다.

Usage:
    python3 -m rdfp.samples.sample_ffmpeg_mp4_recorder [OUTPUT_PATH]

    OUTPUT_PATH 를 생략하면 현재 디렉터리의 `sample_recorder_output.mp4` 에
    저장한다.

Requirements:
    - ffmpeg 가 시스템 PATH 에 있어야 한다
    - numpy 패키지
"""

from __future__ import annotations

from typing import Sequence

import argparse
import logging
import os
import sys

import numpy as np

from rdfp.recorder import FFMpegMp4Recorder
from rdfp.types import Resolution


def _make_gradient_frame(width: int, height: int, t: int) -> np.ndarray:
    """시간에 따라 이동하는 그라디언트 BGR 프레임을 생성한다."""
    # 수평 그라디언트 생성
    row = np.linspace(0, 255, width, dtype=np.uint8)
    grad = np.tile(row, (height, 1))
    # 채널별로 다른 위상을 주어 무지개처럼 보이게 함
    b = np.roll(grad, shift=t, axis=1)
    g = np.roll(grad, shift=t * 2, axis=1)
    r = np.roll(grad, shift=t * 3, axis=1)
    return np.stack([b, g, r], axis=-1)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument(
        "output_path",
        nargs="?",
        default="sample_recorder_output.mp4",
        help="output mp4 path",
    )
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--duration", type=float, default=5.0, help="seconds")
    parser.add_argument(
        "--encoder-mode",
        choices=["auto", "cpu", "gpu"],
        default="auto",
        help="encoder selection mode",
    )
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    log = logging.getLogger("sample_recorder")

    if os.path.exists(args.output_path):
        log.error("output path already exists: %s", args.output_path)
        return 1

    n_frames = int(args.fps * args.duration)
    frame_interval_ms = int(1000 / args.fps)

    log.info(
        "recording: path=%s fps=%d size=%dx%d frames=%d",
        args.output_path, args.fps, args.width, args.height, n_frames,
    )

    # 컨텍스트 매니저로 사용하여 예외 발생 시에도 shutdown() 이 호출되도록 함
    with FFMpegMp4Recorder(
        fps=args.fps,
        resolution=Resolution(args.width, args.height),
        pixel_format="bgr8",
        encoder_mode=args.encoder_mode,
        preset="medium",
    ) as rec:
        log.info("selected codec: %s", rec.selected_codec)

        try:
            rec.start(args.output_path)
            for i in range(n_frames):
                frame = _make_gradient_frame(args.width, args.height, i)
                rec.write(frame)
            rec.stop(timeout=30.0)
        except Exception as exc:
            log.exception("recording failed: %s", exc)
            return 2

        log.info(
            "done: frames_written=%d frames_dropped=%d final_state=%s",
            rec.frames_written, rec.frames_dropped, rec.state,
        )

    size_bytes = os.path.getsize(args.output_path)
    log.info("output file size: %d bytes", size_bytes)
    return 0


if __name__ == "__main__":
    sys.exit(main())

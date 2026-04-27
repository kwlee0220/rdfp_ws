#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Sequence

import argparse
import logging
import pathlib
import sys
import time

import cv2

if __package__ in (None, ""):
    # 단독 실행 시 패키지 루트를 import 경로에 추가한다.
    package_root = pathlib.Path(__file__).resolve().parents[2]
    if str(package_root) not in sys.path:
        sys.path.insert(0, str(package_root))
    from rdfp.camera.opencv_camera import OpenCvCamera
else:
    from ..camera.opencv_camera import OpenCvCamera


def build_parser() -> argparse.ArgumentParser:
    """CLI 파서를 생성한다."""
    parser = argparse.ArgumentParser(
        description="OpenCvCamera 동작 확인용 샘플 프로그램"
    )
    parser.add_argument(
        "--camera-id",
        default="4",
        help="카메라 ID(int) 또는 영상 소스 경로/URI(str)",
    )
    parser.add_argument(
        "--resolution",
        default="640x480",
        help="요청 해상도 (예: 640x480)",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="요청 FPS (0보다 큰 실수, 예: 29.97)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=300,
        help="처리할 최대 프레임 수 (0 이하면 무제한)",
    )
    parser.add_argument(
        "--max-consecutive-fails",
        type=int,
        default=30,
        help="연속 read 실패 허용 횟수",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="프레임 창을 표시한다 (q 키로 종료)",
    )
    parser.add_argument(
        "--window-name",
        default="OpenCvCamera Sample",
        help="--show 사용 시 표시할 창 이름",
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="로그 레벨",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """샘플 프로그램 진입점."""
    parser = build_parser()
    args = parser.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level),
        format="%(asctime)s | %(levelname)s | %(name)s | %(message)s",
    )
    logger = logging.getLogger(__name__)

    try:
        camera = OpenCvCamera(args.camera_id, resolution=args.resolution, fps=args.fps)
    except ValueError as exc:
        logger.error("Camera configuration error: %s", exc)
        return 2

    frame_count = 0
    fail_count = 0
    start_time = time.monotonic()

    try:
        opened = camera.open()
        if opened is None:
            logger.error("Failed to open camera")
            return 1

        (actual_width, actual_height), actual_fps = opened
        logger.info(
            "Camera opened: requested=%sx%s@%s, actual=%sx%s@%s, camera_id=%s",
            args.resolution.split("x")[0],
            args.resolution.split("x")[1],
            args.fps,
            actual_width,
            actual_height,
            actual_fps,
            args.camera_id,
        )

        while True:
            frame = camera.read()
            if frame is None:
                fail_count += 1
                logger.warning(
                    "Frame read failed (%s/%s)",
                    fail_count,
                    args.max_consecutive_fails,
                )
                if fail_count >= args.max_consecutive_fails:
                    logger.error("Too many consecutive frame read failures")
                    return 1
                continue

            fail_count = 0
            frame_count += 1

            if args.show:
                cv2.imshow(args.window_name, frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    logger.info("Stop requested by user input (q)")
                    break

            if args.max_frames > 0 and frame_count >= args.max_frames:
                logger.info("Reached max frames: %s", args.max_frames)
                break

        elapsed = time.monotonic() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0.0
        logger.info(
            "Completed: frames=%s, elapsed=%.2fs, avg_fps=%.2f",
            frame_count,
            elapsed,
            avg_fps,
        )
        return 0
    finally:
        camera.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())

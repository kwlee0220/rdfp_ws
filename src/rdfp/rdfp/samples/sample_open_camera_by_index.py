#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

from typing import Sequence

import argparse
import logging
import sys

import cv2


def build_parser() -> argparse.ArgumentParser:
    """CLI 파서를 생성한다."""
    parser = argparse.ArgumentParser(
        description="OpenCV 카메라 식별자 번호로 카메라를 열고 프레임을 표시한다"
    )
    parser.add_argument(
        "camera_index",
        type=int,
        help="cv2.VideoCapture 에 전달할 카메라 식별자 번호 (예: 0, 2, 6, 8)",
    )
    parser.add_argument(
        "--backend",
        default="V4L2",
        choices=["V4L2", "ANY"],
        help="OpenCV 캡처 백엔드 선택",
    )
    parser.add_argument("--width", type=int, default=0, help="요청 프레임 너비 (0=자동)")
    parser.add_argument("--height", type=int, default=0, help="요청 프레임 높이 (0=자동)")
    parser.add_argument("--fps", type=float, default=0.0, help="요청 FPS (0=자동)")
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="처리할 최대 프레임 수 (0 이하면 무제한)",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="창을 띄우지 않고 프레임 카운트만 출력한다",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    """카메라 인덱스로 cv2.VideoCapture 를 열고 프레임을 표시한다."""
    args = build_parser().parse_args(argv)

    logging.basicConfig(level=logging.INFO, format="%(levelname)s | %(message)s")
    logger = logging.getLogger(__name__)

    backend = cv2.CAP_V4L2 if args.backend == "V4L2" else cv2.CAP_ANY
    cap = cv2.VideoCapture(args.camera_index, backend)
    if not cap.isOpened():
        logger.error("Failed to open camera index=%d backend=%s", args.camera_index, args.backend)
        cap.release()
        return 1

    if args.width > 0:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    if args.height > 0:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    if args.fps > 0:
        cap.set(cv2.CAP_PROP_FPS, args.fps)

    actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = cap.get(cv2.CAP_PROP_FPS)
    fourcc_int = int(cap.get(cv2.CAP_PROP_FOURCC))
    fourcc = "".join(chr((fourcc_int >> 8 * k) & 0xFF) for k in range(4))
    logger.info(
        "Camera opened: index=%d size=%dx%d fps=%.2f fourcc=%s",
        args.camera_index, actual_w, actual_h, actual_fps, fourcc,
    )

    window_name = f"camera[{args.camera_index}]"
    frame_count = 0
    try:
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                logger.warning("Frame read failed")
                break
            frame_count += 1

            if not args.no_show:
                cv2.imshow(window_name, frame)
                if (cv2.waitKey(1) & 0xFF) == ord("q"):
                    logger.info("Stop requested by user (q)")
                    break

            if args.max_frames > 0 and frame_count >= args.max_frames:
                logger.info("Reached max frames: %d", args.max_frames)
                break
    finally:
        cap.release()
        if not args.no_show:
            cv2.destroyAllWindows()

    logger.info("Captured frames: %d", frame_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())

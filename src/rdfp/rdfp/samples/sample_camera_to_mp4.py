#!/usr/bin/env python3
"""OpenCvCamera 와 FFMpegMp4Recorder 를 함께 사용하는 샘플.

카메라에서 프레임을 지속적으로 읽어 MP4 파일로 저장한다.
ROS2 없이 독립적으로 실행할 수 있다.

Usage:
    python3 -m rdfp.samples.sample_camera_to_mp4 OUTPUT_PATH [options]

Examples:
    python3 -m rdfp.samples.sample_camera_to_mp4 out.mp4
    python3 -m rdfp.samples.sample_camera_to_mp4 out.mp4 --camera-id 4 --fps 30
    python3 -m rdfp.samples.sample_camera_to_mp4 out.mp4 --camera-id rtsp://...
"""

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
    from rdfp.camera.camera_utils import parse_camera_id
    from rdfp.camera.opencv_camera import OpenCvCamera
    from rdfp.recorder import FFMpegMp4Recorder
else:
    from ..camera.camera_utils import parse_camera_id
    from ..camera.opencv_camera import OpenCvCamera
    from ..recorder import FFMpegMp4Recorder


def build_parser() -> argparse.ArgumentParser:
    """CLI 파서를 생성한다."""
    parser = argparse.ArgumentParser(
        description="OpenCvCamera 로 입력받아 MP4 로 저장하는 샘플 프로그램"
    )
    parser.add_argument(
        "output_path",
        help="생성할 mp4 파일 경로",
    )
    parser.add_argument(
        "--camera-id",
        default="0",
        help="카메라 ID(int) 또는 영상 소스 경로/URI(str)",
    )
    parser.add_argument(
        "--resolution",
        default="640x480",
        help="요청 해상도 (예: 640x480)",
    )
    parser.add_argument(
        "--fps",
        type=int,
        default=30,
        help="녹화 FPS (양의 정수)",
    )
    parser.add_argument(
        "--max-frames",
        type=int,
        default=0,
        help="녹화할 최대 프레임 수 (0 이하면 비활성화)",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="녹화 최대 시간(초). 0 이하면 시간 제한 없음",
    )
    parser.add_argument(
        "--max-consecutive-fails",
        type=int,
        default=30,
        help="연속 read 실패 허용 횟수",
    )
    parser.add_argument(
        "--encoder-mode",
        choices=["auto", "cpu", "gpu"],
        default="auto",
        help="인코더 선택 모드",
    )
    parser.add_argument(
        "--bitrate",
        default="4M",
        help="ffmpeg 비트레이트 (예: 4M)",
    )
    parser.add_argument(
        "--gop-size",
        type=int,
        default=None,
        help="GOP 크기 (생략 시 fps * 2)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="캡처 중 프레임 창을 표시한다 (q 키로 종료)",
    )
    parser.add_argument(
        "--window-name",
        default="Camera To MP4 Sample",
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

    if args.max_frames > 0 and args.duration > 0:
        logger.error("Only one of --max-frames or --duration can be specified")
        return 2

    try:
        camera_id = parse_camera_id(args.camera_id, "camera_id")
        camera = OpenCvCamera(
            camera_id,
            resolution=args.resolution,
            fps=args.fps,
        )
    except ValueError as exc:
        logger.error("Invalid argument: %s", exc)
        return 2

    frame_count = 0
    fail_count = 0
    start_time = time.monotonic()

    try:
        opened = camera.open()
        if opened is None:
            logger.error("Failed to open camera")
            return 1

        actual_resolution, actual_fps = opened
        logger.info(
            "Camera opened: requested_resolution=%s requested_fps=%d actual_resolution=%s actual_fps=%.2f camera_id=%s",
            args.resolution,
            args.fps,
            actual_resolution,
            actual_fps,
            camera_id,
        )

        with FFMpegMp4Recorder(
            fps=args.fps,
            resolution=actual_resolution,
            pixel_format="bgr8",
            encoder_mode=args.encoder_mode,
            bitrate=args.bitrate,
            gop_size=args.gop_size,
        ) as recorder:
            logger.info("Selected codec: %s", recorder.selected_codec)
            recorder.start(args.output_path)

            while True:
                frame = camera.read()
                if frame is None:
                    fail_count += 1
                    logger.warning(
                        "Frame read failed (%d/%d)",
                        fail_count,
                        args.max_consecutive_fails,
                    )
                    if fail_count >= args.max_consecutive_fails:
                        logger.error("Too many consecutive frame read failures")
                        return 1
                    continue

                fail_count = 0
                recorder.write(frame)
                frame_count += 1

                if args.show:
                    cv2.imshow(args.window_name, frame)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        logger.info("Stop requested by user input (q)")
                        break

                if args.max_frames > 0 and frame_count >= args.max_frames:
                    logger.info("Reached max frames: %d", args.max_frames)
                    break

                elapsed = time.monotonic() - start_time
                if args.duration > 0 and elapsed >= args.duration:
                    logger.info("Reached duration limit: %.2fs", args.duration)
                    break

            recorder.stop(timeout=30.0)
            logger.info(
                "Done: frames_written=%d frames_dropped=%d final_state=%s",
                recorder.frames_written,
                recorder.frames_dropped,
                recorder.state,
            )

        elapsed = time.monotonic() - start_time
        avg_fps = frame_count / elapsed if elapsed > 0 else 0.0
        logger.info(
            "Completed: frames=%d elapsed=%.2fs avg_fps=%.2f output=%s",
            frame_count,
            elapsed,
            avg_fps,
            args.output_path,
        )
        return 0
    except Exception as exc:
        logger.exception("Recording failed: %s", exc)
        return 2
    finally:
        camera.release()
        if args.show:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    sys.exit(main())
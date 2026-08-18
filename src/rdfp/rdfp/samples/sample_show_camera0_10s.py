#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import annotations

import sys
import time

import cv2


DURATION_SEC = 10.0
CAMERA_INDEX = 8
WINDOW_NAME = "camera[0]"


def main() -> int:
    """0번 카메라를 열어 10초 동안 프레임을 화면에 표시한다."""
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"ERROR: failed to open camera index={CAMERA_INDEX}", file=sys.stderr)
        cap.release()
        return 1

    start = time.monotonic()
    try:
        while time.monotonic() - start < DURATION_SEC:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("WARN: frame read failed", file=sys.stderr)
                continue
            cv2.imshow(WINDOW_NAME, frame)
            # q 키 입력 시 조기 종료한다.
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    sys.exit(main())

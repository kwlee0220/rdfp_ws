"""영상 한 프레임 읽기 — 에피소드 상세의 미리보기가 쓴다. ROS 를 요구하지 않는다.

**PyAV 로 읽는다** — `cv2.CAP_PROP_POS_FRAMES` 탐색은 매번 디코더를 비워 느리다(CLAUDE.md, 4.8배).
프레임 번호는 `image_frames.frame_index` 와 같은 0-기반 mp4 프레임 번호다. 시각으로 찾지 않는다
— 명목 fps 로 ROS 시각을 계산하면 실제 stamp 와 어긋난다(design_gpt §4.2).
"""

from __future__ import annotations

from typing import Any


class FrameReadError(RuntimeError):
    """요청한 프레임을 읽을 수 없을 때."""


def read_frame(path: str, index: int, *, max_width: int = 960) -> Any:
    """mp4 의 `index` 번째 프레임을 RGB `numpy` 배열(H×W×3, uint8)로.

    앞에서부터 디코드한다 — 에피소드 mp4 는 수백 프레임이라 키프레임 탐색보다 단순하고 정확하다.
    `max_width` 보다 넓으면 줄인다 (미리보기용).
    """
    import av

    if index < 0:
        raise FrameReadError(f'frame index must be >= 0, got {index}')
    with av.open(path) as container:
        for i, frame in enumerate(container.decode(video=0)):
            if i == index:
                if frame.width > max_width:
                    height = max(1, round(frame.height * max_width / frame.width))
                    frame = frame.reformat(width=max_width, height=height)
                return frame.to_ndarray(format='rgb24')
    raise FrameReadError(f'{path}: has fewer than {index + 1} frame(s)')


def frame_count(path: str) -> int:
    """mp4 의 프레임 수 (컨테이너가 적어 둔 값이 없으면 끝까지 센다)."""
    import av

    with av.open(path) as container:
        stream = container.streams.video[0]
        if stream.frames:
            return int(stream.frames)
        return sum(1 for _ in container.decode(video=0))


__all__ = ['FrameReadError', 'frame_count', 'read_frame']

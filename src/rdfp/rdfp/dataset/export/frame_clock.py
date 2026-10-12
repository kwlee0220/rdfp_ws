"""frame clock — 카메라 프레임 시각의 열 (E1).

**카메라 프레임 하나가 학습 샘플 하나다** (수집 설계 D7). 모든 채널을 이 시각열에
맞춰 재표본화한다 — 팔 관측은 보간, 팔 행동은 레이블 함수 평가, 그리퍼는 0차 유지.

기준 시각의 출처는 **적재된 `image_frames` 의 stamp** 이고, 그 값은 레코더가 남긴
sidecar 에서 왔다(I4). 그래서 시각과 영상이 같은 근거를 갖는다 — ``frame_index`` 가
곧 mp4 안의 프레임 번호다.

**고정 fps 격자를 쓰지 않는 이유**는 카메라 간격이 흔들리기 때문이다. 펑션베이는
설정이 10 Hz 인데 실측이 9.54~9.60 Hz 였고 회차마다 달랐다. 격자를 따로 두면 영상과
레이블의 위상이 어긋나는데, 그 어긋남은 에러가 아니라 **학습이 끝난 뒤에** 드러난다.

이 모듈은 ROS 를 모른다 — `load_frame_clock` 만 DB 커넥션을 받으며 그것도 인자로만
쓴다.
"""

from __future__ import annotations

from typing import Any, Optional

from dataclasses import dataclass

NS = 1_000_000_000


class FrameClockError(RuntimeError):
    """frame clock 을 고를 수 없을 때. **조용히 아무거나 고르지 않는다.**"""


@dataclass(frozen=True)
class FrameRef:
    """프레임 하나 — 학습 샘플 하나의 기준점.

    Attributes:
        index: mp4 안의 0-기반 프레임 번호. 영상을 꺼낼 때 그대로 쓴다.
        stamp_ns: 원본 `header.stamp` (나노초). 채널 정렬의 기준이다.
    """

    index: int
    stamp_ns: int

    @property
    def t(self) -> float:
        """초 단위 시각. 레이블 함수(`plan(τ)`)가 받는 값이다."""
        return self.stamp_ns / NS


@dataclass(frozen=True)
class ImageStream:
    """frame clock 이 가리키는 mp4 한 개."""

    topic: str
    mp4_path: str
    codec: str
    pixel_format: str
    fps: int
    width: int
    height: int
    frame_count: int


@dataclass(frozen=True)
class FrameClock:
    """에피소드 하나의 frame clock.

    Attributes:
        episode_id: `sessions.id`.
        stream: 시계를 준 영상.
        frames: 프레임 열. **시각 오름차순**이다.
        episode_start_ns / episode_end_ns: 에피소드 경계. 뒤엣것은 chunk 의
            ``is_pad`` 판정에 쓴다 — 레이블 함수에 그대로 넘긴다.
    """

    episode_id: int
    stream: ImageStream
    frames: tuple
    episode_start_ns: int
    episode_end_ns: int

    def __len__(self) -> int:
        return len(self.frames)

    @property
    def episode_end(self) -> float:
        """초 단위 에피소드 끝. `PlanLabel(episode_end=...)` 에 넘긴다."""
        return self.episode_end_ns / NS

    @property
    def taus(self) -> tuple:
        """프레임 시각(초)만 뽑은 열."""
        return tuple(f.t for f in self.frames)

    def interval_stats(self) -> dict:
        """프레임 간격 통계 — **설정 fps 를 믿지 말고 이것을 본다.**

        펑션베이 실측에서 설정 10 Hz 가 9.54~9.60 Hz 였고 회차마다 달랐다. 간격이
        크게 벌어진 구간은 카메라가 프레임을 흘린 자리이며, 그 사이의 레이블은
        보간으로만 존재한다.

        Returns:
            ``{'count', 'min', 'max', 'mean', 'hz'}``. 프레임이 2 개 미만이면 빈 dict.
        """
        if len(self.frames) < 2:
            return {}
        gaps = [b.t - a.t for a, b in zip(self.frames, self.frames[1:])]
        mean = sum(gaps) / len(gaps)
        return {'count': len(gaps), 'min': min(gaps), 'max': max(gaps),
                'mean': mean, 'hz': (1.0 / mean if mean > 0 else 0.0)}


def available_clock_topics(conn: Any, episode_id: int) -> list:
    """그 에피소드에서 frame clock 이 될 수 있는 토픽 이름."""
    sql = ('SELECT t.topic_name FROM image_streams s JOIN topics t ON t.id = s.topic_id '
           'WHERE s.episode_id = %s ORDER BY t.topic_name')
    with conn.cursor() as cur:
        cur.execute(sql, (episode_id,))
        return [r[0] for r in cur.fetchall()]


def load_frame_clock(conn: Any, episode_id: int, *,
                     topic: Optional[str] = None) -> FrameClock:
    """한 에피소드의 frame clock 을 DB 에서 읽는다.

    Args:
        conn: DB 커넥션.
        episode_id: `sessions.id`.
        topic: 시계로 쓸 카메라 토픽. 카메라가 **하나뿐이면 생략할 수 있다.**

    Raises:
        FrameClockError: 영상이 없거나, 카메라가 여럿인데 ``topic`` 을 안 줬거나,
            준 토픽이 그 에피소드에 없을 때. **아무거나 고르지 않는다** — 카메라마다
            시각이 달라 조용히 다른 시계를 쓰면 영상과 레이블의 위상이 어긋난다.
    """
    topics = available_clock_topics(conn, episode_id)
    if not topics:
        raise FrameClockError(
            f'episode {episode_id} has no registered image stream; '
            f'frame clock needs one (did the recorder output get registered?)')
    if topic is None:
        if len(topics) > 1:
            raise FrameClockError(
                f'episode {episode_id} has {len(topics)} cameras {topics}; '
                f'pass topic= to choose the frame clock')
        topic = topics[0]
    elif topic not in topics:
        raise FrameClockError(
            f'episode {episode_id} has no image stream for {topic!r}; available: {topics}')

    with conn.cursor() as cur:
        cur.execute(
            'SELECT s.mp4_path, s.codec, s.pixel_format, s.container_fps, s.width, '
            '       s.height, s.frame_count, s.topic_id '
            'FROM image_streams s JOIN topics t ON t.id = s.topic_id '
            'WHERE s.episode_id = %s AND t.topic_name = %s', (episode_id, topic))
        mp4, codec, pix, fps, width, height, count, topic_id = cur.fetchone()
        cur.execute(
            'SELECT frame_index, stamp_sec, stamp_nanosec FROM image_frames '
            'WHERE episode_id = %s AND topic_id = %s ORDER BY frame_index',
            (episode_id, topic_id))
        rows = cur.fetchall()
        cur.execute('SELECT start_sec, start_nanosec, stop_sec, stop_nanosec '
                    'FROM sessions WHERE id = %s', (episode_id,))
        s_sec, s_nsec, e_sec, e_nsec = cur.fetchone()

    frames = tuple(FrameRef(index=int(i), stamp_ns=int(sec) * NS + int(nsec))
                   for i, sec, nsec in rows)
    return FrameClock(
        episode_id=int(episode_id),
        stream=ImageStream(topic=topic, mp4_path=str(mp4), codec=str(codec),
                           pixel_format=str(pix), fps=int(fps), width=int(width),
                           height=int(height), frame_count=int(count)),
        frames=frames,
        episode_start_ns=int(s_sec) * NS + int(s_nsec),
        episode_end_ns=int(e_sec) * NS + int(e_nsec))


def arm_action_chunks(clock: FrameClock, label: Any, *, k: int, step: float,
                      delta: float) -> tuple:
    """frame clock 의 프레임마다 팔 행동 chunk 를 만든다 (E1 × E3).

    이것이 ACT 샘플의 ``action.arm`` 부분이다 — 관측(`qpos`)과 그리퍼는 E2 · E4 다.

    Args:
        clock: 기준 시각열.
        label: :class:`~rdfp.dataset.export.motion_label.PlanLabel`.
            **``episode_end=clock.episode_end`` 로 만들어야** ``is_pad`` 가 선다.
        k / step / delta: chunk 모양. 값은 학습 비교로 정한다(ACT 문서 §9 의 1).

    Returns:
        프레임마다 ``(frame_index, t, values, is_pad)`` 인 튜플.
    """
    return tuple((f.index, f.t) + label.chunk(f.t, k=k, step=step, delta=delta)
                 for f in clock.frames)


__all__ = ['FrameClock', 'FrameClockError', 'FrameRef', 'ImageStream',
           'arm_action_chunks', 'available_clock_topics', 'load_frame_clock']

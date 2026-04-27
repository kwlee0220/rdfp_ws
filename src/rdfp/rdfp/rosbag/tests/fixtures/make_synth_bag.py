"""합성 MCAP rosbag 생성기.

Phase 1 단위/통합 테스트에 사용하기 위한 최소 rosbag 아카이브를 구성한다.
운영 방안의 디렉터리 구조 (`<root>/YYYY-MM-DD/session_YYYY-MM-DD_HH-MM-SS/`)
를 따르며 `metadata.yaml` 도 함께 생성한다.
"""

from __future__ import annotations

from typing import Any, Iterable

from dataclasses import dataclass, field
from pathlib import Path

import yaml
from mcap.records import Schema
from mcap_ros2.writer import Writer


# --- ROS 2 IDL 스키마 텍스트 (최소 필드만) -----------------------------------

HEADER_SUBSCHEMA = """
================================================================================
MSG: std_msgs/Header
builtin_interfaces/Time stamp
string frame_id
================================================================================
MSG: builtin_interfaces/Time
int32 sec
uint32 nanosec
"""

SESSION_COMMAND_SCHEMA = """\
std_msgs/Header header
string state
string task_label
""" + HEADER_SUBSCHEMA

POSE_STAMPED_SCHEMA = """\
std_msgs/Header header
geometry_msgs/Pose pose
================================================================================
MSG: geometry_msgs/Pose
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
================================================================================
MSG: geometry_msgs/Point
float64 x
float64 y
float64 z
================================================================================
MSG: geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w
""" + HEADER_SUBSCHEMA

JOINT_STATE_SCHEMA = """\
std_msgs/Header header
string[] name
float64[] position
float64[] velocity
float64[] effort
""" + HEADER_SUBSCHEMA

IMAGE_SCHEMA = """\
std_msgs/Header header
uint32 height
uint32 width
string encoding
uint8 is_bigendian
uint32 step
uint8[] data
""" + HEADER_SUBSCHEMA

COMPRESSED_IMAGE_SCHEMA = """\
std_msgs/Header header
string format
uint8[] data
""" + HEADER_SUBSCHEMA


# --- 헬퍼 ---------------------------------------------------------------------


def _header(sec: int, nanosec: int, frame_id: str = '') -> dict[str, Any]:
    return {
        'stamp': {'sec': sec, 'nanosec': nanosec},
        'frame_id': frame_id,
    }


def _split_ns(ns: int) -> tuple[int, int]:
    return ns // 1_000_000_000, ns % 1_000_000_000


# --- 이벤트 DSL ---------------------------------------------------------------


@dataclass
class SessionEventSpec:
    """합성 `/session` 메시지 명세."""

    stamp_ns: int
    state: str
    task_label: str = ''


@dataclass
class PoseEventSpec:
    """합성 `/ee_pose_publisher/ee_pose` 메시지 명세."""

    stamp_ns: int
    position: tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)


@dataclass
class JointStateEventSpec:
    """합성 `/joint_states` 메시지 명세."""

    stamp_ns: int
    position: list[float] = field(default_factory=list)
    velocity: list[float] = field(default_factory=list)
    effort: list[float] = field(default_factory=list)


@dataclass
class ImageEventSpec:
    """합성 카메라 이미지 (`sensor_msgs/msg/Image`) 메시지 명세."""

    stamp_ns: int
    topic: str
    width: int = 16
    height: int = 12
    encoding: str = 'bgr8'    # bgr8 / rgb8 / mono8
    fill: tuple[int, int, int] | int = (0, 0, 0)
    frame_id: str = ''        # header.frame_id


@dataclass
class CompressedImageEventSpec:
    """합성 카메라 이미지 (`sensor_msgs/msg/CompressedImage`) 메시지 명세."""

    stamp_ns: int
    topic: str
    data: bytes                       # 이미 JPEG 등으로 압축된 바이트
    format: str = 'jpeg'
    frame_id: str = ''                # header.frame_id


# --- 공개 API -----------------------------------------------------------------


def write_synth_bag(
    root: Path,
    *,
    date_dir: str = '2026-04-11',
    session_name: str = 'session_2026-04-11_09-00-03',
    session_events: Iterable[SessionEventSpec] = (),
    pose_events: Iterable[PoseEventSpec] = (),
    joint_state_events: Iterable[JointStateEventSpec] = (),
    image_events: Iterable[ImageEventSpec] = (),
    compressed_image_events: Iterable[CompressedImageEventSpec] = (),
) -> Path:
    """합성 rosbag 세션 디렉터리를 `root` 하위에 생성한다.

    Args:
        root: rosbag 루트 디렉터리 (tmp_path 등).
        date_dir: `YYYY-MM-DD` 형식의 날짜 폴더명.
        session_name: `session_YYYY-MM-DD_HH-MM-SS` 형식의 세션 디렉터리명.
        session_events: `/session` 메시지들 (stamp 오름차순 권장).
        pose_events: `/ee_pose_publisher/ee_pose` 메시지들.
        joint_state_events: `/joint_states` 메시지들.

    Returns:
        생성된 세션 디렉터리 경로.
    """
    session_dir = root / date_dir / session_name
    session_dir.mkdir(parents=True, exist_ok=True)
    bag_basename = session_name
    mcap_relpath = f'{bag_basename}_0.mcap'
    mcap_path = session_dir / mcap_relpath

    all_events: list[tuple[int, str, dict]] = []
    topics_with_counts: dict[str, tuple[str, int]] = {}

    with open(mcap_path, 'wb') as fp, Writer(fp) as w:
        sess_schema: Schema | None = None
        pose_schema: Schema | None = None
        js_schema: Schema | None = None

        # 세션 메시지
        sess_list = list(session_events)
        if sess_list:
            sess_schema = w.register_msgdef(
                'rdfp_msgs/msg/SessionCommand', SESSION_COMMAND_SCHEMA,
            )
            topics_with_counts['/session'] = ('rdfp_msgs/msg/SessionCommand', len(sess_list))

        pose_list = list(pose_events)
        if pose_list:
            pose_schema = w.register_msgdef(
                'geometry_msgs/msg/PoseStamped', POSE_STAMPED_SCHEMA,
            )
            topics_with_counts['/ee_pose_publisher/ee_pose'] = ('geometry_msgs/msg/PoseStamped', len(pose_list))

        js_list = list(joint_state_events)
        if js_list:
            js_schema = w.register_msgdef(
                'sensor_msgs/msg/JointState', JOINT_STATE_SCHEMA,
            )
            topics_with_counts['/joint_states'] = (
                'sensor_msgs/msg/JointState', len(js_list),
            )

        img_list = list(image_events)
        img_schema: Schema | None = None
        if img_list:
            img_schema = w.register_msgdef(
                'sensor_msgs/msg/Image', IMAGE_SCHEMA,
            )
            by_topic: dict[str, int] = {}
            for ev in img_list:
                by_topic[ev.topic] = by_topic.get(ev.topic, 0) + 1
            for topic, n in by_topic.items():
                topics_with_counts[topic] = ('sensor_msgs/msg/Image', n)

        cimg_list = list(compressed_image_events)
        cimg_schema: Schema | None = None
        if cimg_list:
            cimg_schema = w.register_msgdef(
                'sensor_msgs/msg/CompressedImage', COMPRESSED_IMAGE_SCHEMA,
            )
            by_topic = {}
            for ev in cimg_list:
                by_topic[ev.topic] = by_topic.get(ev.topic, 0) + 1
            for topic, n in by_topic.items():
                topics_with_counts[topic] = ('sensor_msgs/msg/CompressedImage', n)

        for ev in sess_list:
            sec, nsec = _split_ns(ev.stamp_ns)
            msg = {
                'header': _header(sec, nsec),
                'state': ev.state,
                'task_label': ev.task_label,
            }
            assert sess_schema is not None
            w.write_message('/session', sess_schema, msg, log_time=ev.stamp_ns)
            all_events.append((ev.stamp_ns, '/session', msg))

        for ev in pose_list:
            sec, nsec = _split_ns(ev.stamp_ns)
            msg = {
                'header': _header(sec, nsec),
                'pose': {
                    'position':    {'x': ev.position[0], 'y': ev.position[1],
                                    'z': ev.position[2]},
                    'orientation': {'x': ev.orientation[0], 'y': ev.orientation[1],
                                    'z': ev.orientation[2], 'w': ev.orientation[3]},
                },
            }
            assert pose_schema is not None
            w.write_message('/ee_pose_publisher/ee_pose', pose_schema, msg, log_time=ev.stamp_ns)
            all_events.append((ev.stamp_ns, '/ee_pose_publisher/ee_pose', msg))

        for ev in js_list:
            sec, nsec = _split_ns(ev.stamp_ns)
            msg = {
                'header': _header(sec, nsec),
                'name': [],
                'position': ev.position,
                'velocity': ev.velocity,
                'effort': ev.effort,
            }
            assert js_schema is not None
            w.write_message('/joint_states', js_schema, msg, log_time=ev.stamp_ns)
            all_events.append((ev.stamp_ns, '/joint_states', msg))

        for ev in img_list:
            sec, nsec = _split_ns(ev.stamp_ns)
            channels, data = _image_payload(
                ev.width, ev.height, ev.encoding, ev.fill,
            )
            msg = {
                'header': _header(sec, nsec, frame_id=ev.frame_id),
                'height': ev.height,
                'width': ev.width,
                'encoding': ev.encoding,
                'is_bigendian': 0,
                'step': ev.width * channels,
                'data': data,
            }
            assert img_schema is not None
            w.write_message(ev.topic, img_schema, msg, log_time=ev.stamp_ns)
            all_events.append((ev.stamp_ns, ev.topic, msg))

        for ev in cimg_list:
            sec, nsec = _split_ns(ev.stamp_ns)
            msg = {
                'header': _header(sec, nsec, frame_id=ev.frame_id),
                'format': ev.format,
                'data': ev.data,
            }
            assert cimg_schema is not None
            w.write_message(ev.topic, cimg_schema, msg, log_time=ev.stamp_ns)
            all_events.append((ev.stamp_ns, ev.topic, msg))

    if not all_events:
        raise ValueError('no events provided; cannot build metadata')

    stamps = [e[0] for e in all_events]
    start_ns = min(stamps)
    end_ns = max(stamps)
    duration_ns = max(end_ns - start_ns, 1)

    metadata = {
        'rosbag2_bagfile_information': {
            'version': 6,
            'storage_identifier': 'mcap',
            'duration': {'nanoseconds': duration_ns},
            'starting_time': {'nanoseconds_since_epoch': start_ns},
            'message_count': len(all_events),
            'topics_with_message_count': [
                {
                    'topic_metadata': {
                        'name': topic,
                        'type': type_name,
                        'serialization_format': 'cdr',
                        'offered_qos_profiles': '',
                    },
                    'message_count': count,
                }
                for topic, (type_name, count) in topics_with_counts.items()
            ],
            'compression_format': '',
            'compression_mode': '',
            'relative_file_paths': [mcap_relpath],
            'files': [
                {
                    'path': mcap_relpath,
                    'starting_time': {'nanoseconds_since_epoch': start_ns},
                    'duration': {'nanoseconds': duration_ns},
                    'message_count': len(all_events),
                },
            ],
        },
    }
    with (session_dir / 'metadata.yaml').open('w', encoding='utf-8') as f:
        yaml.safe_dump(metadata, f, sort_keys=False)

    return session_dir


def _image_payload(
    width: int, height: int, encoding: str, fill,
) -> tuple[int, bytes]:
    """encoding 에 맞는 채널 수와 단색 바이트 페이로드를 생성한다."""
    if encoding in ('bgr8', 'rgb8'):
        channels = 3
        if isinstance(fill, int):
            fill = (fill, fill, fill)
        b, g, r = (fill[2], fill[1], fill[0]) if encoding == 'rgb8' else fill
        pixel = bytes((b, g, r)) if encoding == 'bgr8' else bytes((r, g, b))
    elif encoding == 'mono8':
        channels = 1
        v = fill if isinstance(fill, int) else fill[0]
        pixel = bytes((v,))
    else:
        raise ValueError(f'unsupported test image encoding: {encoding!r}')
    return channels, pixel * (width * height)


__all__ = [
    'SessionEventSpec',
    'PoseEventSpec',
    'JointStateEventSpec',
    'ImageEventSpec',
    'CompressedImageEventSpec',
    'write_synth_bag',
]

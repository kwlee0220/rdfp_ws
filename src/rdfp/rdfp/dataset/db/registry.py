"""메시지 타입 → (테이블, writer, reader) 바인딩.

신규 메시지 타입 지원 시 `MESSAGE_TYPE_REGISTRY` 에 한 줄을 추가하면
적재(writer) 와 복원(reader) 양쪽이 한 곳에서 선언된다. 이미지 메시지
(`sensor_msgs/msg/Image`, `sensor_msgs/msg/CompressedImage`) 는 mp4 sink
로 라우팅되며 본 registry 에서 제외된다.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .readers.base import ReaderBase
from .readers.gripper_command import GripperCommandReader
from .readers.gripper_state import GripperStateReader
from .readers.joint_jog import JointJogReader
from .readers.joint_state import JointStateReader
from .readers.pose_stamped import PoseStampedReader
from .readers.target_joint_states import TargetJointStatesReader
from .readers.twist_stamped import TwistStampedReader
from .writers.base import WriterBase
from .writers.gripper_command import GripperCommandWriter
from .writers.gripper_state import GripperStateWriter
from .writers.joint_jog import JointJogWriter
from .writers.joint_state import JointStateWriter
from .writers.pose_stamped import PoseStampedWriter
from .writers.target_joint_states import TargetJointStatesWriter
from .writers.twist_stamped import TwistStampedWriter


SESSION_COMMAND_TYPE = 'rdfp_msgs/msg/SessionCommand'


@dataclass(frozen=True)
class TypeBinding:
    """한 메시지 타입의 DB 접점을 한 줄로 선언한다.

    Attributes:
        table: 적재 / 조회 대상 테이블 이름.
        writer_cls: `WriterBase` 서브클래스 (INSERT 담당).
        reader_cls: `ReaderBase` 서브클래스 (SELECT → 메시지 복원 담당).
    """

    table: str
    writer_cls: type[WriterBase]
    reader_cls: type[ReaderBase]


# 타입 문자열 → TypeBinding. writer 와 reader 가 한 엔트리로 묶인다.
MESSAGE_TYPE_REGISTRY: dict[str, TypeBinding] = {
    # 관련 토픽: /servo_node/delta_twist_cmds
    #   - 키보드/조이스틱 등을 통해 실시간 제어할때 사용.
    #   - ServoNode의 입력 토픽으로 활용.
    'geometry_msgs/msg/TwistStamped': TypeBinding(
        table='twist_stampeds',
        writer_cls=TwistStampedWriter,
        reader_cls=TwistStampedReader,
    ),
    # 관련 토픽: /servo_node/delta_joint_cmds
    #   - 키보드/조이스틱 등을 통해 조인트 단위로 jog 제어할때 사용.
    #   - ServoNode 의 입력 토픽으로 활용.
    'control_msgs/msg/JointJog': TypeBinding(
        table='joint_jogs',
        writer_cls=JointJogWriter,
        reader_cls=JointJogReader,
    ),
    # 관련 토픽: /target_joint_states
    #   - target_joint_states_publisher 가 servo/controller trajectory 의
    #     마지막 point 를 현재 시각 stamp 와 함께 재발행.
    'rdfp_msgs/msg/TargetJointStates': TypeBinding(
        table='target_joint_states',
        writer_cls=TargetJointStatesWriter,
        reader_cls=TargetJointStatesReader,
    ),
    # 관련 토픽: /joint_states
    #   - joint_state_broadcaster 가 현재 로봇 조이트 정보를 발송.
    'sensor_msgs/msg/JointState': TypeBinding(
        table='joint_states',
        writer_cls=JointStateWriter,
        reader_cls=JointStateReader,
    ),
    # 관련 토픽: /ee_pose
    #   - `/joint_states` + FK 로 계산된 end-effector 포즈.
    'geometry_msgs/msg/PoseStamped': TypeBinding(
        table='pose_stampeds',
        writer_cls=PoseStampedWriter,
        reader_cls=PoseStampedReader,
    ),
    # 관련 토픽: /gripper_control/gripper_cmds
    #   - 그리퍼 제어 명령. (open/close)
    'rdfp_msgs/msg/GripperCommand': TypeBinding(
        table='gripper_cmds',
        writer_cls=GripperCommandWriter,
        reader_cls=GripperCommandReader,
    ),
    # 관련 토픽: /gripper_control/gripper_states
    #   - 그리퍼 상태. (열림/닫힘 정도)
    'rdfp_msgs/msg/GripperState': TypeBinding(
        table='gripper_states',
        writer_cls=GripperStateWriter,
        reader_cls=GripperStateReader,
    ),
}


# mp4 sink 로 라우팅될 이미지 메시지 타입.
# 현재 ``sensor_msgs/msg/Image`` (8-bit raw) 만 지원한다. CompressedImage /
# 16UC1 / mono16 / 32FC1 등은 FrameRouter 에서 ``UnsupportedImageError`` 로
# fail-fast 한다 (필요 시 향후 별도 sink 로 확장).
IMAGE_MESSAGE_TYPES: frozenset[str] = frozenset({
    'sensor_msgs/msg/Image',
})


def is_image_type(type_name: str) -> bool:
    """이미지 메시지 타입 여부를 반환한다."""
    return type_name in IMAGE_MESSAGE_TYPES


def resolve_message_type(topic_type: str) -> Optional[TypeBinding]:
    """`topic_type` 에 매핑된 `TypeBinding` 을 반환하고, 없으면 `None`."""
    return MESSAGE_TYPE_REGISTRY.get(topic_type)


__all__ = [
    'SESSION_COMMAND_TYPE',
    'TypeBinding',
    'MESSAGE_TYPE_REGISTRY',
    'IMAGE_MESSAGE_TYPES',
    'is_image_type',
    'resolve_message_type',
]

"""컨트롤러가 **들고 있는 명령 위치**를 ``sensor_msgs/JointState`` 로 다시 낸다 — servo 의
기준 상태로 쓰기 위해서다.

    /panda_arm_controller/controller_state (control_msgs/JointTrajectoryControllerState)
        ── reference(desired).positions ──┐
    /joint_states (측정)                   ├─> [본 노드] ─> joint_states_commanded
        ── 컨트롤러 밖 관절(손가락) 통과 ──┘                    (servo 의 joint_topic)

왜 있는가 — servo 의 래칫
--------------------------

``moveit_servo`` 에는 목표가 없다. 매 주기 **측정된** 관절값에 IK 증분을 더해 다음 명령을
만든다. 그래서 팔이 명령보다 처져 있으면 그 처짐이 다음 명령의 기준이 되어 **명령 안으로
적분된다.** Isaac 실측(2026-09-07, ``docs/teleop/servo_vs_planned_motion.md`` §3.1):
100 g 블록을 쥔 채 순수 +z 를 넣었더니 ``panda_joint5`` 가 주기(34 ms)마다 7.7 mrad 씩
처지고, 44 주기 뒤 명령은 −0.220 rad — **IK 몫 +0.117 / 되읽기 몫 −0.337** — 손끝은 위로
36 mm 가는 동안 옆으로 30 mm 새었다. 빈손은 되읽기가 1 mrad/주기 뿐이라 IK 가 상쇄한다.

같은 부하에서 ``move_linear``(MoveIt 계획 + JTC)은 1.8 mm 로 들어온다. 목표가 고정돼 있으면
같은 처짐이 컨트롤러의 추종 오차로 남아 **메워지기** 때문이다.

이 노드는 servo 에게 "측정값" 대신 **컨트롤러가 지금 추종 중인 명령 위치**를 준다. 그러면
servo 는 직전 명령에 IK 증분을 쌓고, 처짐은 ``move_linear`` 때와 똑같이 JTC 의 추종 오차로만
남는다. servo 쪽 변경은 ``joint_topic`` 파라미터 하나다.

무엇을 잃는가
-------------

- 팔이 실제로 막히거나 처져도 servo 는 모른다. 그러나 그것은 지금도 마찬가지다 — 래칫
  상태에서도 ``/servo_node/status`` 는 ``NO_WARNING`` 이었다. 오차는 컨트롤러 쪽에 남는다.
- 충돌 검사가 명령 자세 기준이 된다. 정착 오차가 수 mrad(부하 시 20 mrad)라 무시한다.
- 컨트롤러가 없는 스택(펑션베이)에는 ``controller_state`` 가 없어 쓸 수 없다.

손가락 관절
-----------

servo 의 상태 모니터는 **로봇 전체**가 채워져야 기동한다. JTC 상태에는 팔 7 관절만 있으므로
컨트롤러 밖 관절(``panda_finger_joint*``)은 ``/joint_states`` 의 측정값을 그대로 태운다.
측정값이 한 번도 안 왔으면 발행하지 않는다 — 팔만 낸 뒤 손가락이 영영 안 채워지면 servo 가
조용히 기다리기만 하기 때문이다.

``reference`` 와 ``desired``
----------------------------

Humble 의 JTC 상태 메시지에는 둘 다 있고 새 이름이 ``reference`` 다. ``reference`` 가
채워져 있으면 그것을, 비어 있으면 ``desired`` 를 쓴다. 길이가 ``joint_names`` 와 다르면
**버린다** — 잘라 쓰면 관절이 밀린 채 servo 가 그 위에 적분한다.

토픽은 전부 상대 경로다. launch 가 ``controller_state`` 를 컨트롤러 이름으로 remap 한다.
"""

from __future__ import annotations

from typing import Optional

import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from sensor_msgs.msg import JointState

CONTROLLER_STATE_TOPIC = 'controller_state'
MEASURED_TOPIC = 'joint_states'
OUTPUT_TOPIC = 'joint_states_commanded'


def _commanded_point(state: JointTrajectoryControllerState):
    """``reference`` 가 채워져 있으면 그것, 아니면 ``desired``. 둘 다 비면 None."""
    if len(state.reference.positions) == len(state.joint_names) and state.joint_names:
        return state.reference
    if len(state.desired.positions) == len(state.joint_names) and state.joint_names:
        return state.desired
    return None


def merge_commanded(state: JointTrajectoryControllerState,
                    measured: Optional[JointState]) -> Optional[JointState]:
    """컨트롤러 관절은 명령 위치로, 나머지 관절은 측정값 그대로 합친 ``JointState``.

    - 명령 위치가 없거나 길이가 안 맞으면 None (발행하지 않는다).
    - ``measured`` 가 None 이면 None — 손가락이 빠진 상태를 내지 않는다.
    - 속도는 **양쪽 모두** 있을 때만 싣는다. 한쪽만 있으면 배열 길이가 어긋난다.
    """
    point = _commanded_point(state)
    if point is None or measured is None:
        return None

    names = list(state.joint_names)
    positions = list(point.positions)
    have_velocity = len(point.velocities) == len(names)
    velocities = list(point.velocities) if have_velocity else []

    measured_velocity = len(measured.velocity) == len(measured.name)
    for index, name in enumerate(measured.name):
        if name in state.joint_names:
            continue
        names.append(name)
        positions.append(measured.position[index])
        if have_velocity and measured_velocity:
            velocities.append(measured.velocity[index])
        else:
            have_velocity = False

    out = JointState()
    out.header = state.header
    out.name = names
    out.position = positions
    out.velocity = velocities if have_velocity else []
    return out


class CommandedJointStatePublisher(Node):
    """``controller_state`` 가 올 때마다(컨트롤러의 state_publish_rate) 합쳐서 낸다."""

    def __init__(self) -> None:
        super().__init__('commanded_joint_state_publisher')
        self._measured: Optional[JointState] = None
        self._waiting_logged = False
        self._pub = self.create_publisher(JointState, OUTPUT_TOPIC, 10)
        self.create_subscription(JointState, MEASURED_TOPIC, self._on_measured, 10)
        self.create_subscription(JointTrajectoryControllerState, CONTROLLER_STATE_TOPIC,
                                 self._on_controller_state, 10)
        self.get_logger().info(
            f"republishing '{CONTROLLER_STATE_TOPIC}' commanded positions as '{OUTPUT_TOPIC}'")

    def _on_measured(self, msg: JointState) -> None:
        self._measured = msg

    def _on_controller_state(self, msg: JointTrajectoryControllerState) -> None:
        out = merge_commanded(msg, self._measured)
        if out is None:
            if not self._waiting_logged:
                self._waiting_logged = True
                self.get_logger().warning(
                    'not publishing yet: controller reports no commanded positions '
                    'or measured joint_states has not arrived')
            return
        if self._waiting_logged:
            self._waiting_logged = False
            self.get_logger().info('publishing commanded joint state')
        self._pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CommandedJointStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

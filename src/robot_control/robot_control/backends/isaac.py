#!/usr/bin/env python3

"""Isaac Sim 백엔드 — `topic_based_ros2_control/TopicBasedSystem`.

값은 `config/backends/isaac.yaml` 에 있다. 여기에는 **값으로 표현되지 않는 것**만 둔다.
"""

from __future__ import annotations

from robot_control.backends.base import Backend


class IsaacBackend(Backend):
    """servo 의 기준을 명령 위치로 돌려 부하 처짐이 적분되는 것을 막는다."""

    def servo_parameters(self) -> dict:
        """`joint_source: commanded` 를 servo 의 `joint_topic` 으로 옮긴다.

        **값 하나가 파라미터 하나로 안 간다.** 프로파일은 "무엇을 기준으로 삼는가"를
        말하고, servo 는 "어느 토픽을 읽는가"를 받는다. 그 사이 변환이 코드다.

        `commanded` 는 `commanded_joint_state_node` 가 JTC 의 명령 위치를 실어 내는
        토픽을 가리킨다. 그것을 읽어야 servo 가 *측정* 관절값 대신 *명령* 관절값에
        IK 증분을 더한다 — 물체를 든 채 `+z` 를 넣었을 때 옆으로 새던 몫의 절반이
        여기서 사라진다 (나머지 절반은 JTC 의 `open_loop_control`,
        `docs/teleop/servo_vs_planned_motion.md` §3.3).
        """
        params = super().servo_parameters()
        if self.value('servo', 'joint_source') == 'commanded':
            params['joint_topic'] = COMMANDED_JOINT_STATE_TOPIC
        return params


# `commanded_joint_state_node` 가 JTC 의 명령 위치를 내보내는 토픽.
COMMANDED_JOINT_STATE_TOPIC = '/joint_states_commanded'

__all__ = ['COMMANDED_JOINT_STATE_TOPIC', 'IsaacBackend']

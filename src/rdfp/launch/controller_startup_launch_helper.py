from __future__ import annotations

from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown


def _chain_or_shutdown(next_actions, spawner_name: str):
    """성공 시 다음 액션으로 진행하고 실패 시 런치를 종료한다."""

    def _handler(event, _context):
        if event.returncode == 0:
            if isinstance(next_actions, list):
                return next_actions
            return [next_actions]
        return [
            EmitEvent(
                event=Shutdown(
                    reason=(
                        f"{spawner_name} exited with code {event.returncode}; "
                        "aborting launch sequence"
                    )
                )
            )
        ]

    return _handler


def _chain_or_skip(next_actions, spawner_name: str):
    """실패해도 로그를 남기고 다음 액션으로 진행한다."""

    def _handler(event, _context):
        actions = []
        if event.returncode != 0:
            actions.append(
                LogInfo(
                    msg=(
                        f"{spawner_name} exited with code {event.returncode}; "
                        "continuing launch (skip controller)"
                    )
                )
            )
        if isinstance(next_actions, list):
            actions.extend(next_actions)
        else:
            actions.append(next_actions)
        return actions

    return _handler


def create_controller_startup_handlers(
    ros2_control_node,
    joint_state_broadcaster_spawner,
    panda_arm_controller_spawner,
    panda_hand_controller_spawner,
    post_hand_actions,
) -> list[RegisterEventHandler]:
    """컨트롤러 순차 기동을 위한 event handler들을 생성한다."""
    delay_joint_state_broadcaster_after_ros2_control = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller_after_joint_state_broadcaster = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=_chain_or_shutdown(
                panda_arm_controller_spawner,
                "joint_state_broadcaster_spawner",
            ),
        )
    )

    delay_hand_controller_after_arm_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_arm_controller_spawner,
            on_exit=_chain_or_shutdown(
                panda_hand_controller_spawner,
                "panda_arm_controller_spawner",
            ),
        )
    )

    delay_post_nodes_after_hand_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=panda_hand_controller_spawner,
            on_exit=_chain_or_skip(
                post_hand_actions,
                "panda_hand_controller_spawner",
            ),
        )
    )

    return [
        delay_joint_state_broadcaster_after_ros2_control,
        delay_arm_controller_after_joint_state_broadcaster,
        delay_hand_controller_after_arm_controller,
        delay_post_nodes_after_hand_controller,
    ]

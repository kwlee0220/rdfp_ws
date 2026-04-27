"""``replay`` 독립 CLI 단위 테스트 (argparse 흐름).

본 모듈은 ROS 2 런타임이 필요하므로 (rclpy / sensor_msgs 등 전이 의존),
ROS sourcing 이 안 된 환경에서는 전체 테스트를 skip 한다.
"""

from __future__ import annotations

import pytest

replay_cmd_mod = pytest.importorskip(
    'rdfp.dataset.replay_cmd',
    reason='requires ROS 2 runtime (rclpy / sensor_msgs)')


def _parser():
    return replay_cmd_mod._build_parser()


# ---- argparse ----------------------------------------------------------

def test_replay_parser_parses_episode_id_and_defaults() -> None:
    ns = _parser().parse_args(['42', '--config', 'x.yaml'])
    assert ns.episode_id == 42
    assert ns.config == 'x.yaml'
    assert ns.topic == ['/servo_node/delta_twist_cmds']


def test_replay_parser_accepts_single_topic_override() -> None:
    ns = _parser().parse_args(['7', '--topic', '/my/pose'])
    assert ns.topic == ['/my/pose']


def test_replay_parser_accepts_multiple_topics() -> None:
    ns = _parser().parse_args(['7', '--topic', '/a', '/b', '/c'])
    assert ns.topic == ['/a', '/b', '/c']


def test_replay_parser_requires_episode_id() -> None:
    with pytest.raises(SystemExit):
        _parser().parse_args([])

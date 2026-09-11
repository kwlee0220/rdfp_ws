#!/usr/bin/env python3

"""`teleop_keyboard` 의 **deadman** 동작 — 키를 떼면 멈추는가.

터미널은 **키를 뗐다는 이벤트를 주지 않는다.** 그래서 이 노드는 자동반복으로
들어오는 문자가 끊기는 것을 "뗐다"로 읽고, TTL(`deadman_ttl_sec`)이 만료되면
정지한다.

**만료 시 침묵하면 안 된다.** 발행을 멈추기만 하면 servo 가
`incoming_command_timeout`(0.1s)을 다 기다린 뒤에야 정지로 판정하고, 그동안
Butterworth 평활 필터가 마지막 속도를 이어 내보내 **키를 뗀 뒤에도 팔이 흘러간다.**
그래서 0 을 명시적으로 한 번 보낸다 — 매 틱 보내면 명령이 안 끊겨 servo 가 자체
정지 경로로 못 들어간다.

노드 전체는 termios 로 터미널을 raw 모드로 바꾸므로 pytest 안에서 띄울 수 없다.
타이머 콜백만 빌려 대역 위에서 돌린다.
"""

from __future__ import annotations

import pytest

pytest.importorskip('rclpy', reason='requires ROS 2 runtime')

from rdfp.teleop.key_hold import HoldKeyTracker           # noqa: E402
from rdfp.teleop.teleop_keyboard import TeleopKeyboard    # noqa: E402

TICK_SEC = 0.01
TTL_SEC = 0.06


class _Timer:
    timer_period_ns = int(TICK_SEC * 1e9)


class _Logger:
    def warning(self, *_a, **_k):
        pass

    def error(self, *_a, **_k):
        pass

    def info(self, *_a, **_k):
        pass


class _Keys:
    forward, back = 'q', 'a'
    deadman, stop, home = ' ', 'x', '/'


class _Node:
    """`_on_timer` 만 빌려 쓰는 대역. 발행 호출을 기록한다."""

    _on_timer = TeleopKeyboard._on_timer

    _publish_zero = TeleopKeyboard._publish_zero

    def __init__(self) -> None:
        self.timer = _Timer()
        self.keys = _Keys()
        self.motion_keys = {'q', 'a'}
        self.deadman_ttl_sec = TTL_SEC
        self._home_in_progress = False
        self._pending: list[str] = []
        self.published: list[str] = []      # 'move' / 'zero'
        self._motion_is_zero = True
        # 홀드 판정은 노드가 갖지 않는다 — 터미널 대신 `_pending` 을 읽는 트래커를
        # 끼워 `_on_timer` 의 **배선**만 본다. 판정 자체는 test_key_hold.py 가 본다.
        self._holds = HoldKeyTracker(self.motion_keys, ttl_sec=TTL_SEC,
                                     hold_keys=(self.keys.deadman,),
                                     read_keys=self._drain)

    # -- 노드가 쓰는 것들 --------------------------------------------------
    def get_logger(self):
        return _Logger()

    def _drain(self):
        keys, self._pending = self._pending, []
        return keys

    def _handle_oneshot_key(self, key):
        return False

    def _get_all_valid_keys(self):
        return {'q', 'a', ' ', 'x', '/'}

    def _handle_unknown_key(self, key):
        pass

    def _zero_twist(self):
        self._motion_is_zero = True

    def _zero_joint_jog(self):
        pass

    def _apply_key_to_motion(self, key):
        self._motion_is_zero = False

    def _publish_twist_safely(self):
        self.published.append('zero' if self._motion_is_zero else 'move')

    def _publish_joint_jog_safely(self):
        pass

    # -- 시험 편의 --------------------------------------------------------
    def press(self, key):
        self._pending.append(key)

    def tick(self, n=1):
        for _ in range(n):
            self._on_timer()


@pytest.fixture
def node() -> _Node:
    return _Node()


# ---------- 누르는 동안 ----------

def test_holding_keeps_publishing_motion(node):
    """자동반복이 들어오는 동안은 계속 움직인다."""
    for _ in range(5):
        node.press('q')
        node.tick()
    assert node.published == ['move'] * 5


def test_motion_continues_between_autorepeat_chars(node):
    """자동반복 간격(~33ms)은 TTL(60ms)이 메운다 — 홀드 중 끊기면 안 된다."""
    node.press('q')
    node.tick()                      # 문자 도착
    node.tick(3)                     # 30ms 공백
    assert node.published == ['move'] * 4


# ---------- 뗐을 때 ----------

def test_release_publishes_zero_exactly_once(node):
    """**핵심.** TTL 이 만료되면 0 을 보내고, 그 뒤로는 조용해야 한다."""
    node.press('q')
    node.tick()
    node.tick(20)                    # 200ms 무입력 — TTL 만료를 넘긴다

    assert node.published[0] == 'move'
    assert node.published.count('zero') == 1, '0 은 한 번만 보낸다'
    assert node.published[-1] == 'zero', '마지막 발행이 정지여야 한다'


def test_zero_is_not_repeated_every_tick(node):
    """매 틱 0 을 보내면 servo 가 자체 정지 경로로 못 들어간다."""
    node.press('q')
    node.tick()
    node.tick(50)
    assert node.published.count('zero') == 1


def test_stop_happens_within_the_ttl(node):
    """TTL(60ms) + 한 틱 안에 정지가 나가야 한다 — 그래야 servo 의 0.1s 를 건너뛴다."""
    node.press('q')
    node.tick()
    ticks_until_zero = 0
    while 'zero' not in node.published:
        node.tick()
        ticks_until_zero += 1
        assert ticks_until_zero < 20, '정지가 안 나갔다'
    assert ticks_until_zero <= int(TTL_SEC / TICK_SEC) + 1


def test_press_after_release_moves_again(node):
    """한 번 멈춘 뒤 다시 누르면 움직여야 한다 — 표시가 안 풀리는 실수를 잡는다."""
    node.press('q')
    node.tick()
    node.tick(20)                    # 정지
    node.press('q')
    node.tick()
    assert node.published[-1] == 'move'


def test_second_release_publishes_zero_again(node):
    """두 번째로 뗐을 때도 정지가 나가야 한다."""
    for _ in range(2):
        node.press('q')
        node.tick()
        node.tick(20)
    assert node.published.count('zero') == 2


# ---------- Home 이동 중 ----------

def test_nothing_is_published_while_moving_home(node):
    """**'/' 로 ready 이동 중에는 servo 로 아무것도 보내지 않는다.**

    0 이라도 보내면 servo 가 깨어나 같은 컨트롤러 토픽에 궤적을 내고 **move_group
    궤적을 선점한다** — '/' 를 눌러도 ready 로 안 가는 증상이 된다.
    """
    node._home_in_progress = True
    node.press('q')
    node.tick()                      # TTL 갱신
    node.tick(20)                    # TTL 만료까지 — 여기서 0 이 새면 안 된다

    assert node.published == []


def test_publishing_resumes_after_home_finishes(node):
    """이동이 끝나면 다시 정상 동작해야 한다."""
    node._home_in_progress = True
    node.press('q')
    node.tick(5)
    assert node.published == []

    node._home_in_progress = False
    node.press('q')
    node.tick()
    assert node.published[-1] == 'move'


# ---------- '/' (ready 이동) 은 servo 를 멈춰야 한다 ----------

class _HomeNode:
    """`_call_home` 사슬만 빌려 쓰는 대역."""

    _call_home = TeleopKeyboard._call_home
    _start_home_move = TeleopKeyboard._start_home_move
    _resume_servo = TeleopKeyboard._resume_servo
    _on_home_done = TeleopKeyboard._on_home_done

    def __init__(self, stop_ready=True, start_ready=True) -> None:
        self._home_in_progress = False
        self.calls: list[str] = []
        self._zeroed = 0
        node = self

        class _Cli:
            def __init__(self, name, ready):
                self.name, self.ready = name, ready

            def service_is_ready(self):
                return self.ready

            def call_async(self, _req):
                node.calls.append(self.name)

                class _F:
                    def add_done_callback(_s, cb):
                        cb(None)
                return _F()

        class _Servo:
            stop_client = _Cli('stop_servo', stop_ready)
            start_client = _Cli('start_servo', start_ready)

        class _MG:
            def move_to_named_target_async(_s, target):
                node.calls.append(f'move:{target}')

                class _F:
                    def add_done_callback(_x, cb): pass
                return _F()

        self.servo_utils = _Servo()
        self._move_group = _MG()

    def get_logger(self):
        return _Logger()

    def _zero_twist(self):
        self._zeroed += 1

    def _zero_joint_jog(self):
        pass


def test_home_stops_servo_before_moving():
    """**발행을 멈추는 것만으로는 부족하다.**

    명령이 끊기면 servo 가 halt 궤적을 스스로 4회 발행하고, 그것이 같은 컨트롤러
    토픽으로 가서 move_group 궤적을 선점한다. 그래서 servo 를 **정지**시켜야 한다.
    """
    n = _HomeNode()
    n._call_home()
    assert n.calls == ['stop_servo', 'move:ready'], '순서가 뒤바뀌면 선점당한다'


def test_home_resumes_servo_when_done():
    """이동이 끝나면 되살려야 한다 — 안 그러면 이동 키가 영영 안 먹는다."""
    n = _HomeNode()
    n._call_home()

    class _F:
        def exception(self):
            return None
    n._on_home_done(_F())

    assert n.calls[-1] == 'start_servo'
    assert n._home_in_progress is False


def test_home_proceeds_when_stop_servo_is_absent():
    """servo 가 없는 스택에서도 이동은 시도한다 — 그쪽은 선점할 것도 없다."""
    n = _HomeNode(stop_ready=False)
    n._call_home()
    assert n.calls == ['move:ready']


# ---------- 그리퍼 키 ----------

class _GripNode:
    """`_handle_oneshot_key` 의 그리퍼 갈래만 빌려 쓴다."""

    _handle_oneshot_key = TeleopKeyboard._handle_oneshot_key

    def __init__(self) -> None:
        from rdfp.teleop.teleop_keyboard import KeyMapping
        self.keys = KeyMapping()
        self.sent: list[str] = []

    def _call_gripper(self, command):
        self.sent.append(command)

    def _call_home(self):
        self.sent.append('home')

    def get_logger(self):
        return _Logger()

    def __getattr__(self, name):
        # 세션·태스크 갈래는 이 시험의 관심이 아니다 — 호출되면 기록만 한다.
        return lambda *a, **k: None


@pytest.mark.parametrize('key,expected', [('=', 'open'), ('-', 'close'), ('\\', 'grasp')])
def test_gripper_keys_send_their_symbol(key, expected):
    """**명령은 심볼이다** — 숫자는 그리퍼에 종속이라 다른 기구로 옮기면 틀린다."""
    n = _GripNode()
    assert n._handle_oneshot_key(key) is True
    assert n.sent == [expected]


def test_grasp_is_distinct_from_close():
    """`close` 와 `grasp` 는 **성공 판정이 다르다.**

    close 는 목표 자세 도달로, grasp 는 `stalled`(물체에 막혀 더 안 닫힘)로 판정한다.
    물건을 집을 때 close 를 쓰면 `at_goal` 이 영영 서지 않는다.
    """
    from rdfp.teleop.teleop_keyboard import KeyMapping
    k = KeyMapping()
    assert len({k.gripper_open, k.gripper_close, k.gripper_grasp}) == 3


def test_grasp_key_is_recognised_as_valid():
    """유효 키 집합에 없으면 '알 수 없는 키'로 빠져 경고만 남고 무시된다."""
    n = _GripNode()
    n.motion_keys = set()
    n.task_keys = {}
    valid = TeleopKeyboard._get_all_valid_keys(n)
    assert n.keys.gripper_grasp in valid
    assert {n.keys.gripper_open, n.keys.gripper_close} <= valid


def test_every_gripper_key_symbol_is_publishable():
    """**키맵과 허용 심볼 목록이 갈리면 조용히 안 나간다.**

    `_call_gripper` 는 `_GRIPPER_LABELS` 에 없는 심볼을 경고만 남기고 버린다 —
    키는 눌리는데 그리퍼가 반응하지 않아 원인이 안 보인다.
    """
    from rdfp.teleop.teleop_keyboard import _GRIPPER_LABELS
    assert {'open', 'close', 'grasp'} <= set(_GRIPPER_LABELS)


def test_gripper_symbols_match_the_node_targets():
    """`GripperNode` 의 `targets.<goal>` 과 짝이 맞아야 한다 — 모르는 심볼은 거부된다."""
    from rdfp.teleop.teleop_keyboard import _GRIPPER_LABELS
    from robot_control.gripper.gripper_action_node import _DEFAULT_TARGETS
    assert set(_GRIPPER_LABELS) <= set(_DEFAULT_TARGETS)

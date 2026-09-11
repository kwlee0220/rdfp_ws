# HoldKeyTracker Programmer's Guide

**키를 누르고 있는 동안만** 명령이 나가고 떼면 멈추게 하는 장치다. 본 가이드는
[key_hold.py](../../src/rdfp/rdfp/teleop/key_hold.py) 의 `HoldKeyTracker` 와 그 동반
클래스들을 **언제, 어떻게 쓰고, 어떤 한계가 있는지** 다룬다.

> **이 모듈은 ROS 를 모른다.** 무엇을 발행할지는 호출자가 정한다. `rclpy` 없이
> import 되고 테스트도 ROS 없이 돈다.

## 1. 왜 있는가 — 터미널은 '뗐다'를 알려 주지 않는다

터미널은 문자 스트림만 준다. `q` 를 누르고 있으면 자동반복(typematic)으로 `q q q q…`
가 오다가, 떼면 그냥 **안 온다**.

```
누름:  q · · q q q q q q q q q ...
뗌:    (아무 이벤트도 없다)
```

'뗐다'와 '잠깐 쉰다'가 구분되지 않는다. 그래서 **문자가 끊기는 것을 뗀 것으로 읽고**
**TTL**(Time To Live, 마지막 입력이 유효한 잔여 시간)이 만료되면 멈춘다. 저장소 다른
곳에서 "deadman TTL" 이라 부르는 것이 이것이다.

세 조각이 함께여야 동작한다. **하나라도 빠지면 에러 없이 팔이 흘러간다.**

| 조각 | 담당 | 빠뜨리면 |
|---|---|---|
| 매 틱 입력 버퍼를 **완전히** 비운다 | `TerminalKeyReader.__call__` | 자동반복 잔여 문자가 뗀 뒤에도 TTL 을 갱신해 팔이 계속 움직인다 |
| TTL 로 뗌을 추정한다 | `HoldKeyTracker.tick` | 홀드 자체가 없다 |
| 만료 시 0 을 **한 번** 보낸다 | `HoldState.just_released` | 침묵하면 servo 가 `incoming_command_timeout`(0.1 s)을 다 기다리고 그동안 평활 필터가 마지막 속도를 이어 내보낸다. **매 틱** 보내면 명령이 안 끊겨 servo 가 자체 정지 경로로 못 들어간다 |

## 2. 구조

```
   stdin ──▶ TerminalKeyReader ──┐
                                 │  read_keys() -> ['q', 'q', '=']
   (또는 evdev / 테스트 대역) ────┘
                                 ▼
                        ┌──────────────────────┐
                        │   HoldKeyTracker     │   ← TTL·마지막 키·0 발행 표시
                        │   .tick(dt)          │
                        └──────────┬───────────┘
                                   ▼  HoldState
                    key / active / just_released / other_keys
                                   ▼
                          호출자가 발행을 정한다
```

**입력원은 주입한다.** 기본은 터미널이지만 같은 모양(`() -> Sequence[str]`)이면
무엇이든 된다 — 테스트 대역, 그리고 언젠가 press/release 를 직접 주는 evdev.

## 3. 빠른 시작

### 3.1 최소 예제 (ROS 없이)

```python
from rdfp.teleop.key_hold import HoldKeyTracker, TerminalRawMode

TICK = 0.01

tracker = HoldKeyTracker(('q', 'a', 'j', 'l'), ttl_sec=0.06, hold_keys=(' ',))

with TerminalRawMode():                 # 터미널을 cbreak 로 — 끝나면 자동 복원
    while True:
        state = tracker.tick(TICK)

        for key in state.other_keys:    # 모션 키가 아닌 것은 그대로 넘어온다
            print(f'one-shot: {key!r}')

        if state.just_released:
            print('STOP')               # 정확히 한 번 나간다
        elif state.active:
            print(f'move {state.key}')  # key 가 None 이면 홀드 키만 눌린 상태

        time.sleep(TICK)
```

`tick()` 은 **틱마다 정확히 한 번** 부른다. 두 번 부르면 TTL 이 두 배로 빨리 닳는다.

### 3.2 rclpy 타이머에 붙이기 (`teleop_keyboard` 의 실제 배선)

```python
class TeleopKeyboard(Node):
    def __init__(self):
        ...
        self._holds = HoldKeyTracker(self.motion_keys, ttl_sec=self.deadman_ttl_sec,
                                     hold_keys=(self.keys.deadman,))
        self.timer = self.create_timer(1.0 / self.rate_hz, self._on_timer)

    def _on_timer(self):
        dt = self.timer.timer_period_ns * 1e-9
        state = self._holds.tick(dt)

        for key in state.other_keys:
            if self._handle_oneshot_key(key):        # 그리퍼·에피소드·태스크
                continue
            if key not in self._get_all_valid_keys():
                self._handle_unknown_key(key)
                continue
            if key == self.keys.stop:                # 3.4 참조
                self._holds.cancel()
                self._publish_zero()
                return

        if state.just_released:
            self._publish_zero()
            return
        if not state.active:
            return

        if state.key is None:
            self._zero_twist()                       # 홀드 키(SPACE)만 눌렸다
        else:
            self._apply_key_to_motion(state.key)
        self._publish_twist_safely()
```

전체는 [teleop_keyboard.py](../../src/rdfp/rdfp/teleop/teleop_keyboard.py) 의
`_on_timer` 를 본다.

### 3.3 입력원 바꿔 끼우기 — 테스트

터미널 없이 시험할 수 있다. 이것이 이 클래스를 노드에서 분리한 이유다.

```python
class FakeKeys:
    def __init__(self):
        self.pending = []

    def feed(self, *keys):
        self.pending.extend(keys)

    def __call__(self):
        keys, self.pending = self.pending, []
        return keys


keys = FakeKeys()
tracker = HoldKeyTracker(('q', 'a'), ttl_sec=0.06, read_keys=keys)

keys.feed('q')
assert tracker.tick(0.01).key == 'q'          # 누르는 동안

for _ in range(20):                            # 200 ms 무입력
    state = tracker.tick(0.01)
assert state.active is False                   # 멈췄다
```

실제 시험 묶음: [test_key_hold.py](../../src/rdfp/rdfp/teleop/tests/test_key_hold.py).

### 3.4 정지 키 — `cancel()` 을 함께 부른다

정지 키(`x`)는 **호출자가 직접 0 을 보낸다.** 그때 `cancel()` 을 부르지 않으면
이어서 `just_released` 가 떠 **0 이 두 번** 나가고, 두 번째가 servo 의 정지 판정을
다시 깨뜨려 오히려 늦어진다.

```python
if key == self.keys.stop:
    self._holds.cancel()      # 홀드를 끊고 just_released 도 억제한다
    self._publish_zero()      # 0 은 여기서 한 번
    return
```

### 3.5 다른 조작기에 붙이기

모션 키 집합과 TTL 만 바꾸면 된다 — 이 클래스는 무엇을 발행할지 모른다.

```python
# 그리퍼 조그: 누르는 동안만 열림/닫힘 폭을 밀어 준다
tracker = HoldKeyTracker(('o', 'p'), ttl_sec=0.08)

# 관절 조그: 7 축을 두 방향씩
JOINT_KEYS = tuple('1234567') + tuple('!@#$%^&')
tracker = HoldKeyTracker(JOINT_KEYS, ttl_sec=0.06)
```

## 4. API 레퍼런스

### `HoldKeyTracker(motion_keys, *, ttl_sec, hold_keys=(), read_keys=None)`

| 인자 | 뜻 |
|---|---|
| `motion_keys` | 눌린 동안 움직이는 키들 |
| `ttl_sec` | 마지막 입력 뒤 이만큼 지나면 뗀 것으로 본다. 자동반복 설정과 함께 정해야 하는 값이다 — 부등식과 실측은 **§6**. `teleop_keyboard` 기본 0.06 s |
| `hold_keys` | TTL 만 갱신하고 움직이지는 않는 키 (SPACE 같은 데드맨 키) |
| `read_keys` | 이번 틱의 입력을 돌려주는 호출 가능 객체. 생략하면 `TerminalKeyReader()` |

`ttl_sec <= 0` 이면 `ValueError` 다 — 홀드가 성립하지 않는데 조용히 안 움직이는 것보다
낫다.

#### `tick(dt) -> HoldState`

`dt` 초가 지났다고 보고 한 틱 진행한다. 하는 일:

1. TTL 을 `dt` 만큼 깎는다
2. `read_keys()` 로 이번 틱의 입력을 **전부** 가져온다
3. 모션·홀드 키를 만나면 TTL 을 되채우고 **배치의 마지막 것**을 현재 키로 삼는다
4. 나머지는 `other_keys` 로 넘긴다
5. TTL 이 만료됐고 아직 0 을 안 보냈으면 `just_released=True`

#### `cancel() -> None`

홀드를 즉시 끊는다. `just_released` 가 뒤이어 뜨지 않는다 (§3.4).

#### 조회 속성

`key` (현재 모션 키 또는 `None`) · `active` (홀드가 살아 있는가) ·
`remaining_sec` (남은 TTL).

### `HoldState`

frozen dataclass — 호출자가 바꿔 다음 틱에 영향을 줄 수 없다.

| 필드 | 뜻 |
|---|---|
| `key` | 지금 유효한 **모션 키**. 홀드 키만 눌렸거나 홀드가 끊기면 `None` |
| `active` | 홀드가 살아 있는가. `True` 면 명령을 발행한다 |
| `just_released` | **이번 틱에** 만료됐는가. 여기서 0 을 한 번 발행한다 |
| `other_keys` | 모션·홀드 키가 아닌 입력 (입력 순서 유지). 처리는 호출자 몫 |

`active` 와 `just_released` 는 **동시에 참이 되지 않는다.** 만료된 틱은
`active=False, just_released=True` 이고, 그 다음 틱부터는 둘 다 거짓이다.

### `TerminalKeyReader`

`read_one() -> Optional[str]` — 키 하나. escape sequence(`\x1b[A` 같은 화살표)는
합쳐서 돌려준다. 합치지 않으면 `[` 가 별개 키로, `A` 가 알 수 없는 키로 오인식된다.

`__call__() -> list` — 이번 틱에 쌓인 키 **전부**. 반드시 다 비운다 (§1).

### `TerminalRawMode`

cbreak 모드 컨텍스트 매니저. 예외로 벗어나도 설정을 복원한다. 복원 실패는 조용히
넘긴다 — 이미 터미널이 없어진 상황이라 여기서 예외를 올리면 원인만 가린다.

`teleop_keyboard` 와 `session_teleop` 이 함께 쓴다.

## 5. 함정

- **`tick()` 은 틱마다 한 번.** 두 번 부르면 TTL 이 두 배로 닳아 홀드가 일찍 끊긴다.
- **`other_keys` 는 TTL 을 갱신하지 않는다.** one-shot 키를 연타해도 팔은 멈춘다 —
  의도된 동작이다.
- **홀드 키가 모션 키보다 뒤에 오면 그것이 이긴다.** 배치의 마지막 것만 유효하므로
  `q` 뒤에 SPACE 가 오면 `key` 는 `None` 이다.
- **`ttl_sec` 은 자동반복 설정과 짝이다.** 초기지연·반복간격보다 짧으면 홀드 중에
  끊기고, 너무 길면 키를 뗀 뒤에도 그만큼 더 움직인다 (§6).
- **`TerminalRawMode` 밖에서 `TerminalKeyReader` 를 쓰면 안 된다.** 줄 단위 버퍼링
  때문에 Enter 를 누를 때까지 아무것도 안 온다.

## 6. `ttl_sec` 과 자동반복 설정의 관계

TTL 로 뗌을 추정하는 이상, **`ttl_sec` 은 터미널의 자동반복 설정과 함께 정해야 한다.**
`xset r rate <초기지연ms> <반복Hz>` 의 **두 인자가 각각 하나씩** 조건을 만든다.

```
    초기지연 / 1000  <  ttl_sec     ①  홀드 시작 직후 한 번 끊기지 않는다
    1 / 반복Hz       <  ttl_sec     ②  반복 사이에 끊기지 않는다
                        ttl_sec ↓   ③  떼면 빨리 멈춘다
```

①·②는 `ttl_sec` 의 **하한**이고 ③은 사람이 느끼는 **정지 지연**이다. 두 요구가 반대라
`ttl_sec` 은 "하한을 여유 있게 넘되 최대한 작게" 잡는 값이 된다.

### 6.1 실측 — 1 초 홀드 중 몇 번 끊기나

자동반복 문자열을 만들어 실제 `HoldKeyTracker` 에 먹인 결과다 (노드 틱 100 Hz).

| 초기지연 | 반복 | 간격 | `ttl_sec` | 끊김 | 깨진 조건 |
|---:|---:|---:|---:|---:|---|
| **660 ms** | 25 Hz | 40 ms | 60 ms | **1회** | ① (660 > 60) — **X 기본값**  |
| 660 ms | 25 Hz | 40 ms | 700 ms | 0 | 통과하지만 ③이 무너진다 |
| **30 ms** | **30 Hz** | 33 ms | **60 ms** | **0** | 둘 다 통과 — **권장** |
| 30 ms | 30 Hz | 33 ms | 30 ms | 10회 | ② (33 > 30) |
| 30 ms | 30 Hz | 33 ms | 40 ms | 0 | 통과, 여유 1.2배로 아슬아슬 |
| 100 ms | 10 Hz | 100 ms | 60 ms | 10회 | ② (100 > 60) |

**끊기는 양상이 다르다.** ①이 깨지면 **시작 직후 딱 한 번**, ②가 깨지면 **반복마다
계속**이다. "키를 누르면 잠깐 갔다가 멈췄다 다시 간다"는 ①이다.

### 6.2 어느 쪽을 고치나 — 자동반복이다

②를 만족시키려고 `ttl_sec` 을 초기지연 위로 올리면(표 2행) ①·②는 통과하지만 **키를
떼고도 0.7 초를 더 움직인다.** ③이 무너져 데드맨이 데드맨이 아니게 된다. 그래서
**`ttl_sec` 은 작게 두고 자동반복을 빠르게** 한다.

```bash
xset q | grep -A1 'auto repeat delay'   # 지금 값
xset r rate 30 30                       # 지연 30 ms, 초당 30 회
```

`teleop_keyboard` 는 [run_teleop_keyboard.sh](../../scripts/run_teleop_keyboard.sh) 가
이것을 걸고 끝나면 되돌린다 (`TELEOP_KB_DELAY` / `TELEOP_KB_RATE` 로 조절, `RATE=0` 이면
건드리지 않는다). **X 서버 전역 설정**이라 실행 중에는 다른 창의 키 반복도 함께
빨라진다.

- **Wayland 에서는 `xset` 이 안 먹는다.** `gsettings set
  org.gnome.desktop.peripherals.keyboard delay 30` / `repeat-interval 33` 을 쓴다.
- **SSH 셸에서는 못 바꾼다.** `DISPLAY` 가 없어 스크립트도 이 단계를 건너뛴다.

### 6.3 여유는 2배 안팎

`ttl_sec` 은 노드 타이머(`rate_hz`, 기본 100 Hz → `dt` 10 ms)마다 깎이므로 **틱 한 칸의
양자화 오차**가 있고, 문자 도착에도 스케줄링 지터가 있다. 표의 40 ms 행은 통과하지만
간격 33 ms 대비 1.2배뿐이라 지터에 취약하다. 기본 조합의 여유는 ① 2.0배 / ② 1.8배다.

`deadman_ttl_sec` 을 바꾸면 위 두 부등식을 다시 확인한다.

### 6.4 근본 해결은 evdev

**근본 해결은 evdev** 다 — `/dev/input/event*` 를 직접 읽어 press/release 를 모두
받으므로 추정이 필요 없다. `clutch_pedal` 이 그 선례이며
([clutch_pedal_guide.md](clutch_pedal_guide.md)), 대가는 입력 장치 권한과 **SSH
세션에서 못 쓴다**는 것이다. 옮긴다면 `read_keys` 를 evdev 판으로 바꿔 끼우고
`ttl_sec` 를 아주 크게 두거나 `cancel()` 을 release 이벤트에 걸면 된다 — 이 모듈의
나머지는 그대로다.

## 7. 관련

- [README.md](README.md) — teleop 문서 진입점, 조용히 실패하는 함정 표
- [clutch_pedal_guide.md](clutch_pedal_guide.md) — evdev 로 press/release 를 받는 선례
- [servo_vs_planned_motion.md](servo_vs_planned_motion.md) — servo 로 물체를 옮길 때의 한계
- [../moveit/servo_client_programmers_guide.md](../moveit/servo_client_programmers_guide.md) — `/servo_node` 시작·정지·상태

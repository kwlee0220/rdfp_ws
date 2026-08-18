# 조이스틱(게임패드) ROS 2 연결 가이드

> 📖 teleop 문서 진입점: [README.md](README.md) — 하려는 일부터 문서를 고른다.

새 PC 에서 게임패드(Sony DualShock 호환 / 일반 HID 패드)를 ROS 2 의 `/joy`
토픽으로 publish 하기까지의 절차를 정리한다. 본 문서는 다음 환경에서 검증됨.

- Ubuntu 22.04 + ROS 2 Humble
- Bluetooth 페어링된 HID 게임패드 (예: USB ID `1949:0402` Lab126 클론,
  bluetoothctl 상에서 `Gamepad` 로 표시)
- ROS 2 표준 `joy` 패키지(SDL2 기반) 가 enumerate 하지 못하는 케이스를 포함

---

## 0. 목표

```
[게임패드] → /dev/input/jsN → joy_linux_node → /joy (sensor_msgs/Joy)
```

`/joy` 토픽이 흐르고, 패드 입력에 따라 `axes` / `buttons` 가 바뀌면 완료.

---

## 1. ROS 패키지 설치

```bash
sudo apt update
sudo apt install -y ros-humble-joy ros-humble-joy-linux ros-humble-teleop-twist-joy
```

| 패키지 | 역할 | 비고 |
|---|---|---|
| `joy` | SDL2 기반 표준 joy 노드 | 일부 BT HID 패드를 enumerate 못 함 |
| **`joy_linux`** | `/dev/input/jsN` 직접 읽는 노드 | **권장** — 호환성 가장 좋음 |
| `teleop_twist_joy` | `Joy → Twist` 어댑터 | 본 워크스페이스는 자체 teleop 사용 권장 (servo 가 `TwistStamped` 요구) |

확인:

```bash
ros2 pkg list | grep -E "^(joy|joy_linux|teleop_twist_joy)$"
ros2 pkg executables joy_linux
# 출력에 'joy_linux joy_linux_node' 가 보여야 함
```

---

## 2. 사용자 그룹 (`input`)

`/dev/input/js*` 는 보통 `root:input` 소유. 사용자가 `input` 그룹에 있어야 read 가능.

```bash
# 현재 셸이 input 그룹에 들어 있는지
id -nG | tr ' ' '\n' | grep -x input || echo "(NOT in input)"

# 들어있지 않으면 추가
sudo usermod -aG input "$USER"
```

> **중요**: 그룹 추가는 **로그인 세션이 새로 만들어져야** 적용됨.
> 데스크톱 로그아웃 후 재로그인 (또는 재부팅) 필요.
> 단발 검증은 `sg input -c "<명령>"` 또는 `newgrp input` 후 같은 셸에서.

---

## 3. 패드 연결

### 3-A. USB 연결 (가장 단순)

```bash
# 케이블 연결 후
lsusb | grep -iE "sony|054c|1949|playstation|gamepad"
ls /dev/input/js*
```

`/dev/input/js0` 같은 노드가 만들어지면 OK.

### 3-B. Bluetooth 연결

```bash
# bluetoothctl 진입
bluetoothctl
> power on
> agent on
> scan on
# 패드 페어링 모드 진입 후 보이는 MAC 확인
> pair  <MAC>
> trust <MAC>
> connect <MAC>
> exit
```

연결 상태 확인:

```bash
bluetoothctl info <MAC> | grep -E "Connected|Paired|Trusted"
ls /dev/input/js*
```

`Connected: yes` + `/dev/input/jsN` 노드 생성 확인.

> DualShock 3 정품의 BT 페어링은 표준 절차로 안 되는 경우가 많음 — 이 경우
> `sixad`/`sixpair` 도구로 호스트 BT MAC 을 컨트롤러에 write 하는 별도 절차 필요.
> Lab126 등 클론은 일반 BT HID 처럼 동작.

---

## 4. raw 입력 검증 (드라이버 단)

`joy_node` 실행 전에 OS 단에서 디바이스가 이벤트를 내는지 직접 확인. 이게 안 되면
ROS 단도 동작 불가.

```bash
python3 -c "
import os, struct
fd = os.open('/dev/input/js0', os.O_RDONLY)
print('opened, press buttons / move sticks (Ctrl+C to stop)')
while True:
    t, v, ty, n = struct.unpack('IhBB', os.read(fd, 8))
    if ty & 0x80:           # init 이벤트는 무시
        continue
    print('btn' if ty == 1 else 'axis', n, '=', v)
"
```

스틱/버튼 조작 시 줄 단위로 `btn N = V` / `axis N = V` 가 흘러야 정상.

이 단계가 비어있으면:
- 패드 절전 — PS 버튼 길게 눌러 깨우기
- BT 끊김 — `bluetoothctl info <MAC>` 로 Connected 확인
- 권한 — 위 2번 (input 그룹)

---

## 5. SDL2 enumerate 가능한지 체크 (선택, 진단용)

표준 `joy` 패키지는 SDL2 기반이라 패드가 SDL2 에 보여야 동작함.

```bash
python3 -c "
import ctypes
sdl = ctypes.CDLL('libSDL2-2.0.so.0')
sdl.SDL_Init(0x200)
n = sdl.SDL_NumJoysticks()
print(f'SDL2 sees {n} joystick(s)')
sdl.SDL_JoystickNameForIndex.restype = ctypes.c_char_p
for i in range(n):
    name = sdl.SDL_JoystickNameForIndex(i)
    print(f'  [{i}]', name.decode() if name else '?')
sdl.SDL_Quit()
"
```

- `sees 0` → SDL2 미인식. **`joy_linux` 사용 (6번)**
- `sees 1` → `joy` 패키지로도 가능. 단 `joy_linux` 가 더 안정적이라 본 가이드는
  `joy_linux` 로 통일.

> SDL2 가 보지 못하는 가장 흔한 원인은 udev hwdb 가 해당 디바이스에
> `ID_INPUT_JOYSTICK=1` 태그를 안 붙여서다. udev rule 로 우회 가능하지만
> `joy_linux` 가 훨씬 간단함.

---

## 6. `joy_linux_node` 실행

```bash
source /opt/ros/humble/setup.bash

ros2 run joy_linux joy_linux_node --ros-args \
    -p dev:=/dev/input/js0 \
    -p deadzone:=0.1 \
    -p autorepeat_rate:=20.0
```

| 파라미터 | 값 | 의미 |
|---|---|---|
| `dev` | `/dev/input/js0` | js 디바이스 경로 (디바이스 번호가 다르면 변경) |
| `deadzone` | `0.1` | 스틱 중립 무시 영역 (0.0–1.0) |
| `autorepeat_rate` | `20.0` | 입력 변화가 없어도 20Hz 로 마지막 상태 publish |

기동 시 콘솔에 다음과 비슷한 줄이 떠야 정상:

```
[INFO] [joy_node]: Opened joystick: /dev/input/js0. deadzone_: 0.100000.
```

`Couldn't open joystick force feedback: Bad file descriptor` 는 **무해한 WARN**
(rumble 미지원 패드에서 흔함, 무시).

---

## 7. `/joy` 토픽 검증

별개 터미널에서:

```bash
source /opt/ros/humble/setup.bash

# (1) 토픽 + publisher 등록 확인
ros2 topic info /joy --verbose

# (2) 발행 주기 측정 — autorepeat 적용 시 패드 가만히 둬도 ~20Hz
ros2 topic hz /joy

# (3) 실제 메시지 한 번 확인
ros2 topic echo /joy --once
```

기대치:
- `topic info` → `Publisher count: 1`, `Node name: joy_node`
- `topic hz` → `average rate: ~20.x` (autorepeat 영향). 패드 만지면 살짝 더 높음
- `topic echo` → `axes` (float[]) 와 `buttons` (int[]) 배열 출력

---

## 8. 버튼/스틱 매핑 측정

워크스페이스 통합 시 어느 인덱스가 어느 동작인지 알아야 한다. `echo` 흐르는 동안
한 가지씩 조작하며 메모.

```bash
ros2 topic echo /joy --field axes
# 좌 스틱을 위로만 살짝 → 변화하는 인덱스 = 좌 스틱 Y

ros2 topic echo /joy --field buttons
# X 버튼만 → 1 로 바뀌는 인덱스 = X
```

권장 메모 항목 (패드마다 다름):

| 동작 | axes/buttons 인덱스 |
|---|---|
| 좌 스틱 X / Y | axes[?], axes[?] |
| 우 스틱 X / Y | axes[?], axes[?] |
| L2 / R2 트리거 | axes[?] / axes[?] (또는 buttons[?]) |
| D-pad 상하/좌우 | axes[?] / axes[?] (또는 buttons) |
| ○ × △ □ / A B X Y | buttons[?] |
| L1 / R1 | buttons[?] |
| Start / Select | buttons[?] |
| L3 / R3 (스틱 클릭) | buttons[?] |
| PS / Home | buttons[?] |

매핑은 패드 모델뿐 아니라 커널 버전·드라이버에 따라서도 바뀔 수 있으므로,
**새 PC 에서는 새로 측정** 한다.

---

## 9. 트러블슈팅

| 증상 | 원인 / 해결 |
|---|---|
| `lsusb` 에 패드 안 보임 | 데이터 케이블 아님(전원 전용) / 포트 / 드라이버 |
| `/dev/input/js0` 없음 | `joydev` 모듈 미로드. `sudo modprobe joydev` |
| Python raw read 도 비어있음 | 패드 절전 (PS 버튼 깨우기) / BT 끊김 / 권한 |
| `ros2 run joy joy_node` 가 publish 안 함 | SDL2 미인식. **`joy_linux` 사용** |
| `joy_linux_node` 실행 시 `No executable found` | 패키지 미설치. `sudo apt install ros-humble-joy-linux` |
| `Couldn't open joystick force feedback` WARN | 무해. 패드가 rumble 미지원 |
| `topic hz` 가 `no messages received` | joy_node 가 실제로 디바이스 read 못 함. `dev` 경로 확인, 권한 확인 |
| `Permission denied` on `/dev/input/js0` | `input` 그룹 미소속 또는 셸에 미적용 (재로그인) |

---

## 10. 다음 단계 (rdfp 워크스페이스 통합)

`/joy` 토픽이 정상 흐르면, 이걸 본 워크스페이스의 servo 토픽으로 변환하는 노드를
직접 작성한다 (`rdfp/teleop/joy_teleop.py` 추가 권장).

핵심 매핑:

| Joy 입력 | rdfp 동작 | 발행 토픽 (메시지) |
|---|---|---|
| 좌/우 스틱 + 트리거 | EE Cartesian Twist | `/servo_node/delta_twist_cmds` (`geometry_msgs/TwistStamped`) |
| L1 / R1 | gripper open / close | gripper service 또는 토픽 |
| Start | session 시작/종료 | `session_control` 명령 |
| Select | episode 시작/종료 | `session_control` 명령 |
| PS | 위치 초기화 (`ready`) | `move_to_named_target_async` (클라이언트는 `create_move_group_client()` 로 생성) |

`teleop_twist_joy` 의 표준 출력은 `Twist` (header 없음) 인데 본 워크스페이스 servo 는
`TwistStamped` 를 요구하므로 그대로는 못 쓴다. 자체 노드로 변환해야 한다.

매핑 측정 결과를 가지고 `joy_teleop.py` 작성 단계로 진행한다.

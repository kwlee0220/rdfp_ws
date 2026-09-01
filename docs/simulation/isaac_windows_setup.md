# Isaac Sim 백엔드 — 새 Windows 11 머신 구축 절차

**Isaac Sim 만 설치된 Windows 11 머신에서 이 저장소의 Isaac 백엔드를 처음부터
돌리기까지의 절차다.** 배포 구성 **A**(Windows Isaac + WSL2 스택, 한 머신)를 전제한다.

설계·실측·함정의 근거는 [isaac_backend_skeleton.md](isaac_backend_skeleton.md) 에 있고
이 문서는 **재현 절차만** 담는다. 막히면 그 문서의 §7 함정 사전을 증상으로 찾는다.

> **Ubuntu 한 대에서 둘 다 돌릴 계획이라면 이 문서가 아니다.** 구성 B 는 DDS 프로파일도
> 런처도 필요 없다 — [isaac_backend_skeleton.md](isaac_backend_skeleton.md) §1 B 를 본다.

---

## 요약

| 단계 | 어디서 | 요지 |
|:-:|---|---|
| 1 | Windows | WSL2 + **Ubuntu-22.04** 설치 |
| 2 | Windows | `.wslconfig` — 메모리 캡 + mirrored networking |
| 3 | WSL | ROS 2 Humble + 의존 패키지 |
| 4 | WSL | 워크스페이스 빌드 |
| 5 | Windows | **파일 2개 복사** (`C:\isaacsim\`) |
| 6 | WSL | 환경변수 4개 |
| 7 | Isaac | 로봇 올리고 스크립트 6개 실행 → 저장 |
| 8 | WSL | 검증 |

**Isaac 쪽에서 할 일은 5·7 뿐이다.** 나머지는 전부 WSL2 준비다.

---

## 1. WSL2 + Ubuntu 22.04

```powershell
wsl --install -d Ubuntu-22.04
wsl -l -q                       # 이름이 정확히 'Ubuntu-22.04' 인지 확인한다
```

**배포판 이름을 확인하는 이유가 있다.** Isaac 쪽 스크립트가 UNC 경로
(`//wsl.localhost/<배포판>/...`)로 저장소를 읽으므로 이름이 다르면 파일을 못 연다.
실제로 `Ubuntu` 와 `Ubuntu-22.04` 를 혼동해 막힌 적이 있다.

이름이 다르면 7단계에서 `RDFP_WORKSPACE` 로 덮어쓴다.

## 2. `.wslconfig`

`C:\Users\<사용자>\.wslconfig` 에 둔다. 없으면 만든다.

```ini
[wsl2]
memory=10GB              # MoveIt+RViz+recorder 에 충분
processors=12            # 전부 주면 Isaac 물리 스텝과 코어를 다툰다
swap=8GB
networkingMode=mirrored  # ★ Isaac↔WSL2 DDS 에 필요
dhcp=true
dnsTunneling=true

[experimental]
hostAddressLoopback=true    # ★ 서로 127.0.0.1 로 닿게 한다
autoMemoryReclaim=gradual
```

```powershell
wsl --shutdown              # 재기동해야 적용된다
```

**안 하면 WSL2 가 호스트 RAM 의 절반을 물고 놓지 않는다.** 32 GB 머신에서 15.4 GB 가
WSL2 로 가면 Isaac 이 최소 사양의 절반으로 돈다. WSL2 는 한 번 잡은 메모리를 자동
반환하지 않으므로 캡을 거는 것이 맞다.

## 3. ROS 2 Humble + 의존 패키지 (WSL 안)

```bash
sudo apt install ros-humble-desktop python3-colcon-common-extensions

sudo apt install \
  ros-humble-moveit ros-humble-moveit-py \
  ros-humble-moveit-resources-panda-description \
  ros-humble-moveit-resources-panda-moveit-config \
  ros-humble-controller-manager ros-humble-joint-state-broadcaster \
  ros-humble-joint-trajectory-controller ros-humble-position-controllers \
  ros-humble-robot-state-publisher ros-humble-tf-transformations \
  ros-humble-control-msgs ros-humble-xacro \
  ffmpeg python3-opencv python3-yaml
```

Isaac 만 쓸 거면 **빼도 되는 것** — Gazebo 계열(`ros-humble-ros-gz-*`,
`ros-humble-ign-ros2-control`)과 풋페달용 `python3-evdev`.

## 4. 워크스페이스

```bash
mkdir -p ~/development/ros/rdfp_ws/src
cd ~/development/ros/rdfp_ws/src
#   rdfp_msgs 는 별도 저장소다 — 먼저 받는다
#   그다음 이 저장소(robot_control / robot_twin / rdfp)

cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp
source install/setup.bash
```

**경로를 `~/development/ros/rdfp_ws` 로 맞추면** Isaac 쪽 스크립트를 손대지 않아도 된다.
그 경로가 기본값으로 박혀 있다.

> **`colcon build` 를 빠뜨리지 않는다.** `config/isaac_scene.json` 을 Isaac 쪽 스크립트는
> `src/` 에서, ROS 노드는 `install/share/` 에서 읽는다. 빌드를 안 하면 두 사본이 갈라져
> **Isaac 에만 있고 `/scene/objects` 에는 없는 물체**가 조용히 생긴다.

## 5. Isaac 쪽 — 파일 2개 복사

```bash
cp scripts/isaac/sim_side/run_isaac_humble.bat      /mnt/c/isaacsim/
cp src/robot_control/config/fastdds_wsl_bridge.xml  /mnt/c/isaacsim/
```

- Isaac 설치 경로가 `C:\isaacsim` 이 아니면 `run_isaac_humble.bat` 의 `ISAAC_ROOT` 한
  줄만 고친다.
- **프로파일 XML 은 그대로 쓴다** — 안에 든 것은 `127.0.0.1` 과 멀티캐스트 주소뿐이라
  머신 고유값이 없다.
- UNC 경로(`\\wsl.localhost\...`)에서 직접 실행하지 않는다. cmd 가 작업 디렉터리를
  잡지 못한다.

### 왜 런처가 따로 필요한가

기본 런처 `isaac-sim.bat` 은 `setup_ros_env.bat` 을 부르고, 거기에 이렇게 박혀 있다.

```bat
set DEFAULT_ROS_DISTRO=jazzy
if "%ROS_DISTRO%"=="" (
    set ROS_DISTRO=%DEFAULT_ROS_DISTRO%
    set "PATH=%PATH%;%BRIDGE_EXT_PATH%\%DEFAULT_ROS_DISTRO%\lib"
)
```

Windows 기본이 **jazzy** 라 Humble 인 스택과 서로 보이지 않는다. 그런데
`ROS_DISTRO=humble` 만 미리 넣으면 **더 나빠진다** — `if` 블록이 통째로 건너뛰어져
`humble\lib` 이 PATH 에 안 붙고 Isaac 이 크래시한다. 런처는 그 둘을 함께 설정한다.

## 6. WSL 쪽 환경변수

`~/.bashrc` 에 넣는다.

```bash
export ROS_DOMAIN_ID=31
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_LOCALHOST_ONLY=0
export FASTRTPS_DEFAULT_PROFILES_FILE=~/development/ros/rdfp_ws/src/robot_control/config/fastdds_wsl_bridge.xml
```

**프로파일은 양쪽 모두에 걸어야 한다.** 한쪽만 하면 SHM 을 고른 쪽이 상대를 못 본다.
Isaac 쪽은 런처가 이미 설정한다.

### 프로파일이 하는 일

mirrored networking 에서는 Isaac 과 WSL2 가 **같은 호스트로 보인다.** Fast DDS 가 그래서
공유메모리(SHM)를 고르는데, **Windows SHM 과 리눅스 `/dev/shm` 은 다른 물건**이라 아무것도
도착하지 않는다. 에러도 경고도 없이 토픽이 그냥 안 보인다.

| 설정 | 막는 증상 |
|---|---|
| SHM 끄고 UDPv4 강제 | 토픽이 하나도 안 보인다 |
| `maxMessageSize=1400` | **큰 것만** 안 온다 — 9 KB SRDF 조회 실패 |
| `initialPeersList` 에 포트 16개 명시 | 노드 **5개째부터** 사라진다 (포트 없이 두면 참가자 ID 0~3 만 탐색) |

## 7. Isaac scene 구성

1. `C:\isaacsim\run_isaac_humble.bat` 으로 띄운다.
2. asset browser 에서 **Franka(Panda) 를 스테이지에 올린다.** 스크립트가 로봇을 불러오지
   않는 이유는 에셋 경로가 Isaac 버전마다 바뀌기 때문이다 — 대신 articulation root 를
   자동 탐색한다.
3. **Stop(■) 상태에서** `Window > Script Editor` 에 아래를 붙여 실행한다.

```python
path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
for s in ("setup_scene", "setup_graph", "place_robot",
          "set_home_pose", "tune_drive", "tune_grasp"):
    exec(open(f"{path}/scripts/isaac/sim_side/{s}.py", encoding="utf-8").read())
```

4. **Play(▶)** 를 누른다. `OnPlaybackTick` 은 재생 중에만 tick 을 내므로 정지 상태에서
   토픽이 안 보이는 것이 정상이다.
5. `File > Save As` 로 scene 을 저장한다. 다음부터는 열고 Play 만 하면 된다.

**경로·사용자명이 다르면** 스크립트 실행 전에 환경변수를 준다. 런처에 넣어도 되고
Script Editor 에서 `os.environ` 으로 넣어도 된다.

```bat
set "RDFP_WORKSPACE=//wsl.localhost/<배포판>/home/<사용자>/development/ros/rdfp_ws"
set "RDFP_LOG_DIR=//wsl.localhost/<배포판>/tmp"
```

### 여섯 스크립트가 하는 일 (순서에 의미가 있다)

| 스크립트 | 없으면 |
|---|---|
| `setup_scene` | 테이블·블록·카메라 prim 이 없다 |
| `setup_graph` | **토픽이 하나도 안 나온다** (ROS2 브리지 그래프가 없다) |
| `place_robot` | `panda_link0` 이 월드 원점에 없어 **모든 좌표가 그만큼 어긋난다** |
| `set_home_pose` | Play 직후 팔이 탁자에 박히고 **MoveIt 이 계획 자체를 거부**한다 |
| `tune_drive` | 팔 응답이 느리고(τ 195.6 ms) 중력에 처진다 |
| `tune_grasp` | 손가락이 블록을 **미끄러뜨린다** — 집어도 안 들린다 |

`place_robot` 과 `set_home_pose` 는 **에셋을 새로 올릴 때마다** 필요하다. 저장한 scene 을
다시 여는 경우에는 값이 USD 에 남아 있으므로 건너뛴다.

> **저장된 USD 를 다른 머신에서 복사해 오지 않는다.** Franka 에셋 경로가 설치마다 달라
> 깨진다. 스크립트로 새로 만드는 것이 확실하고, 그것이 스크립트가 있는 이유다.

## 8. 검증

**배관 먼저, 그다음 스택, 그다음 단계 검사.** 순서를 지키면 원인 범위가 좁아진다.

```bash
cd ~/development/ros/rdfp_ws
source install/setup.bash

./scripts/isaac/is_topics.py                       # 1) 배관
ros2 launch robot_control panda_isaac.launch.py \
    enable_gripper:=true enable_scene:=true        # 2) 스택
./scripts/isaac/is_check_phase0.py                 # 3) ~ phase6 순서대로
```

`is_check_phase0.py` ~ `is_check_phase6.py` 가 각 단계의 수용 기준을 검사한다. 전부
통과하면 이 PC 에서 얻은 실측(팔 τ 34.9 ms, 파지 시 블록 +0.0999 m 상승 등)을 그대로
재현한 것이다.

### 수집 계층까지

```bash
ros2 launch rdfp rdfp_panda_isaac.launch.py        # 제어 + 수집 한 번에
```

## 자주 막히는 곳

**셋 다 "에러 없이 토픽이 안 보이는" 증상이라 눈으로는 구별되지 않는다.**

| 증상 | 원인 | 확인 |
|---|---|---|
| 토픽이 하나도 안 보인다 | `isaac-sim.bat` 을 직접 실행 → **jazzy** 로드 | 아래 로그 확인 |
| 토픽이 하나도 안 보인다 | 프로파일을 한쪽에만 설정 | 양쪽 환경변수 |
| 토픽이 하나도 안 보인다 | 배포판 이름 불일치로 스크립트가 파일을 못 읽음 | `wsl -l -q` |
| 토픽이 하나도 안 보인다 | Play 를 안 눌렀다 | 정상 동작이다 |
| **일부 노드만** 안 보인다 | `initialPeersList` 포트 누락 | 프로파일이 최신인지 |
| **큰 것만** 안 온다 | MTU 초과 단편 소실 | `maxMessageSize=1400` |
| 계획이 `-2` 로 거부된다 | 시작 자세가 scene 과 충돌 | `./scripts/isaac/is_recover.py` |

로드된 ROS 배포판 확인:

```bash
grep -h "internal rclpy for ROS Distro" \
  "/mnt/c/Users/$USER/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/6.0/"*.log | tail -2
```

`humble` 이 나와야 한다.

> **Script Editor 출력창은 복사가 안 된다.** 그래서 모든 sim_side 스크립트가 같은 내용을
> `/tmp/isaac_*.log` 에 남긴다. WSL 에서 `cat` 으로 읽는다.

## 선택 — 트윈과 DB

| 필요할 때 | 추가 작업 |
|---|---|
| 트윈 REST API | `ros2 run robot_twin robot_twin --config src/robot_twin/config/robot_twin_panda_isaac.yaml` (포트 8802) |
| 데이터셋 DB 적재 | PostgreSQL 설치 + `RDFP_DB_DSN` 설정 + `ros2 run rdfp init-db` |

## 하드웨어 참고

| | 이 PC 실측 | Isaac 최소 사양 |
|---|---|---|
| RAM | 32 GB | 32 GB |
| GPU | RTX 4060 / VRAM **8 GB** | RTX 4080 / **16 GB** |

**최소 사양 미달이지만 실제로는 돌았다.** 640×480 @ 5 Hz 카메라 하나가 VRAM 180 MiB ·
GPU 47% 였다. 공식 기준은 센서가 많은 복잡한 scene 을 전제한다. 다만 VRAM 8 GB 에서는
WSLg 가 같은 메모리를 나눠 쓰므로 **`enable_rviz:=false` 를 기본 운용으로 삼는다.**

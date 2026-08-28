# 실기 Franka Panda bringup — 준비 사항

작성 2026-08-23 · **준비 단계 (미착수)**

> `panda_mock` / `panda_gazebo` 처럼 **실기 Franka Panda 용 로봇 스택 launch** 를 만들려면 무엇이 필요한지 정리한다. 코드는 아직 없다 — 착수 조건과 확인해야 할 정보를 모아 둔 문서다.
>
> 이 문서의 "현재 상태" 항목은 2026-08-23 에 이 개발 PC 에서 **실측**한 값이다.

---

## 0. 요약 — 막힌 곳은 셋

| 항목 | 상태 |
|---|---|
| **로봇 정보** (모델·시스템 버전·FCI 라이선스) | ❓ 미확인 — **나머지가 전부 여기서 결정된다** |
| **franka_ros2 / libfranka** | ❌ 미설치 (`franka_description` 만 있음). 다만 **ansible role 이 준비되어 있다** — §3.1, 실행 전 수정 3건 |
| **실시간 커널 (PREEMPT_RT)** | ❌ 없음 — 현재 `PREEMPT_DYNAMIC` 일반 커널. **팔 실행에만 걸린다** — 배선 검증 단계는 이대로 진행 가능(§4.3) |

**이 표는 상태 목록이지 확인 순서가 아니다.** 무엇부터 확인해야 하는지는 §8 에 있다 —
"아니오면 전부 무의미해지는 것"을 앞에 두므로 위 나열 순서와 다르다.

launch 구조 자체는 이미 백엔드 교체를 전제로 만들어져 있으므로(§1), **위 셋이 해결되면 나머지는 기존 패턴을 따르는 작업**이다.

---

## 1. 구조는 이미 열려 있다

launch 아키텍처에 백엔드 교체용 이음매가 둘 있다.

| 이음매 | 위치 | 현재 값 |
|---|---|---|
| `ros2_control_hardware_type` 인자 | [launch_helpers/common.py](../../src/robot_control/robot_control/launch_helpers/common.py) | 기본 `mock_components`, xacro 로 전달 |
| 백엔드별 helper | [launch_helpers/gazebo.py](../../src/robot_control/robot_control/launch_helpers/gazebo.py) | "mock 과의 **차이점만** 여기 모은다" |

[panda_functionbay.launch.py](../../src/robot_control/launch/panda_functionbay.launch.py) 가 **비-mock 백엔드를 붙인 선례**다. 이미 세 번(mock / Gazebo / 펑션베이) 해 본 패턴이라 네 번째가 새로운 구조를 요구하지는 않는다.

> **"열려 있다"** 는 객체지향의 **개방-폐쇄 원칙**(Open/Closed Principle — "확장에는 **열려** 있고 수정에는 닫혀 있다")에서 온 표현이다. 새 백엔드를 꽂을 확장 지점이 미리 마련되어 있어, **기존 구조를 뜯어고치지 않고 더하기만 하면 된다**는 뜻이다. 닫혀 있다면 mock 전용으로 하드코딩되어 있어 launch·helper 를 전면 개편해야 했을 것이다.
>
> **코드를 안 고친다는 뜻은 아니다.** §5 가 고칠 것 넷을 나열한다 — 바뀌지 않는 것은 launch 배치 · helper 조립 방식 · 기동 체인 골격 · 상위 앱(트윈·MCP)의 계약이고, 더해지는 것은 xacro 한 갈래 · helper 모듈 하나 · 그리퍼 어댑터 · servo YAML 이다.
>
> 열려 있는 범위도 **로봇 스택까지**다. 그리퍼는 반쯤 열려 있고(§5.2 어댑터 필요), scene 은 열려 있지 않다(§6).
> 
---

## 2. 제공해야 할 정보

### 2.1 로봇 자체 — 이것이 먼저다

| 항목 | 왜 필요한가 | 확인처 |
|---|---|---|
| **모델** — Panda(FER) / FR3 | 지원 SW 계열이 갈린다 | 본체 라벨 |
| **시스템 버전** (예: `4.2.x`) | **libfranka 버전을 결정**하고, 그것이 다시 franka_ros2 버전을 결정한다 | Franka Desk → Settings → System |
| **FCI 활성화 여부** | FCI 없이는 외부 제어 자체가 불가능하다. 구형 Panda 는 **유료 옵션**이라 라이선스가 없으면 여기서 끝난다 | Desk 에 FCI 항목이 보이는지 |
| **컨트롤 박스 IP** (기본 `172.16.0.2`) | `robot_ip` 파라미터 | Desk 접속 주소 |
| **Franka Hand 장착 여부** | 그리퍼 어댑터 필요 여부(§5.2) | 육안 |
| **엔드이펙터 부착물의 질량·무게중심·관성** | 실기는 load 를 알려 줘야 제어가 정상 동작한다. 카메라 브래킷 등을 달았다면 필수 | 실측 또는 도면 |

> **버전 조합은 추정으로 정하면 안 된다.** 시스템 버전이 확인되면 그에 맞는 libfranka / franka_ros2 조합을 찾아야 한다. Panda(FER)는 특정 버전 이후 지원이 끊긴 이력이 있어 여기서 경로가 갈린다.

### 2.2 네트워크

| 항목 | 현재 이 PC | 정해야 할 것 |
|---|---|---|
| 유선 NIC | **`eno1`** (그 외는 wifi · docker 브리지) | 로봇 전용으로 쓸 수 있는가 |
| 호스트 IP | — | 보통 `172.16.0.1/24` |
| 로봇 IP | — | 기본 `172.16.0.2` |

FCI 는 **1 kHz UDP** 다. 기존 네트워크와 회선을 공유하면 패킷 지연으로 통신 오류가 난다. **로봇 전용 NIC 를 권장**한다.

### 2.3 운영·안전

| 항목 | 어디에 쓰이나 |
|---|---|
| 작업 공간 물리 한계 (테이블 크기, 주변 장애물) | 이동 범위 검사 · planning scene |
| 충돌 임계값 정책 | `collision_behavior`. 민감하면 계속 멈추고 둔하면 위험하다 |
| 초기 자세(`ready` named target)가 실기에서 안전한가 | mock 기준 값이 실기 셋업에서 충돌할 수 있다 |
| 비상정지 위치·복구 절차 | `error_recovery` 호출 시점 |
| 카메라 배치 (연결 예정이라면) | 씬을 담는지가 [observation 결정](../rosbag2/scene_objects_observation_decision.md)과 직결된다 |

---

## 3. 소프트웨어 — 설치 필요

현재 이 PC 에 apt 로 설치된 franka 관련 패키지는 **하나뿐**이다.

| 패키지 | 상태 | 비고 |
|---|:-:|---|
| `franka_description` | ✅ **1.0.1** | `fer` · `fr3` · `fp3` · `fr3_duo` · `fr3v2` 모델과 `franka_hand` 포함 |
| `libfranka` | ❌ | **Franka Robotics 가 배포하는 FCI 프로토콜 C++ 클라이언트** (Apache 2.0, ROS 패키지가 아니라 일반 CMake 라이브러리다). 소스 빌드이며 의존은 Poco · Eigen3. **쓸 수 있는 버전이 로봇 모델(FER/FR3)과 시스템 버전에 묶인다** — §2.1 |
| `franka_ros2` (`franka_hardware` · `franka_gripper` · `franka_msgs`) | ❌ | Humble 브랜치 소스 빌드 |

즉 **URDF·메시는 있지만 하드웨어를 잡을 ros2_control 플러그인이 없다.**

확인 명령:

```bash
dpkg -l | grep -i franka
ls /opt/ros/humble/share/franka_description/robots/
```

### 3.1 ansible role 이 이미 있다

`~/ansible/roles/franka_ros2` 가 위 둘을 소스 빌드로 채운다. **아직 실행 전이다** (워크스페이스도 `.bashrc` 블록도 없음, 2026-08-23 확인).

| 단계 | 결과 |
|---|---|
| `git clone frankarobotics/franka_ros2` → `~/local/ros2/franka_ros2_ws/src` | `franka_hardware` · `franka_gripper` · `franka_msgs` |
| `vcs import src < src/dependency.repos --recursive` | **`libfranka` 가 여기서 딸려 온다** |
| `rosdep install` | Poco · Eigen3 등 시스템 의존 |
| `colcon build --symlink-install -DCMAKE_BUILD_TYPE=Release` | 빌드 |

빌드 도구(`vcs` / `rosdep` / `colcon` / `bloom-generate` / `fakeroot`)는 전부 설치되어 있다.

**실행 전에 고쳐야 할 것이 셋 있다.**

#### (1) `.bashrc` 자동 source 태스크는 빼야 한다

role 의 마지막 태스크가 `~/.bashrc` 에 무조건 source 를 넣는다.

```yaml
- name: Add workspace sourcing to .bashrc using blockinfile
  block: |
      source {{ workspace_dir }}/install/setup.bash
```

**이 머신은 정확히 그 방식을 걷어낸 구성이다** — [python_env_guide.md §2.1](../environment/python_env_guide.md) 이 "과거 `~/.bashrc` 는 ROS 환경을 무조건 적용했다 → 모든 셸이 오염된다"를 기록하고, 그래서 `ros2_env` / `rdfp_env` 옵트인 함수로 바꾼 것이다. 이 블록이 들어가면 둘이 동시에 깨진다.

1. **모든 셸이 다시 오염된다** — `install/setup.bash` 가 `/opt/ros/humble` 을 chain 하므로 ROS 와 무관한 uv 프로젝트 셸에도 `PYTHONPATH` · `AMENT_PREFIX_PATH` 가 박힌다.
2. **`ROS_DOMAIN_ID` 가 0, RMW 가 fastrtps 로 뜬다** — `.ros2rc` 를 거치지 않으므로 domain 31 / cyclonedds 가 아니다. [replay_mock_stack_guide §3-1](../replay/replay_mock_stack_guide.md) 에 정리된 **"노드는 떴는데 토픽이 안 보인다"** 함정 그 자체다.

**대응**: 그 태스크를 빼고 `franka_env` 같은 옵트인 함수로 만든다. 이미 실행했다면 `.bashrc` 의 `# BEGIN Franka ROS2:` 블록을 제거한다.

#### (2) 버전이 고정돼 있지 않다 — §2.1 과 직결된다

```yaml
- name: Clone franka_ros2 repository
  git:
    repo: 'https://github.com/frankarobotics/franka_ros2.git'
    dest: "{{ workspace_dir }}/src"
    # version: 이 없다 → 기본 브랜치를 그대로 받는다
```

`ros_distro: humble` 은 rosdep / colcon 쪽 인자일 뿐 **체크아웃되는 코드가 humble 용이라는 보장이 아니다.** 최신 브랜치가 FR3 전용이면 구형 Panda(FER)에는 아예 붙지 않는다.

**파급이 franka_ros2 하나로 끝나지 않는다.** libfranka 를 role 이 직접 지정하지 않고 `dependency.repos` 로 끌어오므로, **브랜치 하나가 libfranka 버전까지 연쇄로 결정한다.**

```
franka_ros2 브랜치 (미지정)  →  src/dependency.repos  →  libfranka (그 브랜치가 정한 버전)
```

libfranka 버전이 어긋나면 컴파일은 되고 **연결 시점에 거부된다** — 빌드를 다 마친 뒤 로봇에 붙이는 순간 드러나므로 미리 맞추는 편이 시간을 아낀다.

```
libfranka: incompatible library version. Server version: X, library version: Y
```

**대응**: `version:` 을 명시한다. 그 값은 **§2.1 의 시스템 버전을 확인해야 정할 수 있으므로, role 실행보다 로봇 정보 확인이 먼저다.**

#### (3) `franka_deb_builder.sh` 의 경로가 어긋나 있다

```bash
WS_DIR="$HOME/franka_ros2_ws"                          # 스크립트
workspace_dir: "{{ ... }}/local/ros2/franka_ros2_ws"   # vars (이전 경로는 주석 처리됨)
```

vars 를 옮기면서 스크립트가 따라가지 않았다. 지금 실행하면 `cd $WS_DIR/src` 에서 실패한다. `.deb` 패키징을 쓸 계획이 아니면 `tasks/main.yml` 과 무관하므로 무시해도 된다.

#### role 이 해 주지 않는 것

RT 커널(§4.1) · realtime 권한(§4.2) · 네트워크 설정(§2.2) · 이 워크스페이스의 코드 변경(§5)은 전부 별도다.

---

## 4. 시스템 설정 — 이 PC 는 아직 준비되지 않았다 (팔 실행 기준)

### 4.1 실시간 커널 (PREEMPT_RT)

**PREEMPT_RT 는 커널 코드를 거의 전부 선점 가능하게 만들어, 우선순위 높은 프로세스가 정해진 시간 안에 반드시 CPU 를 받도록 보장하는 설정**이다. 핵심은 평균 속도가 아니라 **최악의 경우 지연에 상한이 있느냐**다.

| | 일반 커널 | PREEMPT_RT |
|---|---|---|
| 평균 지연 | 빠르다 | 비슷하거나 약간 느리다 |
| **최악 지연** | **보장 없음** — 수 ms ~ 수십 ms 튀는 일이 있다 | 수십 µs 수준으로 **상한이 있다** |
| 처리량 | 높다 | 약간 손해 |

**왜 필요한가** — FCI 는 1 kHz 루프이므로 **매 1 ms 마다** 응답해야 한다. 일반 커널에서는 디스크 I/O · 인터럽트 · 다른 프로세스 때문에 제어 프로세스가 몇 ms 밀리는 일이 생기고, libfranka 는 이를 통신 실패로 보아 제어를 중단한다.

```
libfranka: UDP receive: Timeout
control loop timeout
```

**동작 중에 로봇이 갑자기 멈추는** 형태로 나타나므로 안전 문제이기도 하다.

**현재 상태** — 실측:

```
$ uname -r
6.8.0-136-generic

$ grep -E "CONFIG_PREEMPT(_RT|_DYNAMIC|_VOLUNTARY)?=" /boot/config-$(uname -r)
CONFIG_PREEMPT_VOLUNTARY=y
CONFIG_PREEMPT_DYNAMIC=y
```

`CONFIG_PREEMPT_RT` 가 **없다**. `PREEMPT_DYNAMIC` 은 부팅 시 선점 강도를 고를 수 있게 한 기능이지 실시간 커널이 아니다 — 고를 수 있는 가장 센 `full` 조차 PREEMPT_RT 와 다르다. RT 는 그 위에서 **스핀락을 잠들 수 있게 바꾸고 인터럽트를 스레드화**해 지연의 상한을 만든다.

| 모드 | 이 PC | 뜻 |
|---|:-:|---|
| `PREEMPT_NONE` | | 처리량 우선, 서버용 |
| `PREEMPT_VOLUNTARY` | **기본값** | 정해진 지점에서만 양보 |
| `PREEMPT_FULL` | 선택 가능 | 커널 코드 대부분 선점 가능 |
| **`PREEMPT_RT`** | **불가** | 위 + 인터럽트 스레드화 → **지연 상한 보장** |

**얻는 방법** — 둘 다 커널 교체와 재부팅을 수반하므로 **사용자 결정이 필요하다.**

1. **Ubuntu Pro 실시간 커널** — `pro enable realtime-kernel`. 22.04 에서 가장 간단하다.
2. **직접 빌드** — 커널 소스 + RT 패치. 22.04 기본 저장소에는 RT 이미지가 없다(조회 확인).

확인:

```bash
uname -v | grep -q PREEMPT_RT && echo "RT 커널" || echo "RT 아님"
```

### 4.2 실시간 우선순위 권한

커널만으로는 부족하고 프로세스가 실시간 우선순위를 **쓸 수 있어야** 한다. 현재 `/etc/security/limits.conf` 에 관련 설정이 **없다**(실측).

```
@realtime  -  rtprio   99
@realtime  -  memlock  unlimited
```

사용자를 `realtime` 그룹에 넣는다. 스왑으로 페이지가 디스크에 내려가면 그 접근 자체가 ms 단위 지연이 되므로 `memlock` 도 함께 필요하다.

### 4.3 없으면 아예 안 되나 — **아니다. 팔 실행에만 걸린다**

libfranka 는 채널이 둘이고, **RT 가 필요한 것은 그중 하나뿐**이다.

```
libfranka ─┬─ 비실시간 (TCP)   : 상태 읽기 · 그리퍼 · 파라미터 설정 · 에러 복구
           └─ 실시간 (1kHz UDP): 팔 제어 루프          ← RT 가 필요한 건 여기뿐
```

| 기능 | RT 없이 |
|---|---|
| FCI 연결 · 핸드셰이크 | ✅ 문제없다 |
| **로봇 상태 읽기** (`/joint_states` · TF · EE pose) | ✅ 문제없다 |
| **그리퍼** (grasp / move / homing) | ✅ **설계상 비실시간 인터페이스**라 무관하다 |
| `collision_behavior` 설정 · `error_recovery` | ✅ 문제없다 |
| MoveIt **계획만** (실행 없이) | ✅ 로봇과 무관한 계산이다 |
| **팔 실행** (JTC / servo) | ⚠️ **여기만** — 짧게는 되고 길수록 깨진다 |

#### 팔을 움직이면 실제로 어떻게 되나

크래시가 아니라 **로봇 자체의 안전 정지**다. 커널이 제어 프로세스를 몇 ms 붙잡으면 주기를
놓치고, 로봇이 이를 통신 이상으로 보아 감속 정지한다.

```
franka::ControlException: communication constraints violation
```

하드웨어가 상하지는 않지만 **예고 없이 멈추고**, 이어서 `error_recovery` 를 불러야 다시
움직인다. **시간에 비례해 확률이 올라간다** — 몇 초짜리 저속 이동은 대체로 통과하고,
수십 초~분 단위로 계속 움직이면 거의 확실히 걸린다. 즉 짧게는 되지만 **재현성이 없다.**

> libfranka 에는 실시간 강제를 끄는 옵션(`RealtimeConfig`)이 있다. 다만 그것이
> `franka_hardware`(ros2_control 플러그인)에서 파라미터로 노출되는지는 **확인이 필요하다** —
> 노출되지 않으면 기본값이 강제라 아예 기동이 거부될 수도 있다.

#### 이 워크스페이스는 특히 유리하다

수집 계층이 **거의 전부 읽기 전용**이다. `config/recording_topics.list` 에 팔 실행을
요구하는 항목이 사실상 없다.

| 검증 대상 | RT 없이 |
|---|:-:|
| `robot_state_publisher` · TF · RViz | ✅ |
| `ee_pose_node` (`/joint_states` → FK) | ✅ |
| `gripper_control_node` ↔ **franka_gripper 어댑터** (§5.2) | ✅ **핵심 작업인데 RT 무관** |
| `session_control_node` · `image_recorder` · rosbag2 녹화 | ✅ |
| dataset import → DB 적재 → replay | ✅ |
| robot twin 변수 조회 · MCP 도구 | ✅ (이동 연산 제외) |
| xacro `franka` 분기 · launch 기동 체인 | ✅ |

즉 §5 의 코드 변경 넷 중 **1 · 2 · 4 는 RT 없이 검증까지 끝낼 수 있다.** 3번(servo
파라미터)만 실제 이동이 필요하다.

#### 단계를 나눈다

| 단계 | RT | 할 수 있는 것 |
|---|:-:|---|
| **1. 배선 검증** | 불필요 | 연결 · 상태 · 그리퍼 · 녹화 · 적재 전 경로 |
| **2. 짧은 이동 시험** | 없어도 시도 가능 | 저속 · 단거리. **끊길 수 있음을 전제로**, 주변에 사람 없이 |
| **3. 실제 수집** | **필요** | 에피소드 중간에 멈추면 그 데이터는 버려야 한다 |

1단계에서 **가이딩 모드로 팔을 손으로 움직이며 `/joint_states` 를 녹화**하면, 실기 데이터로
수집 · 적재 · 재생 경로 전체를 RT 없이 검증할 수 있다.

**RT 커널은 "3단계를 하겠다"의 전제 조건**이지 실기를 붙이는 것 자체의 조건은 아니다.

---

## 5. 코드에서 고쳐야 할 것 — 넷

### 5.1 xacro 에 `franka` 분기 추가

[description/panda.ros2_control.xacro](../../src/robot_control/description/panda.ros2_control.xacro) 가
`mock_components` / `isaac` / `gazebo` 세 갈래뿐이다. 네 번째를 넣는다.

```xml
<xacro:if value="${ros2_control_hardware_type == 'franka'}">
    <plugin>franka_hardware/FrankaHardwareInterface</plugin>
    <param name="robot_ip">${robot_ip}</param>
</xacro:if>
```

**`robot_ip` 라는 새 파라미터가 필요한 점이 앞선 셋과 다르다** — mock/gazebo/isaac 은 인자 하나로 끝났지만 실기는 접속 정보가 있어야 한다.

> 상류 `moveit_resources_panda_moveit_config` 의 xacro 는 `mock_components` / `isaac` 두 갈래뿐이다. 그래서 Gazebo 백엔드가 이미 **자체 description** ([description/](../../src/robot_control/description/))을 쓰고 있고, franka 도 같은 경로를 탄다.

### 5.2 그리퍼 어댑터

Franka Hand 는 `ros2_control` 컨트롤러가 아니라 **별도 `franka_gripper` 액션 서버** (`grasp` / `move` / `homing`)로 움직인다. 반면 이 스택은 `/panda_hand_controller/gripper_cmd` (`control_msgs/GripperCommand`) 를 전제한다.

| 안 | 내용 | 평가 |
|---|---|---|
| **A** | `gripper_control_node` 에 franka 백엔드를 추가해 `GripperCommand` → `franka_gripper` 액션으로 번역 | **권장** — 계약이 유지되어 상위(트윈·MCP)가 무변경 |
| B | 트윈 설정에서 그리퍼 연산을 다시 매핑 | 계약이 백엔드마다 갈라진다 |

"백엔드 차이는 어댑터가 흡수한다"는 [scene 계약](../scene/scene_objects_guide.md)과 같은 원칙이다.

### 5.3 servo 설정 교체

[common.py](../../src/robot_control/robot_control/launch_helpers/common.py) 의 `build_servo_params()` 가 `panda_**simulated**_config.yaml` 을 읽는다. 실기는 속도·가속도 한계와 충돌 임계가 달라 별도 YAML 이 필요하다.

### 5.4 기동 체인에 안전 단계 추가

mock 은 `ros2_control_node` → broadcaster → 컨트롤러면 끝이지만, 실기는 앞뒤가 붙는다.

```
FCI 연결 확인 → collision_behavior 설정 → (기존 체인) → error_recovery 준비
```

펑션베이의 `readiness_gate`(첫 관절 보고를 기동 완료 신호로 삼고 종료하는 노드)가 정확히 이런 용도의 선례다.

---

## 6. scene 은 별도 과제다

로봇 스택은 붙지만 [`/scene/objects`](../scene/scene_objects_guide.md) 는 **인식 노드를 만들기 전까지 채워지지 않는다.** `mock_scene_state_node` 는 MoveIt planning scene 을 읽는데, 실기의 planning scene 에는 누가 넣어 주지 않는 한 물체가 없다.

| 경로 | 실기에서 |
|---|---|
| 읽기 (`/scene/objects`) | ⚠️ 발행 노드를 새로 만들어야 한다 — AprilTag / 모션캡처 / pose estimation |
| 쓰기 (`reset_scene`) | ❌ **원리적으로 불가.** 물체를 순간이동시킬 수단이 없다. 연산을 노출하지 않는 것이 자연스럽다 |

즉 `panda_franka.launch.py` 는 **팔·그리퍼·카메라·수집 경로까지는 정상 동작**하고 scene 만 남는다. 이는 [rdfp_framework_design.md §9](../rdfp_framework_design.md) 미결 2번 ("시뮬레이터는 ground truth 가 있고 실기는 인식이 필요하다 — 두 경로의 인터페이스를 하나로 정의해야 한다")과 같은 항목이다.

---

## 7. 지금 할 수 있는 것 / 로봇이 필요한 것

| 로봇 없이 가능 | 로봇 · SW 필요 |
|---|---|
| xacro `franka` 분기 (§5.1) | 실제 기동 검증 |
| `panda_franka.launch.py` 골격 + `launch_helpers/franka.py` | 컨트롤러 튜닝 |
| 그리퍼 어댑터 설계·구현 (§5.2) | servo 파라미터 실측 |
| 기동 체인 설계 (§5.4) | 충돌 임계값 결정 |
| ansible role 수정 3건 후 실행 (§3.1) — 단 `version:` 은 §2.1 확인 후 | — |

---

## 8. 최소 착수 조건 — 답이 필요한 셋

나머지는 나중에 채워도 되지만, 이 셋 없이는 첫 줄도 쓸 수 없다. **"아니오면 전부
무의미해지는 것"부터** 확인한다 — 앞의 둘은 프로젝트를 중단시키는 항목이고, 셋째는
"할 수 있는가"가 아니라 "무엇을 깔지"의 문제다.

| 순위 | 확인 | 아니오라면 |
|:-:|---|---|
| **1** | **FCI 라이선스가 있는가** | 시스템 버전이 무엇이든 **외부 제어 자체가 불가능**하다. 여기서 끝난다 |
| **2** | **RT 커널을 깔 수 있는가** (이 PC 에) | **실제 수집 단계에서만** 막힌다 — 배선 검증 · 그리퍼 · 녹화 · 적재는 RT 없이 된다(§4.3). 다만 없으면 **수집은 성립하지 않는다** |
| 3 | **모델 + 시스템 버전** | 중단 사유는 아니지만 **SW 버전 사슬 전체가 여기서 결정된다** (§2.1) |

### 실제로는 이 순서로 움직인다

1 과 3 은 **Franka Desk 한 세션에서 함께 확인**되므로 작업으로는 하나다.

1. **로봇 전원 + Desk 접속** ← 진짜 첫 행동. 그 전제로 로봇 실물 접근과 유선 연결이 필요하다
2. 그 자리에서 **FCI 유무 · 모델 · 시스템 버전 · 컨트롤 박스 IP** 를 한꺼번에 기록한다
3. **병행**: RT 커널 설치 의향 결정 — 로봇과 무관하므로 동시에 진행할 수 있다

로봇이 아직 없다면, 확인을 기다리는 동안 §7 의 "로봇 없이 가능" 항목(xacro 분기 · launch
골격 · 그리퍼 어댑터 설계)을 먼저 해 두는 편이 낫다.

---

## 9. 참고

- [robot_control/launch/README.md](../../src/robot_control/launch/README.md) — 제어 계열 launch
  인벤토리와 helper 목록. 새 백엔드는 여기 §5 의 helper 패턴을 따른다
- [simulation/multi_simulator_backend_design.md](../simulation/multi_simulator_backend_design.md)
  — §5 백엔드 계약. 실기도 이 계약(`/joint_states` · `FollowJointTrajectory` · TF)을
  만족시키면 상위 앱은 무변경이다
- [simulation/functionbay_backend_design.md](../simulation/functionbay_backend_design.md) —
  비-mock 백엔드를 붙인 실제 사례. `readiness_gate` · `joint_state_fusion` 패턴
- [scene/scene_objects_guide.md](../scene/scene_objects_guide.md) — §6 이 scene 발행 노드
  계약. 실기용 인식 노드를 만들 때의 기준
- `~/ansible/roles/franka_ros2` — franka_ros2 + libfranka 소스 빌드 role (§3.1)
- [rdfp_framework_design.md](../rdfp_framework_design.md) — §2.2 가 "② 실/가상 로봇 환경"에
  `Franka Panda(실기 · mock)` 을 이미 범위로 명시, §9 미결 2번이 인식 경로

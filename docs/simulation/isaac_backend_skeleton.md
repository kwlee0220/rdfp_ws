# Isaac Sim 백엔드 — 골격 설계와 단계 계획

> **설계 근거** [multi_simulator_backend_design.md](multi_simulator_backend_design.md) §6.3 ·
> **선행 사례** [functionbay_backend_design.md](functionbay_backend_design.md) ·
> [functionbay_open_work.md](functionbay_open_work.md)
>
> Isaac Sim 을 `robot_control` 의 시뮬레이터 백엔드로 붙이는 작업의 **골격과 진행
> 순서**를 정한다. 배포 구성 네 가지(§1)에서 같은 그래프·같은 검사로 돌아간다. 한 번에 전부 붙이지 않고 **팔 → 그리퍼 → scene → 카메라** 순으로
> 한 축씩 열며, 각 단계는 *launch 인자 하나 + 검증 스크립트 하나 + 수용 기준*으로 닫는다.

---

## 0. 결정 사항 요약

| # | 결정 | 근거 |
|:-:|---|---|
| D1 | **연동 방식은 토픽 브리지 (설계 문서 §6.3 의 B안)** | A안(`ros2_control` 하드웨어 플러그인)은 Isaac 이 `controller_manager` 를 호스팅해야 하는데 Windows 에서 불가능하다. 두 호스트에서 같은 코드를 쓰려면 B안뿐이다 |
| D2 | **경계 메시지는 표준 타입만** (`rdfp_msgs` 금지) | Isaac 의 WSL2 경로는 custom ROS interface 미지원. Ubuntu 머신에서는 가능하지만 쓰면 두 환경이 갈라진다 |
| D3 | **팔 명령은 `sensor_msgs/JointState`** | Isaac OmniGraph 가 기본 제공(`ROS2SubscribeJointState`). 펑션베이와 계약이 같아 `MoveGroupJgpcClient(arm_command_format='joint_state')` 를 그대로 쓴다 |
| D4 | **`use_sim_time:=true` 전역** | Isaac 은 `/clock` 을 발행한다. 펑션베이(§6.3 에서 sim time 포기)와 갈리는 가장 큰 지점이며, **시계 오프셋 문제가 구조적으로 사라진다** |
| D5 | **구성별 분기는 DDS 설정과 경로뿐** | 노드 그래프·토픽 이름·QoS 는 배포 구성 네 가지에서 동일해야 한다. §1 참조 |
| D6 | **단계 게이팅은 launch 인자로** | `enable_gripper` / `enable_scene` 기본 `false`. 켜지 않은 단계의 노드는 아예 뜨지 않는다. 카메라는 노드가 아니라 Isaac 그래프가 발행하므로 인자가 없다 |

---

## 1. 배포 구성 네 가지

Isaac 과 스택을 어디에 두느냐로 넷이 갈린다. **노드 그래프·토픽 계약·검사 스크립트는
네 구성 모두에서 같다.** 달라지는 것은 아래 표의 네 축뿐이다.

| | 구성 | Isaac | 스택 | 상태 |
|:-:|---|---|---|---|
| **A** | Windows + WSL2 (한 머신) | Windows | WSL2 | **검증 완료** (2026-08-29) |
| **B** | Ubuntu 22.04 한 머신 | Ubuntu | 같은 Ubuntu | 미검증 |
| **C** | Ubuntu 두 대 | Ubuntu #1 | Ubuntu #2 | 미검증 |
| **D** | Windows + Ubuntu 두 대 | Windows | Ubuntu | 미검증 |

| 축 | A | B | C | D |
|---|---|---|---|---|
| **DDS 전송** | 프로파일 **필수** (SHM 끄기 + initialPeers) | **없음** (SHM 사용) | 기본 UDP. 멀티캐스트 안 되면 initialPeers | 기본 UDP. 방화벽·멀티캐스트 확인 |
| **Isaac 실행** | `run_isaac_humble.bat` | `isaac-sim.sh` | `isaac-sim.sh` | `run_isaac_humble.bat` |
| **저장소 사본** | 공유 (UNC) | 공유 (로컬) | **Isaac 머신에 별도 필요** | **Isaac 머신에 별도 필요** |
| **시계** | 같은 머신 | 같은 머신 | **NTP 필요** | **NTP 필요** |

### 공통 (네 구성 전부)

```bash
ROS_DOMAIN_ID=31            # 양쪽이 같아야 한다. 다르면 에러 없이 서로 안 보인다
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ROS_LOCALHOST_ONLY=0
```

Isaac 쪽 `ROS2Context` 노드는 `useDomainIDEnvVar=False` + `domain_id=31` 로 값을 못박는다
(`setup_graph.py`). GUI 로 띄운 프로세스에 환경변수가 없어 도메인 0 으로 떨어지는 것을
막기 위해서다.

**검증 순서도 같다** — `is_topics.py` 로 배관을 먼저 보고, 그다음 스택, 그다음 단계 검사.

---

### A. Windows + WSL2 (한 머신) — 현재 구성

**이 구성만 DDS 프로파일이 필수다.** mirrored networking 에서 WSL2 가 Windows 의
네트워크 정체를 그대로 쓰는 탓에 Fast DDS 가 양쪽을 같은 호스트로 오판해 SHM 을
고르고, Windows SHM 과 Linux `/dev/shm` 은 다른 물건이라 아무것도 도착하지 않는다.
자세한 경위는 §"Windows ↔ WSL2 DDS" 를 본다.

```
Windows                                     WSL2
  run_isaac_humble.bat                        export FASTRTPS_DEFAULT_PROFILES_FILE=...
    ROS_DISTRO=humble                         ros2 launch robot_control panda_isaac...
    PATH += humble\lib
    FASTRTPS_DEFAULT_PROFILES_FILE=C:\isaacsim\fastdds_wsl_bridge.xml
```

```bash
# WSL2 쪽 — .bashrc 에 넣어 두면 편하다
export FASTRTPS_DEFAULT_PROFILES_FILE=~/development/ros/rdfp_ws/src/robot_control/config/fastdds_wsl_bridge.xml
```

- `.wslconfig` 에 `memory=10GB` · `autoMemoryReclaim=gradual` · `networkingMode=mirrored`
  (§5). 안 하면 WSL2 가 호스트 RAM 의 절반을 물고 Isaac 이 최소 사양의 절반으로 돈다.
- sim_side 스크립트는 UNC(`//wsl.localhost/...`)로 저장소를 본다 — 사본이 하나뿐이라
  저장소를 고치면 Isaac 쪽에도 즉시 반영된다.

### B. Ubuntu 22.04 한 머신

**가장 단순하다.** A 에서 걸리적거리던 것이 전부 사라진다.

```bash
unset FASTRTPS_DEFAULT_PROFILES_FILE    # ★ 반드시. SHM 이 진짜로 같은 호스트다
./isaac-sim.sh                          # 런처 불필요
```

- **배포판 자동 선택** — Linux 에서는 `system_default` 가 Ubuntu 버전을 보고 humble 을
  고른다. Windows 에서 jazzy 로 떨어지던 문제가 없다.
- WSL 용 프로파일을 들고 오면 **SHM 을 막아 느려질 뿐 이득이 없다.**
- 워크스페이스를 source 한 셸에서 Isaac 을 띄우면 시스템 ROS 2 를 쓴다(내부 라이브러리가
  아니라). 정상 동작하며 custom message 도 가능해지지만, **D2 에 따라 경계는 표준
  타입만** 유지한다 — 쓸 수 있다고 쓰면 두 구성의 코드가 갈라진다.

### C. Ubuntu 두 대

진짜로 다른 호스트이므로 Fast DDS 가 알아서 UDP 를 고른다. **프로파일은 기본적으로
필요 없다.**

```bash
# 양쪽 모두
export ROS_DOMAIN_ID=31
export ROS_LOCALHOST_ONLY=0
```

새로 챙길 것이 셋이다.

1. **저장소 사본** — `setup_scene.py` 가 `config/isaac_scene.json` 을 읽는데, 그 파일은
   스택 머신에 있다. Isaac 머신에도 사본이 있어야 하고(NFS 마운트·rsync·git clone 중
   택일), 스크립트에 위치를 알려 준다.

   ```bash
   # Isaac 머신에서
   export RDFP_WORKSPACE=/home/<user>/rdfp_ws
   export RDFP_LOG_DIR=/tmp
   ```

   **사본이 갈라지면 물체 크기가 두 곳에서 달라진다** — 단일 진실원본으로 JSON 을 둔
   의미가 사라지므로 NFS 나 rsync 로 한 방향 동기화를 권한다.
2. **방화벽** — `ufw` 를 쓴다면 도메인 31 의 DDS 포트를 연다.

   ```bash
   sudo ufw allow proto udp from <상대 IP> to any port 15150:15200
   ```
3. **시계** — 두 머신에 `chrony`/NTP. `use_sim_time:=true` 라 `/clock` 을 함께 쓰므로
   대부분 면역이지만, 로그 시각과 rosbag 파일 타임스탬프가 어긋나면 추적이 어렵다.
   펑션베이에서 2.22 초 오프셋으로 데이터셋이 오염될 뻔한 사례가 있다.

**멀티캐스트가 스위치에서 막히면** A 와 같은 방식으로 상대 IP 를 initialPeers 에 넣는다
(주소만 `127.0.0.1` → 상대 IP 로 바꾼다). 증상은 A 에서 본 것과 같다 — 한쪽만 상대를
보고, 발행은 되는데 수신이 0 이다.

**Phase 4 (카메라)는 이 구성에서 대역폭을 먼저 본다.** 1280×720 rgb8 이 60 Hz 면
약 1.6 Gbps 다. `CompressedImage` 로 바꾸거나 해상도·주파수를 낮춘다.

### D. Windows + Ubuntu 두 대

A 의 Windows 쪽 준비와 C 의 두 머신 준비를 합친 것이다. 다만 **DDS 프로파일은 A 와
달리 기본적으로 필요 없다** — 진짜 다른 호스트라 SHM 오판이 생기지 않는다.

```
Windows 머신                          Ubuntu 머신
  run_isaac_humble.bat                  export ROS_DOMAIN_ID=31
    ROS_DISTRO=humble                   ros2 launch robot_control panda_isaac...
    PATH += humble\lib
    ROS_DOMAIN_ID=31
    (FASTRTPS_DEFAULT_PROFILES_FILE 는 빼도 된다)
  set RDFP_WORKSPACE=C:\rdfp_ws
```

- `run_isaac_humble.bat` 의 배포판·PATH 설정은 **여전히 필요하다.** 벤더 런처가
  jazzy 를 기본으로 넣는 것은 머신 대수와 무관하다.
- 저장소 사본·방화벽·NTP 는 C 와 동일하다. Windows 방화벽에서 `NVIDIA Omniverse Kit`
  의 인바운드를 사설 네트워크에 허용해야 한다.
- `RDFP_WORKSPACE` 를 Windows 경로로 지정한다 — 기본값이 WSL UNC 라 그대로 두면
  없는 경로를 본다.

---

### 불변식 — 구성이 달라도 이것은 같다

**launch 파일과 노드 그래프는 어느 구성인지 모른다.** 차이는 전부 launch 바깥
(환경변수·`.wslconfig`·방화벽)에서 흡수한다. 그래서 `host_profile` 같은 launch 인자를
두지 않는다.

**검증 방법**: 두 구성에서 `ros2 node list` 와 각 토픽의 엔드포인트를 떠서 **diff 가
비어야 한다.** `rdfp_collect` 조합 동등성을 확인할 때 쓴 것과 같은 기법이다.

> **유일한 예외는 Phase 4 (카메라)다.** raw `Image`(1280×720 rgb8 ≈ 2.7 MB/frame)를
> 경계 너머로 보내는 것은 A·C·D 에서 무리라 `CompressedImage` 가 필요하다. 예외를
> 이 한 곳으로 묶고, 조건 분기가 다른 데로 번지지 않게 한다.

## 2. 단계 계획

**진행 상황** — Phase 0~9 완료, **Phase 10 은 구현만** (Phase 0~5 2026-08-29, Phase 6~10 2026-08-30). 미검증: 토폴로지 B~D(§2 Phase 8), servo 실기 주행(§2 Phase 10).

> ## Isaac Sim 6.0 · Ubuntu 재검증 — 2026-09-02
>
> **기동은 [`scripts/run_isaac_sim.sh`](../../scripts/run_isaac_sim.sh) 로 한다** —
> 환경 변수를 틀리면 ROS 확장이 조용히 죽는다(②).
>
> | | |
> |---|---|
> | `--gui` | **창 + 전 과정 자동.** 평소 쓰는 것 — Script Editor 를 안 쓴다 |
> | `--headless` | 창 없이 같은 일 (검사·CI). 아래 검증이 이것으로 재현됐다 |
> | (인자 없음) | 전체 편집기. 자산 브라우저·프로퍼티 편집이 필요할 때만 |
>
> **스테이지를 저장해 두는 방식은 쓰지 않는다.** 저장하면 물체 이름·크기가 USD 와
> `isaac_scene.json` 두 곳에 살게 되고 조용히 갈라진다 — JSON 은
> `isaac_scene_state_node` 도 읽는 정본이다.
>
> Phase 0~9 는 **Windows Isaac 5.1 + WSL2**(구성 A)에서 통과한 것이라, Ubuntu 단일
> 머신(구성 B) + Isaac 6.0 에서 다시 돌렸다.
>
> | Phase | 결과 | 비고 |
> |:-:|---|---|
> | 0 골격 | ✅ **5/5** | 배속 0.97, `/joint_states` 9관절 |
> | 1 팔 명령 | ⚠️ **4/5** | τ 59.9 ms (기준 ≤50). `tune_drive.py` 로 177→60 ms. 정착 오차는 0.0001 rad 로 통과 |
> | 2 그리퍼 | ✅ **6/6** | |
> | 3 scene 객체 | ✅ **5/5** | 검사 쪽 수정 후 (아래) |
> | 4 카메라 | ✅ **5/5** | 4.57 Hz · 640×480 rgb8 · camera_info. 크래시는 재현되지 않았다 (아래) |
> | 6 물리 파지 | ✅ **6/6** | 폭 0.0251 m·`stalled`, +9.8 cm 들어올림, 3초 유지 낙하 0.6 mm |
>
> **pick-and-place 는 Isaac 6.0 에서 동작한다.** 카메라만 별도 작업이다.
>
> **① Phase 4 카메라 — 크래시는 재현되지 않았고, 카메라는 정상이다.**
> 처음에는 "Play 시 segfault" 로 판단해 `PHASE` 를 3 으로 낮췄다. 그 크래시는
> `isaacsim.exp.full.kit` GUI 세션에서 **한 번** 났을 뿐이고, 스크립트 기동에서는
> **헤드리스와 창 모드 둘 다 PHASE=4 로 통과**한다. 검사도 5/5 다 (4.57 Hz,
> 640×480 rgb8, camera_info fx=733). `PHASE` 기본값을 4 로 되돌렸다.
>
> 다만 크래시가 났던 것 자체는 사실이다 — 로그의 마지막 활동이 렌더 프로덕트 attach
> 였고 파이썬 스레드는 전부 idle 이었다. 전체 GUI 앱에서 다시 겪으면 `--phase 3` 으로
> 조작 계열만 돌릴 수 있다.
>
> **`frameSkipCount` deprecation 은 그대로 둔다.** 경고가 안내하는 대체 수단
> (`omni:sensor:tickRate` + `frameSkipCount=0`)을 실측했는데 **듣지 않았다** —
> `tickRate=5.0` 인데 5 Hz 가 아니라 9.9 Hz 가 나왔다(매 프레임 발행으로 파이프라인
> 포화). Isaac 트리에서도 그 속성은 라이다·음향 센서에만 쓰이고 `UsdGeom.Camera` 에는
> 배선돼 있지 않다.
>
> **② 로봇 배치가 자동화돼 있지 않았다 — 해소.** §7 은 "asset browser 에서
> 올린다"는 수동 단계로만 적어 두었고 경로도 없어 스크립트만으로는 스테이지를
> 재현할 수 없었다. `scripts/isaac/sim_side/load_robot.py` 를 만들어 GUI·헤드리스가
> **같은 목록**을 돌린다. 6.0 경로는
> `{assets_root}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` 다
> (5.x 의 `/Isaac/Robots/Franka/franka.usd` 에서 재편됐다).
>
> **③ 기본 자산은 튜닝이 안 돼 있다.** `tune_drive.py`(τ) 와 `tune_grasp.py`(마찰)를
> 돌리지 않으면 Phase 1 τ 가 177 ms 이고 Phase 6 은 블록이 미끄러진다. 기동 순서에
> 넣어야 한다.
>
> **④ 6.0 이 예고한 API 이동 셋** (아직 동작하나 경고):
> `ROS2PublishJointState.targetPrim` → `IsaacReadJointState` 연결,
> `ROS2PublishTransformTree.targetPrims` → `OgnIsaacComputeTransform` 연결,
> `frameSkipCount` → `omni:sensor…` 발행 주기.
>
> **⑤ `is_check_phase3.py` 가 낡아 있었다** — `isaac_scene.json` 의 물체 전부를
> 기대해서 정상 동작이 `누락: table` 로 나왔다. `/scene/objects` 가 조작 대상만 싣게
> 된 2026-09-01 결정이 반영 안 된 것이라 검사 쪽을 고쳤다.
>
> **⑦ 시뮬레이터를 재시작하면 ROS 스택도 재시작한다 — 그리고 런치를 죽여도 노드는
> 남는다.** Isaac 을 다시 띄우면 sim 시계가 0 으로 돌아가는데, 살아 있던 스택의 TF
> 버퍼에는 **더 큰 타임스탬프의 옛 데이터**가 남아 새 데이터가 `TF_OLD_DATA` 로
> 버려진다. 증상은 `/scene/objects` 가 **이전 실행에서 물체를 놓았던 자리**를 계속
> 말하는 것이다 — 실측에서 스테이지는 z=0.425 인데 토픽은 z=0.524(직전 Phase 6 가
> 들어올린 높이)를 냈고, Phase 6 접근 목표가 10 cm 어긋나 파지가 실패했다.
> `ros2 run tf2_ros tf2_echo panda_link0 block_a` 로 **TF 를 직접 보면** 갈린다.
>
> 더 고약한 것은 **`ros2 launch` 프로세스를 죽여도 자식 노드가 살아남는다**는 점이다.
> 스택을 다시 띄우면 `isaac_scene_state` 나 `gripper_action_bridge` 가 **둘**이 되고,
> 옛 것이 오염된 버퍼로 계속 발행한다. 액션 서버가 둘이면 검사 로그에
> `There may be more than one action server` 가 나온다. 정리는
> `ros2 node list | sort | uniq -c` 로 중복을 먼저 확인하고, 정리는
> [`scripts/kill_stack.sh`](../../scripts/kill_stack.sh) 로 한다 — 이 네 종류를
> 한 번에 걷어낸다 (`--dry-run` 으로 대상부터 본다).
> 펑션베이의 `/ee_pose` 정지 함정과 **같은 종류**다 (CLAUDE.md).
>
> **⑥ 헤드리스는 실시간보다 빨리 돈다** — 렌더링이 없어 배속 2.2 가 나왔고 Phase 0
> `/clock` 검사가 그것을 잡았다. 물리 dt(1/60)에 맞춰 업데이트를 60 Hz 로 묶으면 0.97 이
> 된다. 검사 자체는 정상 동작한 셈이다.

각 단계는 **이전 단계의 수용 기준을 회귀로 계속 돌린다.** 그리퍼를 붙였다가 팔 관절
순서가 밀리는 종류의 사고를 그 자리에서 잡기 위해서다.

### Phase 0 — 골격 (로봇은 움직이지 않는다)

| | |
|---|---|
| 켜는 것 | 기본값 그대로 (`enable_*` 전부 false) |
| Isaac 쪽 | Panda USD + OmniGraph: `ROS2Context` · `ROS2PublishClock` · `ROS2PublishJointState`(팔 7관절) |
| 우리 쪽 | `panda_isaac.launch.py` — static_tf · RSP · `readiness_gate` → (게이트 통과 후) `move_group` · `rviz` · `ee_pose` |
| 검증 | `scripts/isaac/is_check_phase0.py` |

> ## ✅ Phase 0 완료 — 2026-08-29
>
> `is_check_phase0.py` **5/5 통과**.
>
> | 기준 | 실측 |
> |---|---|
> | `/clock` | 배속 **0.99254** (벽시계 13.85s / sim 13.75s, 샘플 826개) |
> | `/joint_states` | **59.6 Hz**, 관절 **9개**, 이름이 URDF 와 일치 |
> | `use_sim_time` | `/move_group` true · `/ee_pose_publisher` true |
> | TF | `world → panda_hand` 조회 성공 |
> | 계획 | named target `extended` **21 point** 생성 성공 |
>
> **재고 Franka 에셋을 그대로 쓴다** — URDF importer 변환은 필요 없다.
> 도달한 경로: `run_isaac_humble.bat` → Franka 배치 → `setup_graph.py`(PHASE=0) → Play
> → `panda_isaac.launch.py` → `is_check_phase0.py`.

**수용 기준**

1. `/clock` 이 흐르고 **배속이 1.0 근처**다
2. `/joint_states` 가 목표 주파수로 오고 **관절 이름 7개가 URDF 와 정확히 일치**한다
3. `move_group` · `rviz` · `ee_pose` 의 `use_sim_time` 이 **true** 다
4. TF 트리가 `world → panda_link0 → … → panda_hand` 로 성립한다
5. named target `extended` **계획이 성공**한다 (실행은 하지 않는다)

3번을 수용 기준에 넣은 이유: `use_sim_time` 이 한 노드라도 어긋나면 `move_group` 이
`Failed to fetch current robot state` 로 조용히 무력해진다. 증상이 원인을 가리키지
않는 대표적인 함정이라 **첫 단계에서 못을 박는다.**

5번까지 통과하면 **설계 문서가 "Isaac 백엔드의 핵심 계약 항목"으로 꼽은 URDF↔USD
관절 이름·순서 정합이 확인된 것**이다.

> **Phase 0 실측 (2026-08-29)** — `/clock` **59.4 Hz**, `/joint_states` 는 sim time
> 스탬프(`sec: 93`)로 **`panda_joint1~7` + `panda_finger_joint1/2` 9관절**을 이름까지
> 채워 보낸다. **URDF 와 이름이 정확히 일치**하므로 재고 Franka 에셋을 그대로 쓴다
> (URDF importer 변환 불필요). `effort` 도 실려 온다 — joint2 −6.56, joint4 18.69 로
> 중력 토크가 보인다.
>
> **Phase 2 (그리퍼)의 절반이 이미 끝나 있다** — 상태는 오고 있고 명령만 남았다.

### Phase 1 — 팔 관절 명령

| | |
|---|---|
| Isaac 쪽 | `setup_graph.py` 의 `PHASE = 1` — `ROS2SubscribeJointState`(`/isaac/arm_command`) → `IsaacArticulationController` |
| 우리 쪽 | 새 노드 없음. `create_move_group_client(mode='jgpc', arm_command_format='joint_state')` 로 스트리밍 |
| 검증 | `scripts/isaac/is_check_phase1.py` |

> ## ✅ Phase 1 완료 — 2026-08-29
>
> `is_check_phase1.py` **5/5 통과** (drive 튜닝 후).
>
> | 항목 | 기본값 (K=400) | 튜닝 후 (K=10000) |
> |---|---|---|
> | 시정수 τ | 195.6 ms | **34.9 ms** |
> | dead time | 46.9 ms | 51.6 ms |
> | 오버슈트 | — | **0.0%** |
> | 정착 오차 @`ready` | 0.0015 rad | **0.0002 rad** |
> | 정착 오차 @`stretch` | — | **0.0001 rad** |
>
> **모델이 실측과 맞는다.** 기본값에서 `τ = B/K = 80/400 = 200 ms` 예측에 실측
> 195.6 ms (오차 2%), 튜닝 후 `400/10000 = 40 ms` 예측에 실측 34.9 ms.
> 그래서 필요한 gain 을 **역산해서** 정할 수 있었다 — 시행착오가 아니다.
>
> **중력 처짐은 사실상 없다** (0.0001 rad = 0.006°). 펑션베이의 0.0983 rad 과
> 1000배 차이다. 벤더 요청서 A-1 에 해당하는 문제가 이 백엔드에는 없다.

**수용 기준**

| 항목 | 기준 |
|---|---|
| named target 도달 | `extended` 계획 → 스트리밍 → 목표 오차 ≤ 0.005 rad |
| **정착 오차** | 자세 3종(`ready`/중간/펴짐)에서 **자세 의존성이 없을 것** |
| **시정수 τ** | 측정해 기록. 필요하면 drive `stiffness`/`damping` 튜닝 |
| dead time | 측정해 기록 |

**펑션베이의 교훈을 앞당겨 적용하는 지점이다.** 거기서는 처짐과 응답 지연을 §3·§6 에
가서야 쟀고, 원인인 제어기 강성을 우리가 못 고쳐 벤더 요청서까지 갔다. Isaac 은
articulation drive 의 `stiffness`/`damping` 을 **우리가 직접 조정할 수 있으므로**,
같은 값을 Phase 1 수용 기준에 넣어 그 자리에서 닫는다.

### Phase 2 — 그리퍼

| | |
|---|---|
| 켜는 것 | `enable_gripper:=true` |
| 우리 쪽 | `isaac_gripper_bridge` (신규) — `panda_hand_controller/gripper_cmd` **액션 서버**를 열어 `/isaac/gripper_command` 로 바꾼다. `enable_gripper:=true` 로 `GripperNode` 와 함께 뜬다 |

> ## ✅ Phase 2 완료 — 2026-08-29
>
> `is_check_phase2.py` **6/6 통과**.
>
> | 기준 | 실측 |
> |---|---|
> | finger 상태 | `panda_finger_joint1/2` 수신 |
> | 액션 서버 | `/panda_hand_controller/gripper_cmd` 응답 |
> | 열기/닫기 왕복 | 최대 이동 **0.0400 m**, 최종 폭 0.0400 m (목표 0.04) |
> | 두 finger 일치 | 최대 차이 **0.0000 m** |
>
> **검사는 팔을 `ready` 로 보낸 뒤 시작한다.** scene 에 테이블이 생긴 뒤로 자세에 따라
> **손가락 끝이 상판 안에 박혀** 그리퍼가 물리적으로 막힌다 — 실제로 이동 0.0018 m 가
> 나와 "그리퍼 고장"으로 오독했다. `ready` 로 올리자 0.0400 m 가 정확히 나왔다.
> Phase 1 에서 배운 자세 앵커 원칙이 여기에도 그대로 적용된다.
>
> **액션 이름을 그대로 쓴 덕에 상위 경로는 변경이 없다** — `GripperNode` 는
> mock 에서와 똑같이 동작하고, 그 아래가 ros2_control 인지 `isaac_gripper_bridge`
> 인지 모른다.

**수용 기준** — open/close 왕복, `/joint_states` 에 finger 포함, `<mimic>` 처리 확인
(`panda_finger_joint2` 는 URDF mimic 이라 MoveIt 이 채운다), planning scene 조회로
두 finger 값 일치.

> **왜 액션 서버가 필요한가** — 상위 경로(teleop · robot twin · 데이터셋 재생)는
> 모두 `GripperNode` 를 거치고, 그 노드는 `/panda_hand_controller/gripper_cmd`
> **액션**을 부른다. mock 에서는 ros2_control 의 `GripperActionController` 가 그 서버지만
> Isaac 에는 ros2_control 이 없어 **서버가 아예 없다** — 호출이 응답 없이 멈춘다.
> `isaac_gripper_bridge` 가 **같은 이름으로** 서버를 열어 그 자리를 채운다. 그래서
> 상위 경로는 백엔드가 무엇인지 몰라도 된다.
>
> 시간 안에 목표에 닿지 못하면 **실패가 아니라 `stalled`** 로 보고한다 — 물체를 쥐어
> 더 닫히지 않는 상태가 정상 동작이기 때문이다.

### Phase 3 — scene 객체

| | |
|---|---|
| 켜는 것 | `enable_scene:=true` |
| Isaac 쪽 | 물체 pose 를 **표준 타입**(TF 또는 `PoseArray`)으로 발행 |
| 우리 쪽 | `isaac_scene_state_node` — 표준 타입 → `rdfp_msgs/SceneObjects` 변환 |

D2 에 따라 변환은 **우리 쪽에서** 한다. 좌표계·단위·쿼터니언 순서를 맞추는 책임은
전부 이 노드에 있고, 순수 함수는 `robot_control/scene/pose_math.py` 를 재사용해 ROS
없이 테스트한다.

> ## ✅ Phase 3 완료 — 2026-08-29
>
> `is_check_phase3.py` **5/5 통과**.
>
> | 기준 | 실측 |
> |---|---|
> | `/scene/objects` | 물체 **4개**, `frame_id='panda_link0'` |
> | 물체 목록 | 이름·종류·크기가 JSON 정의와 일치 |
> | 좌표 기준 | `panda_link0` 기준으로 변환됨 |
> | **쿼터니언** | `block_b` **xyzw=(0, 0, +0.383, +0.924)** = z축 **45.0°** — 정의와 일치 |
>
> **쿼터니언 검증이 이 단계의 핵심이었다.** Isaac 은 wxyz, ROS 는 xyzw 라 순서를
> 틀려도 norm 은 1 이고 모든 타입 검사를 통과한다. 대칭 회전으로는 드러나지 않으므로
> `block_b` 에 **z축 45°** 를 주고 그 값이 그대로 나오는지로 확인했다
> (`rdfp_msgs/SceneObject.msg` 가 지시하는 방법이다).
>
> **TF 경유 설계가 값을 했다** — 좌표 변환과 쿼터니언 규약을 tf2 가 처리하므로
> 손으로 뒤집는 코드가 아예 없다.

**수용 기준** — Isaac 에서 물체를 옮기면 `/scene/objects` 에 반영, 좌표계·쿼터니언
순서 검증, `mock_scene_state_node` 와 동일한 메시지 계약.

### Phase 4 — 카메라

| | |
|---|---|
| 켜는 것 | 없음 — Isaac 그래프(`PHASE>=4`)가 발행한다. 보기·녹화는 수집 계층의 몫 |
| Isaac 쪽 | RTX 카메라 → `Image` + `CameraInfo` |

> ## ✅ Phase 4 완료 — 2026-08-29
>
> `is_check_phase4.py` **5/5 통과**. 고정 시점(eye-to-hand) 카메라, 640×480 @ 5 Hz.
>
> | 기준 | 실측 |
> |---|---|
> | 주파수 | **4.96 Hz** (목표 5 Hz) |
> | 형식 | **640×480 `rgb8`** |
> | 스탬프 | 61.48 → 73.08 (**sim time 으로 진행**) |
> | camera_info | 640×480, fx=733.0 |
> | VRAM 증가 | 1,707 → **1,884 MiB** (카메라 약 180 MiB) |
>
> **RTX 4060 8GB 에서 문제없이 돌았다.** 문서에 "최소 사양 미달이라 위험"으로 적어
> 뒀던 것은 NVIDIA 의 blanket 최소 사양(RTX 4080 16GB)을 그대로 옮긴 것이었는데,
> 그 기준은 **센서를 많이 쓰는 복잡한 scene**을 전제한다. 카메라 하나를 5 Hz 로 돌리는
> 부담은 VRAM 180 MiB · GPU 47% 였다.
>
> **5 Hz 라는 요구가 두 제약을 동시에 비껴갔다** — VRAM 과 DDS 단편화(60 Hz 였다면
> 1400 B 조각이 초당 4만 개였다). `frameSkipCount=11` 로 **렌더 자체를 건너뛰므로**
> 60 Hz 로 렌더한 뒤 버리는 방식보다 GPU 를 훨씬 덜 쓴다.
>
> **스탬프가 sim time 으로 채워진다** — 펑션베이는 이 값이 0 이라 이미지를 시간축에
> 놓지 못했고(그 문서 §6.5) 데이터셋 적재가 통째로 깨졌다. 이 백엔드에는 그 문제가 없다.

**수용 기준** — 발행 주파수, encoding, **`header.stamp` 이 sim time 으로 채워질 것**,
recorder 가 mp4 를 만들 것, **VRAM 사용량 관측**.

**이 단계는 이 PC(RTX 4060 8GB)에서 가장 위험하다** — §5. Ubuntu 22.04 머신에서
수행하는 것을 전제로 잡는다.

---

### Phase 5 — 파지 지원 (attach / detach) — **철회됨 (2026-09-01)**

> ## ⛔ 이 단계는 되돌렸다
>
> `attach`/`detach` 를 제거하고, 대신 **조작 대상을 planning scene 에 넣지 않는다.**
> (2026-09-01 재축소) `planning_scene_sync` 자체를 삭제했다 — Q8 참조.
>
> **무너진 전제 셋.**
>
> 1. `attach` 는 명시적 `detach` 까지 유지되는데 **물체는 미끄러져 떨어질 수 있다.**
>    떨어지면 planner 의 믿음이 두 군데에서 동시에 틀린다 — 손에는 없는 물체가
>    붙어 있고, 탁자 위의 실제 물체는 보이지 않는다(붙은 물체를 world 동기화에서
>    제외하므로).
> 2. 토픽으로 만든 근거였던 "rosbag 에 남는다"가 **거짓**이었다.
>    `/scene/commands` 는 `recording_topics.list` 에 없다.
> 3. 파지 동작은 애초에 planning scene 을 보지 않는다 — 접근·하강·상승이 전부
>    cartesian 이고 `GetCartesianPath` 의 `avoid_collisions` 기본값은 `False` 다.
>
> `is_check_phase5.py` 는 삭제했다. 아래 내용은 **왜 그렇게 했다가 되돌렸는지**의
> 기록으로 남긴다.


**궁극 목적이 물건 집기이므로 파지 경로를 단계로 세운다.** 카메라(Phase 4)보다 뒤인
이유는 수집을 시작하기 전에만 있으면 되기 때문이고, 앞이 아닌 이유는 카메라가
하드웨어 제약을 먼저 확인해야 하는 항목이기 때문이다.

| | |
|---|---|
| Isaac 쪽 | 없음 |
| 우리 쪽 | `AttachedCollisionObject` attach/detach. **채택하지 않았다** (Q9 철회) |
| 검증 | `scripts/isaac/is_check_phase5.py` |

**왜 필요한가** — Phase 3 에서 물체를 MoveIt 의 장애물로 올렸다(Q8). 그 상태로 물체를
집으면 **손가락과 물체의 접촉이 충돌로 잡혀** 이후 계획이 전부 실패한다. MoveIt 이
이 상황을 위해 제공하는 것이 attach 다 — 물체를 손 링크에 붙이면 로봇의 일부가 되어
손과의 충돌은 무시되고 **테이블과의 충돌은 계속 검사된다.**

**텔레오퍼레이션 수집은 이것 없이도 동작한다** — teleop 은 관절 명령을 직접
스트리밍하므로 계획을 거치지 않는다. 걸리는 것은 MoveIt 으로 계획하는 경로다
(에피소드 리셋, 홈 복귀, 자동 파지 시퀀스).

**이 단계는 물리 파지를 검증하지 않는다.** 여기서 보는 것은 planning scene 의
기하학뿐이고, PhysX 안에서 손가락이 블록을 실제로 붙잡는지는 별개다 — 그것이
Phase 6 다. 처음에는 "물리 파지는 Phase 2 에서 이미 동작한다"고 적었는데 **틀렸다.**
Phase 2 는 자유공간에서 손가락이 움직이는 것만 봤다.

> ## ✅ Phase 5 완료 — 2026-08-29
>
> `is_check_phase5.py` **5/5 통과**.
>
> ```
> 0. 초기 상태  world ['block_a','block_b','block_c','table']
> 1. attach     attached ['block_b'], world ['block_a','block_c','table']
> 2. 유지(4초)  attached ['block_b'], world ['block_a','block_c','table']
> 3. 안전 회귀  attach 중에도 'table' 은 world 장애물
> 4. detach     attached [], world 4개 복귀
> ```
>
> **Q9 결정 — 토픽(`/scene/commands`)으로 명시 호출한다.** 서비스가 아닌 이유는
> **rosbag2 에 남기기 위해서**다. 언제 무엇을 집고 놓았는지가 데이터셋의 사건 표시가
> 된다(`GripperCommand` 를 토픽으로 만든 것과 같은 이유). 기존 `SceneCommand` 타입을
> 그대로 쓰므로 `rdfp_msgs` 변경이 없다.
>
> 모르는 명령은 **조용히 무시한다** — 같은 토픽을 `mock_scene_state_node` 의 `reset`
> 도 듣기 때문에, 실패 결과를 내면 남의 명령을 실패로 보고하게 된다.
>
> **2번이 실제로 잡아낸 것**: 붙은 물체를 주기 동기화(2 Hz)가 그대로 world 로
> 되살리면 손과 영구 충돌이 된다. `_attached` 집합으로 제외한다.

**수용 기준**

| 항목 | 기준 |
|---|---|
| pre-grasp 계획 | 물체 바로 위 자세로 계획 성공 |
| attach 후 이동 | 물체를 든 채 `ready` 로 계획 **성공** (attach 전에는 충돌로 실패) |
| detach | 놓은 뒤 물체가 다시 장애물로 돌아온다 |
| 안전 회귀 | 테이블 충돌은 **계속** 검사된다 — attach 가 모든 검사를 끄면 안 된다 |

### Phase 6 — 물리 파지

**Phase 2 도 Phase 5 도 물체를 실제로 드는 것은 보지 않았다.** Phase 2 는 자유공간의
손가락 이동이고, Phase 5 는 MoveIt planning scene 의 기하학이다. 둘 다 통과한 상태로
집으려 하면 블록은 손가락 사이로 미끄러져 제자리에 남는다.

| | |
|---|---|
| Isaac 쪽 | 손끝·물체 **마찰 재질**, 손가락 **drive 게인** (`tune_grasp.py`) |
| 우리 쪽 | **없음** — 기존 그리퍼 액션과 `/scene/objects` 를 그대로 쓴다 |
| 검증 | `scripts/isaac/is_check_phase6.py` |

**빠져 있던 것이 둘이다.**

*마찰* — 재질을 안 붙이면 PhysX 기본값(정적·동적 모두 0.5)이 쓰인다. 손끝과 블록
양쪽이 0.5 면 매끈한 상자를 옆에서 눌러 드는 데 모자란다. USD 에서 마찰은 prim 의
속성이 아니라 **별도 Material prim** 이고 `physics` purpose 로 바인딩해야 한다 —
안 붙여도 경고가 없으므로 조용히 기본값이 쓰인다.

*손가락 drive* — 손가락은 위치 제어다. 닫힘(0.0)을 명령했는데 블록이 0.025 에서
막으면 **그 오차가 그대로 파지력**이 된다.

    파지력 = stiffness x 오차 = 2000 N/m x 0.025 m = 50 N

0.1 kg 블록을 마찰 1.0 으로 드는 데 필요한 힘은 양쪽 합쳐 0.98 N 뿐이라 50 N 은
충분한 여유다. 더 올리면 접촉 순간에 블록을 튕겨낸다.

**유리하게 맞아떨어지는 것 하나** — `gripper_action_bridge` 는 목표 미도달을 실패가
아니라 `stalled` 로 보고한다. 물체를 문 상태가 정확히 그것이므로 **파지 성공이 액션
실패로 뒤집히지 않는다.** Phase 2 를 위해 그렇게 만든 판정이 여기서 그대로 맞는다.

**`block_a` 를 집는다.** `block_b` 는 z축 45° 회전이라 평행 조가 면이 아니라 모서리를
문다 — 대각 0.0707 m 가 최대 개구 0.08 안에 들어와서 '집히긴 하는' 상태가 되어 판정을
흐린다. 그 45° 는 Phase 3 의 쿼터니언 규약 검증용이므로 건드리지 않는다.

> ## ✅ Phase 6 완료 — 2026-08-30
>
> `is_check_phase6.py` **6/6 통과**.
>
> ```
> 1. 기준 자세  ready 관절오차 0.0003 rad, 그리퍼 0.0400 m, 충돌없음 True
> 2. 접근·하강  목표 (0.500,-0.150,0.528) 오차 0.0005 m, 손 기울기 0.0도
> 3. 물림       폭 0.0225 m (블록 반폭 0.025), stalled=True
> 4. 들어올림   블록 z 0.4255 -> 0.5254  (+0.0999 m, 명령 +0.100)
> 5. 유지(3초)  z 0.5254 -> 0.5254  (낙하 0.0000 m)
> ```
>
> **물림 폭 0.0225 가 반폭 0.025 보다 2.5 mm 작다.** 손가락이 블록을 그만큼 파고든
> 것이며, 접촉 강성이 유한하다는 뜻이다. 파지력은 stiffness x 오차이므로 이 상태가
> 곧 힘이 실린 상태다 — 정확히 0.025 에서 멈췄다면 오히려 힘이 0 이다.
>
> `stalled=True` 가 **정상**이다. `gripper_action_bridge` 가 목표 미도달을 실패가
> 아니라 stalled 로 보고하도록 Phase 2 에서 만들어 둔 판정이 여기서 그대로 맞았다.
>
> **막고 있던 것은 마찰이 아니라 좌표였다.** 처음 실행은 손가락이 0.0000 까지 닫히고
> 블록이 소수점 넷째 자리까지 안 움직였다 — 마찰 부족 또는 콜라이더 부재로 읽히는
> 증상이다. 실제로는 `panda_link0` 이 월드 `(-0.035, -0.349, 0)` 에 있어 손이 블록에서
> **y 축으로 35 cm 떨어진 허공**을 쥐고 있었다(§7 로봇 배치). 그 뒤 시작 자세 함정이
> 이어서 드러났다(§7 시작 자세) — 배치를 고치기 전에는 탁자를 비껴가 숨어 있었다.
>
> `set_home_pose.py` 로그가 그 구조를 보여준다. **드라이브 목표는 원래 `ready` 였고**
> 빠진 것은 `state:angular:physics:position`(PhysX 시작 상태)뿐이었다. 그래서 Play
> 순간 말린 자세에서 강성 10000 으로 `ready` 를 향해 튕겼고, 그것이 '팔이 탁자 위로
> 뚝 떨어진다'로 보였다.
>
> `panda_finger_joint2` 에는 drive 가 없다(mimic). 한쪽 drive 만으로 파지가 성립하는
> 것을 확인했으므로 그대로 둔다.

**수용 기준**

| 항목 | 기준 |
|---|---|
| 접근·하강 | 블록 위 0.10 m → 파지 높이. 도달 오차 ≤ 0.02 m |
| 물림 | 손가락 폭이 **0 이 아니라** 0.015~0.035 m 에서 멈춘다 (블록 반폭 0.025) |
| 들어올림 | 블록 z 가 **+0.07 m 이상** 따라 오른다 (명령 +0.10) |
| 유지 | 3 초 뒤 낙하 ≤ 0.02 m |

물림 기준이 범위인 것이 핵심이다. **0 에 가까우면 놓친 것**이고 열림에 가까우면 닫히지
않은 것인데, 둘 다 "명령은 받았고 손가락은 움직였다"로는 구별되지 않는다.

파지 높이는 `panda_link8` 원점에서 손끝 중심까지 **0.1034 m** 라는 값에 기댄다
(URDF: finger 관절 원점 `panda_hand` 기준 z=0.0584 + 손가락 메시 ~0.045). 이 값이
틀리면 블록을 밀어내거나 헛문다. 손이 아래를 보지 않으면 계산 자체가 무의미하므로
검사가 **기울기를 재서 15° 넘으면 경고**한다.

### Phase 7 — 수집 계층 연동

Phase 0~6 은 제어 계층만 다뤘다. 여기서 그 위에 수집 계층을 얹어 **에피소드가
실제로 기록되는지** 확인한다.

| | |
|---|---|
| Isaac 쪽 | **없음** |
| 우리 쪽 | `rdfp/launch/rdfp_panda_isaac.launch.py` — `panda_isaac` include + 수집 노드 |
| 검증 | rosbag2 기록 → `_detect_all_episodes` 로 분절 확인 |

**카메라 설정의 출처가 다른 백엔드와 다르다.** 다른 launch 는 `image_pipeline.yaml`
을 읽지만 Isaac 은 이미지를 카메라 노드가 아니라 **시뮬레이터가 직접** 발행한다.
해상도·주파수·토픽을 정하는 쪽이 Isaac 의 렌더 프로덕트이므로, `isaac_scene.json` 의
`camera` 블록을 launch 가 그대로 읽어 뷰어·레코더의 기본값으로 쓴다. 해상도를 바꾸려면
JSON 한 곳만 고치고 **`colcon build` + `setup_graph.py` 재실행**을 하면 된다.

**arm 명령 채널은 `joint_state` 다.** Isaac 이 `/isaac/arm_command` 로
`sensor_msgs/JointState` 를 받으므로 `target_joint_cmds_publisher` 의 `source` 가
`joint_state` 이고, 이 경로는 **컨트롤러의 `joints` 파라미터를 조회하지 않는다** —
메시지에 이름이 이미 있기 때문이다. ros2_control 컨트롤러가 없는 Isaac 스택에서
중요한 성질이며, 다른 백엔드였다면 조회가 실패해 `JointState.name` 이 비었을 것이다.

> ## ✅ Phase 7 완료 — 2026-08-30
>
> `rdfp_panda_isaac.launch.py` 로 스택을 띄우고 에피소드 둘을 기록했다.
>
> ```
> 토픽 수신     /isaac/camera/image_raw 4.57 Hz (목표 5)
>               /scene/objects 1.98 Hz (목표 2)
>               /joint_states 59.3 Hz (목표 60)
> 레코더 설정   fps=5  resolution=640x480   (isaac_scene.json 에서 자동)
> use_sim_time  session_control / image_recorder / target_joint_cmds_publisher 모두 True
>
> 기록된 토픽 10개 / split 1개
>   /target_joint_cmds   277개  = /isaac/arm_command 277개  (1:1 변환 확인)
>   /session               8개
>   /isaac/camera/image_raw 1044개
>
> 에피소드 2개
>   1. start 1745.08 s  길이 36.07 s  label=None              (파지 6/6 수행)
>   2. start 1782.65 s  길이  9.47 s  label='isaac_pick_block_a'
> ```
>
> **타임스탬프가 sim time 이다** (1745 초 = Isaac 가동시간). `use_sim_time` 이 수집
> 계층까지 전파됐다는 뜻이며, 벽시계였다면 에피소드 경계가 시뮬레이터 시간과 어긋나
> 재생이 불가능해진다.
>
> **`set_task_label` 은 `IN_EPISODE` 중에 거부된다** (`success=False, 'invalid
> command'`). 에피소드를 시작하기 전에 설정해야 한다 — 위 기록에서 1번 에피소드에
> 라벨이 없는 이유다.
>
> **MP4 레코더는 이 경로에 관여하지 않는다.** `image_recorder` 는 서비스로 켜는
> 별도 경로이고, 데이터셋의 MP4 는 `import` 가 rosbag 에서 만든다. 그래서
> `auto_start:=false` 인 채로도 에피소드는 온전하다.

> ## ✅ DB 적재 확인 — 2026-08-30
>
> **적재 경로는 백엔드와 무관하므로 mock 스택으로 검증했다.** Isaac → rosbag 은 위
> Phase 7 에서 이미 확인했고, 그 뒤 `import` 는 rosbag 만 읽는다. (WSL 재시작으로
> `/tmp` 가 비워져 Isaac bag 이 소실됐고, 재기록보다 이쪽이 확실했다.)
>
> DB 는 **Windows 쪽 PostgreSQL 16.10** 이며 WSL2 에서 `127.0.0.1:5432` 로 닿는다.
>
> ```
> init-db      12개 테이블 생성
> import       에피소드 3개, 경고 0
>              joint_states 4929 / pose_stampeds 2455 / scene_objects 99
>              image_streams 3 / image_frames 456 / mp4 3개
>
>  id  label            dur  strm frames joints poses scene
>   1  mock_db_ingest  33.3     1    298   3237  1614    65
>   2  mock_db_ingest  10.8     1    101   1078   539    21
>   3  mock_db_ingest   7.0     1     57    614   302    13
>
> 무결성    고아 행 0, image_streams.frame_count = 실제 image_frames 수
> MP4       h264 960x540, 프레임 수가 DB 와 일치 (298/101/57)
> 재적재    skipped=3, inserted 전부 0  (on_existing_episode: skip)
> stats/list 두 CLI 모두 같은 값을 보고
> ```
>
> **`target_joint_states` 가 0 행인데 이것은 정상이다.** bag 안 `/target_joint_cmds`
> 자체가 0건이다 — mock 의 JTC 경로는 MoveIt **액션**으로 실행하므로
> `/panda_arm_controller/joint_trajectory` 토픽에 아무것도 흐르지 않고,
> `target_joint_cmds_publisher` 는 변환할 입력이 없다.
>
> **Isaac 은 다르다.** 명령 스트리밍이라 토픽에 실제로 흐르고, Phase 7 에서
> `/target_joint_cmds` 277건 = `/isaac/arm_command` 277건으로 1:1 확인했다. 즉
> **action 채널이 채워지는지는 백엔드가 결정한다** — MoveIt 액션으로 움직인 구간은
> 데이터셋에 명령값이 남지 않는다.

### Phase 8 — 테스트와 토폴로지 계약

Phase 0~7 의 검증은 전부 **수동 체크 스크립트**였다. 사람이 Isaac 을 띄우고 Play 를
눌러야 돌아가므로 `colcon test` 에는 잡히지 않는다. 여기서 그 아래를 자동 테스트로
받친다.

| | |
|---|---|
| 대상 | 새로 만든 노드 3종 + Isaac 쪽 스크립트 9개의 경로 계약 |
| 검증 | `colcon test --packages-select robot_control` (Isaac 불필요) |

> ## ✅ Phase 8 완료 — 2026-08-30
>
> `robot_control` **146 → 226 테스트** (+80).
>
> | 새 스위트 | 수 | 무엇을 지키나 |
> |---|--:|---|
> | `isaac/tests/test_scene_state_node.py` | 10 | TF→SceneObject, 쿼터니언 무변환, 누락 물체 |
> | `isaac/tests/test_gripper_action_bridge_node.py` | 9 | **stalled 판정**, 목표 재발행 |
> | `tests/test_isaac_sim_side_scripts.py` | 47 | 배포 구성 A~D 의 경로 해석 |
>
> **테스트가 실제 버그를 하나 잡았다.** 두 노드의 경고 스로틀이
> `now - _logged.get(key, 0.0) >= interval` 형태라 **`now < interval` 인 동안 첫
> 경고를 삼킨다.** Isaac 은 Stop 마다 sim time 이 0 으로 되돌아가므로 기동 직후
> 5 초가 정확히 그 구간이고, 하필 "TF 가 안 온다"를 가장 보고 싶은 때다. 기본값을
> `-inf` 로 바꿔 첫 번째는 반드시 남게 했다 (`isaac_scene_state_node`,
> `isaac/scene_state_node`).
>
> **대역을 만들다 배운 것 둘.**
>
> `_execute` 의 루프 조건이 `rclpy.ok()` 인데, `rclpy.init()` 을 부르지 않은
> 프로세스에서는 False 다. 그러면 루프가 한 번도 안 돌아 **어떤 목표를 줘도
> `reached_goal=False`** 가 나오고, 테스트는 통과하는 것처럼 보이면서 아무것도
> 검증하지 않는다. DDS 를 띄우지 않으려고 그 지점만 대체한다.
>
> `os.name` 을 전역으로 바꾸면 안 된다. `pathlib` 이 그 값으로 구현을 고르므로
> `Path()` 가 `WindowsPath` 를 만들려다 예외를 내고, 그 예외가 pytest 리포터 안에서
> 터져 **실행 전체가 INTERNALERROR 로 죽는다.** `sys.modules['os']` 에 대역을 잠깐
> 끼우면 스크립트만 영향을 받는다.

**토폴로지는 A 만 실측했다.** B~D 는 다른 머신이 있어야 확인할 수 있고, 테스트가 보는
것은 *경로가 올바로 갈라지는지*뿐이다. 그것만으로도 값이 있는 이유는 아홉 개 스크립트에
같은 블록이 복사되어 있어 **하나만 고치고 나머지를 빠뜨리기 쉽기** 때문이다. 공용
헬퍼로 뽑을 수 없는 것은 Isaac 쪽에서 `robot_control` 을 import 할 수 없어서다.

테스트가 함께 고정하는 것 둘 — 모든 스크립트가 **로그를 파일로도 남기고**(출력창을
복사할 수 없다), **WSL 배포판 이름이 하나로 통일**되어 있을 것(`Ubuntu` 와
`Ubuntu-22.04` 를 혼동해 파일을 못 연 적이 있다).

### Phase 9 — robot_twin 연동

트윈은 제어 계층을 REST 로 노출하는 게이트웨이다. Isaac 스택을 그 뒤에 붙인다.

| | |
|---|---|
| Isaac 쪽 | 없음 |
| 우리 쪽 | `moveit.arm_command_*` 설정 3종, `config/robot_twin_panda_isaac.yaml` |
| 검증 | 트윈을 띄우고 REST 로 팔·그리퍼·세션을 실제로 움직인다 |

**명령 채널을 설정으로 뚫는 것이 핵심이다.** `runtime.py` 가
`create_move_group_client(node, mode=mode)` 만 부르고 있어 Isaac 의
`/isaac/arm_command`(JointState) 대신 `/panda_arm_controller/commands`
(Float64MultiArray)로 나갔다. **토픽 remap 으로는 못 고친다 — 메시지 타입이 다르다.**

설정에 짝 검사를 넣었다. `jtc` 에 이 키들을 주면 거부하고(그 모드는 보지 않으므로
조용히 무시되는 키가 생긴다), `joint_state` 형식에 `arm_command_joint_names` 가
없으면 거부한다(조회할 컨트롤러가 없어 첫 스트리밍에서 멈춘다).

> ## ✅ Phase 9 완료 — 2026-08-30
>
> `robot_twin_panda_isaac.yaml` 로 트윈을 띄워 REST 로 실제 제어를 확인했다.
>
> ```
> /health      move_group=READY  mode=jgpc
>              arm_command={topic:/isaac/arm_command, format:joint_state, joints:[7]}
> 변수         joint_states / ee_pose / scene_objects / session_state 모두 OK
>              stamp 이 sim time (sec 3301)
> move_to_joints          panda_joint1  0.000 -> +0.400  (나머지 6축 유지)
> move_to_named_target    ready 복귀 COMPLETED
> move_gripper_to_target  close 0.0017 / open 0.0383  (reached_goal=true)
> 세션 4종     start_session -> start_episode -> stop_episode -> stop_session 전부 COMPLETED
> ```
>
> **여기서 제어 계층의 버그를 하나 찾았다.** `plan_joints_async` 가
> `externally_spun` 을 **받지도 넘기지도 않아서**, executor 가 다른 스레드에서 노드를
> 돌리는 트윈에서는 `_complete_joint_values` 가 노드를 직접 spin 하려다 영원히
> 멈췄다. 콜백은 executor 쪽으로 가므로 여기서는 영영 오지 않는다 — **예외도
> 타임아웃도 없이 오퍼레이션이 RUNNING 인 채로 남는다.** 인자를 추가하고
> `move_to_joints_streamed_async` 에서 전달하게 고쳤다
> (`moveit/tests/test_externally_spun_propagation.py` 5건).
>
> `plan_named_target_async` 는 같은 문제가 없다 — 관절값을 콜백 체인으로 얻으므로
> 애초에 spin 하지 않는다. 그래서 `move_to_named_target` 만 먼저 동작해 원인이
> 더 헷갈렸다.
>
> **트윈은 시작 자세 충돌에서 스스로 빠져나올 수 없다.** 모든 팔 오퍼레이션이 MoveIt
> 계획을 거치는데, 시작 자세가 충돌이면 계획 자체가 거부된다(§7 시작 자세). 실제로
> 첫 시도가 그렇게 막혔다(손가락이 `block_a` 를 물고 있었다). 계획을 거치지 않는
> 복구 경로가 필요해 `scripts/isaac/is_recover.py` 를 만들었다.
>
> **`reset_scene` 은 노출하지 않는다.** scene 물체가 USD 스테이지에 있어 고치려면
> 시뮬레이터 안에서 스테이지를 써야 한다 — ROS 쪽 노드는 원리적으로 할 수 없고,
> `isaac_scene_state_node` 가 `/scene/reset` 서비스를 열지조차 않는 이유다.
> 남겨 두면 결과 토픽을 기다리다 타임아웃할 뿐 구독자가 없다는 단서는 안 남는다.
>
> ## ✅ 해소됨 — 2026-09-02
>
> **전제가 바뀌었다.** Isaac 6.0 의 `isaacsim.ros2.sim_control` 확장이 표준
> `simulation_interfaces` 서비스(`/set_entity_state` 등 19개)를 연다. ROS 쪽 노드가
> 스테이지를 직접 못 쓰는 것은 여전히 맞지만, **Isaac 에게 부탁할 창구가 생겼다.**
> `isaac_scene_state_node` 가 `/scene/reset` 을 열고 물체마다 `/set_entity_state` 를
> 부른다 — 시뮬레이터 쪽에 상주 스크립트를 두지 않는다.
> 상세는 [../scene/isaac_scene_reset.md](../scene/isaac_scene_reset.md).
>
> `gripper_last_command_result` / `gripper_position` 은 `NO_DATA` 다. QoS 는
> 일치하므로 불일치가 아니고, mock 설정에서도 같다 — Isaac 과 무관한 별개 항목이다.

**곁가지로 드러난 것 — robot_twin 테스트가 통째로 안 돌고 있었다.** `test_api.py` 의
모듈 최상단 `pytest.importorskip('httpx')` 가 이 조합(pytest 6.2.5 + 패키지 형태의
tests 디렉터리)에서 **디렉터리 수집 자체를 끝내버려서**, 뒤따르는 열한 개 파일
190여 개가 사라졌다. 결과가 `1 skipped` 한 줄뿐이라 **없어진 것을 아무도 눈치채지
못한다.** import 를 직접 감싸고 `pytestmark` 로 건너뛰게 고쳤다.

### Phase 10 — servo(twist) 경로

**두 텔레오퍼레이션 경로가 모두 servo 로 수렴한다.** 그래서 이 다리가 없으면 Isaac 을
손으로 몰 수 없고, 손으로 못 몰면 파지 에피소드를 모을 수 없다 — 이 프로젝트의 목적이
바로 그것이므로 Q3 은 "있으면 좋은 것"이 아니었다.

```
teleop_keyboard  -> delta_twist_cmds ─┐
teleop_retarget  -> ee_twist_node ────┴─> servo -> [다리] -> /isaac/arm_command
```

| | |
|---|---|
| Isaac 쪽 | **없음** |
| 우리 쪽 | `isaac/servo_command_bridge_node.py`, `panda_isaac.launch.py` 의 `enable_servo` |
| 검증 | `isaac/tests/test_servo_command_bridge_node.py` (14건) |

**타입이 맞지 않아 remap 으로는 못 잇는다.** servo 의 `command_out_type` 은
`trajectory_msgs/JointTrajectory` 아니면 `std_msgs/Float64MultiArray` 둘뿐이고,
Isaac 의 ROS2 브리지는 `ROS2SubscribeJointState` 만 제공한다. 양쪽 어느 쪽도 상대
타입을 낼 수 없으므로 변환 노드가 하나 필요하다.

`Float64MultiArray` 를 고른 이유는 JGPC mock 과 같다. `JointTrajectory` 를 쓰면
컨트롤러가 없는 스택에서 `/panda_arm_controller/joint_trajectory` 라는 이름을 쓰게
되어 **없는 것을 있는 것처럼** 보이게 만든다. 중간 토픽은 `/isaac/servo_command` 다.

> ## 🔶 Phase 10 구현 완료 · **실기 미검증** — 2026-08-30
>
> `enable_servo:=true` 로 세 노드가 함께 뜬다 — `servo_node` ·
> `servo_auto_start` · `servo_command_bridge`. launch 정적 검사로 확인했다.
>
> ```
> moveit_servo   / servo_node_main
> robot_control  / servo_auto_start_node
> robot_control  / isaac_servo_bridge
>                    remap commands    -> /isaac/servo_command
>                    remap arm_command -> /isaac/arm_command
> enable_servo=false -> 기동 0/3      true -> 기동 3/3
> ```
>
> **`publish_joint_velocities: false` 는 선택이 아니다.** `command_out_type` 이
> Float64MultiArray 인데 positions 와 velocities 를 모두 발행하도록 두면 servo 의
> 파라미터 검증이 실패해 **노드가 아예 기동하지 못한다.** 증상은 "팔이 안 움직인다"
> 하나뿐이라 원인이 보이지 않는다. 오버라이드를 `override_servo_params_for_isaac()`
> 로 빼서 회귀 검사를 걸었다.
>
> **길이 불일치는 자르지 않고 버린다.** `Float64MultiArray` 에는 이름이 없고 배열
> 순서가 곧 관절 순서다. 잘라 쓰면 순서가 밀린 채 팔이 움직이는데, 크래시가 아니라
> '그럴듯하게 틀린 자세'로 나타난다. 조회할 컨트롤러가 없으므로 `joint_names` 는
> **필수 파라미터**이며, 비면 기동 단계에서 실패한다.
>
> **실기 확인 (2026-08-30, Isaac Play 중)** — twist 가 Isaac 까지 도달한다.
>
> ```
> /servo_node/delta_twist_cmds (+z)
>   -> /isaac/servo_command   132건 (Float64MultiArray, 길이 7)
>   -> /isaac/arm_command     132건 (JointState, panda_joint1~7)   ← 1:1, 손실 없음
>   -> EE z  0.5901 -> 0.6097  (+0.0196 m)
> ```
>
> **입력은 m/s 가 아니다.** `command_in_type: unitless` 이고 `scale.linear: 0.4`,
> `publish_period: 0.034` 이다. 즉 `linear.z = 0.05` 는 0.02 m/s 를 뜻한다 —
> 처음에 m/s 로 읽어 "명령보다 훨씬 덜 움직인다"고 오독했다.

**scene 을 켜도 정상이다.** 매 시행 전에 `ready` 로 되돌리고 planning scene 내용만 바꿔
비교하면 `diff 없음 / 빈 diff / table / table+블록3` 넷이 모두 `NO_WARNING` 이고
이동량도 같다(0.18~0.20 m). 처음에는 scene 을 원인으로 지목했는데 **틀렸다** — 재기동
과정에 자세 초기화가 섞여 든 것이었다(§7 servo 드리프트).

| 항목 | 실측 | 상태 |
|---|---|---|
| 2초 구간 추종 배율 | 기대 대비 0.46~0.49 (scene on/off 무관) | 필터 램프업 + 개루프 지연이 후보. **분리 못 했다** |
| 반복 주행 시 정지 | 누적 드리프트 | §7 servo 드리프트. 세션 사이 `is_recover.py` |
| NaN 출력 | 7축 전부 NaN, status 는 `NO_WARNING` | **원인 규명·대응 완료** — §7 servo NaN |
| `ready` 에서 `DECEL_COLLISION` | 팔이 탁자와 실제 충돌한 뒤 발생 | **미규명.** `/check_state_validity` 는 충돌 없음으로 본다 |
| `teleop_keyboard` 실주행 | — | **미검증** (지금까지는 raw twist 로만 확인) |


## 3. 골격 구성

### 파일

```
── ROS 쪽 (WSL2 / Ubuntu 스택) ────────────────────────────────────────────
src/robot_control/launch/panda_isaac.launch.py            백엔드 launch (Phase 0~5)
src/robot_control/robot_control/isaac/
    gripper_action_bridge_node.py                         액션 → 관절 토픽 (Phase 2)
    scene_state_node.py                                   Isaac TF → /scene/objects (Phase 3)
src/robot_control/robot_control/scene/
src/robot_control/config/
    isaac_scene.json                                      **물체·카메라 단일 진실원본**
    fastdds_wsl_bridge.xml                                DDS 전송 프로파일 (구성 A 전용)
    panda_real_joint_limits.yaml                          실제 Franka 관절 한계 (Q6)

── 검증 스크립트 (ROS 쪽) ──────────────────────────────────────────────────
scripts/isaac/is_check_phase0.py ~ is_check_phase6.py     단계별 수용 기준 (phase5 는 철회)
scripts/isaac/is_recover.py                               충돌 자세 탈출 (계획 없이 ready 복귀)
scripts/isaac/is_topics.py                                배관 진단 (자체시험 + 수신량)
scripts/isaac/is_probe_srdf.py                            파라미터 서비스 2×2 진단

── Isaac 쪽 (Script Editor 에서 실행) ──────────────────────────────────────
scripts/isaac/sim_side/load_robot.py                      Franka 를 스테이지에 올린다 (멱등)
scripts/isaac/sim_side/setup_graph.py                     OmniGraph 구성 (단계 누적)
scripts/isaac/sim_side/setup_scene.py                     물체·카메라 생성 / 리셋
scripts/isaac/sim_side/tune_drive.py                      팔 drive 게인 (멱등)
scripts/isaac/sim_side/tune_grasp.py                      손가락 drive·손끝 마찰 (멱등)
scripts/isaac/sim_side/place_robot.py                     panda_link0 을 월드 원점으로 (멱등)
scripts/isaac/sim_side/set_home_pose.py                   Play 시작 자세를 ready 로 (멱등)
scripts/isaac/sim_side/check_colliders.py                 충돌·배치 진단 (읽기 전용)
scripts/isaac/sim_side/verify_saved_scene.py              저장된 scene 검증
scripts/isaac/sim_side/check_bridge.py                    Isaac 내부 토픽 가시성 진단
scripts/isaac/sim_side/run_isaac_humble.bat               Windows 런처 (humble·프로파일 고정)

docs/simulation/isaac_backend_skeleton.md                 이 문서
```

`sim_side/` 아래만 **Isaac Sim 안에서** 돈다 — ROS 노드가 아니고, `colcon` 이 빌드하지도
않는다. `.bat` 은 Windows 전용이며 `C:\isaacsim\` 에 **복사본**이 있다(원본을 고치면
복사해야 한다).


**Phase 0 에서 새로 만드는 Python 모듈은 없다.** 브리지 노드를 펑션베이 것으로
그대로 쓰기 때문이다 (§4). `config/isaac_panda.yaml` 도 만들지 않는다 — 살아 있는
소비자가 있는 키만 YAML 에 둔다는 원칙에 따라, launch 인자 기본값으로 충분한 동안은
파일을 만들지 않는다.

### 토픽 계약

| 방향 | 토픽 | 타입 | 단계 |
|---|---|---|:-:|
| Isaac → 스택 | `/clock` | `rosgraph_msgs/Clock` | 0 |
| Isaac → 스택 | `/joint_states` | `sensor_msgs/JointState` | 0 |
| 스택 → Isaac | `/isaac/arm_command` | `sensor_msgs/JointState` | 1 |
| 스택 → Isaac | `/isaac/gripper_command` | `sensor_msgs/JointState` | 2 |
| Isaac → 스택 | `/isaac/scene_poses` | 표준 타입 (TF or `PoseArray`) | 3 |
| Isaac → 스택 | `/isaac/camera/image_raw` · `camera_info` | `sensor_msgs/Image` · `CameraInfo` | 4 |

`/isaac/` 접두사는 **시뮬레이터 경계임을 이름으로 드러내기 위한 것**이다. 스택 내부
토픽(`/joint_states`, `/scene/objects`)과 섞이면 어느 쪽이 원본인지 로그만 보고는
알 수 없다. 펑션베이의 `/input`·`/output` 과 같은 의도다.

### 기동 순서

```
static_tf · robot_state_publisher · readiness_gate
    └ readiness_gate 가 첫 /joint_states 를 받고 **정상 종료**
         └ move_group · rviz · ee_pose
```

**펑션베이와 달리 `joint_state_fusion` 이 없다** — Q1 이 Phase 0 실측으로 닫혔다(§8).

spawner 가 없는 백엔드이므로 게이트 노드의 **종료 코드**를 기동 신호로 쓴다
(`_chain_or_shutdown`). 실패 종료 시 launch 전체가 내려간다 — 시뮬레이터가 꺼져
있는데 `move_group` 만 올라와 원인을 감추는 상황을 막는다.

---

## 4. 재사용하는 것

펑션베이에서 만든 두 노드가 **파라미터화돼 있어 코드 수정 없이 그대로 쓰인다.**

| 실행 파일 | Isaac 에서의 역할 | 넘기는 파라미터 |
|---|---|---|
| `fb_readiness_gate` | 기동 게이트 | `topic` · `timeout_sec` |
| ~~`fb_joint_state_fusion`~~ | **쓰지 않는다** — Isaac 이 이름과 9관절을 모두 채운다 (§8 Q1) | — |

특히 `extra_joint_names` / `extra_joint_positions` 는 **"그리퍼 미연동 구간 동안
finger 를 고정값으로 채워 TF 만 성립시키는"** 용도로 이미 만들어져 있다 — Phase 1 과
Phase 2 사이를 잇는 비계가 이미 있는 셈이다.

> 실행 파일 이름의 `fb_` 접두사는 펑션베이 전용이라는 인상을 주지만 구현은 백엔드
> 중립이다. 세 번째 소비자가 생기면 `sim_readiness_gate` 등으로 공용화한다. **지금
> 이름을 바꾸지 않는 이유**는 펑션베이 작업이 중단 상태라 회귀를 확인할 사람이
> 없기 때문이다.

`fb_hold.py` / `fb_probe.py` / `fb_recover.py` 도 토픽 상수만 바꿔 `scripts/isaac/`
로 이식한다 (Phase 1).

---

## 5. 하드웨어 제약 — 이 PC (WSL2)

| | 실측 | Isaac Sim 5.1 최소 사양 |
|---|---|---|
| RAM (호스트) | 32 GB | **32 GB** |
| RAM (WSL2 몫) | **15.4 GB** (기본 50%, `.wslconfig` 에 `memory=` 없음) | — |
| GPU | **RTX 4060 / VRAM 8 GB** | **RTX 4080 / VRAM 16 GB** |

**WSL2 를 그대로 두면 Isaac 이 최소 사양의 절반으로 돈다.** WSL2 는 한 번 잡은
메모리를 자동 반환하지 않으므로(`autoMemoryReclaim` 미설정) 캡을 거는 것이 맞다.

```ini
# C:\Users\<user>\.wslconfig
[wsl2]
memory=10GB              # MoveIt+RViz+recorder+postgres 에 충분
processors=12            # 28개 전부 주면 Isaac 물리 스텝과 코어를 다툰다
swap=8GB
networkingMode=mirrored  # 유지 — Isaac↔WSL2 DDS 에 유리
dhcp=true
dnsTunneling=true

[experimental]
hostAddressLoopback=true
autoMemoryReclaim=gradual   # ★ 추가
```

**더 빡빡한 쪽은 RAM 이 아니라 VRAM 이다.** 8 GB 는 최소 미달이라 Panda 한 대짜리
단순 scene 은 돌아가도 **RTX 카메라 센서를 붙이는 순간이 위험 구간**이다. WSL2 에서
RViz 를 띄우면 WSLg 가 같은 8 GB 를 나눠 쓰므로, 이 PC 에서는 `enable_rviz:=false`
를 기본 운용으로 삼는다.

→ **실측으로 뒤집혔다.** 640×480 @ 5 Hz 카메라 하나는 VRAM 180 MiB · GPU 47% 로
이 PC 에서 문제없이 돌았다(Phase 4 완료). 최소 사양 미달은 사실이지만, 그 기준은
센서가 많은 복잡한 scene 을 전제한다. **고해상도·고주파가 필요해지면 그때 Ubuntu 머신으로
옮긴다.**

---

## 6. 운용 절차


> **새 Windows 머신에 처음 구축하는 경우**는 [isaac_windows_setup.md](isaac_windows_setup.md)
> 를 따른다. WSL2 설치부터 검증까지의 절차만 추린 문서이며, 아래는 그중 Isaac 쪽
> 운용에 해당한다.

**클릭으로 만들지 않는다.** `scripts/isaac/sim_side/setup_graph.py` 가
OmniGraph 를 구성한다 — UI 절차는 버전마다 바뀌고 재현이 안 되지만 스크립트는
저장소에 남는다.

### 절차

1. **Isaac 을 `run_isaac_humble.bat` 으로 띄운다.**

   ```
   C:\isaacsim\run_isaac_humble.bat
   ```

   원본은 `scripts/isaac/sim_side/run_isaac_humble.bat` 이고 `C:\isaacsim\` 에
   복사해 둔다(UNC 경로에서는 cmd 가 작업 디렉터리를 잡지 못한다).

   **왜 필요한가** — 기본 런처 `isaac-sim.bat` 은 `setup_ros_env.bat` 을 호출하고,
   거기에 이렇게 박혀 있다.

   ```bat
   set DEFAULT_ROS_DISTRO=jazzy
   if "%ROS_DISTRO%"=="" (
       set ROS_DISTRO=%DEFAULT_ROS_DISTRO%
       set "PATH=%PATH%;%BRIDGE_EXT_PATH%\%DEFAULT_ROS_DISTRO%\lib"
   )
   ```

   즉 Windows 기본은 **jazzy** 이고, Humble 인 우리 스택과 서로 보이지 않는다.

   **함정** — `ROS_DISTRO=humble` 만 미리 넣고 띄우면 **더 나빠진다.** 위 `if` 블록이
   통째로 건너뛰어져 `humble\lib` 이 PATH 에 붙지 않고, Isaac 은 "사용자가 ROS 를
   source 했다"고 판단해(`internal_lib_fallback=False`) 시스템 ROS 2 DLL 을 찾다가
   **크래시한다.** `--/exts/isaacsim.ros2.bridge/ros_distro=humble` 플래그도 소용없다 —
   그 설정은 `ROS_DISTRO` 환경변수가 **없을 때만** 읽힌다.

   런처는 벤더 스크립트가 jazzy 에 하던 일(환경변수 + PATH)을 humble 로 재현한다.
   벤더 파일은 고치지 않는다 — 재설치·업데이트로 되돌아간다.

   확인은 kit 로그에서 한다.

   ```bash
   grep -h "internal rclpy for ROS Distro\|Failed to load system ROS 2" \
     "/mnt/c/Users/$USER/.nvidia-omniverse/logs/Kit/Isaac-Sim Full/6.0/"*.log | tail -2
   ```

2. Isaac UI 에서 **Franka(Panda) 를 스테이지에 올린다** (asset browser).
   스크립트가 로봇을 불러오지 않는 이유는 에셋 경로가 Isaac 버전마다 바뀌기
   때문이다 — 대신 스테이지의 **articulation root 를 자동 탐색**한다.

   > **드래그해 올린 로봇은 원점에 떨어지지 않는다.** ROS 쪽 static TF 가
   > `world -> panda_link0` 를 항등변환으로 못박고 있으므로 반드시 맞춰야 한다.
   > 다음 단계의 `place_robot.py` 가 그 일을 한다 — 안 맞추면 **에러 없이**
   > 모든 좌표가 그만큼 어긋난다(§7 로봇 배치).

3. `Window > Script Editor` 에서 **붙여넣지 말고 파일을 불러 실행한다.**

   ```python
   path = "//wsl.localhost/Ubuntu-22.04/home/kwlee/development/ros/rdfp_ws"
   exec(open(path + "/scripts/isaac/sim_side/setup_graph.py", encoding="utf-8").read())
   ```

   붙여넣기를 피하는 이유는 둘이다 — 인코딩을 타고(Windows 기본 cp949 로 읽으면
   한글 주석이 깨진다), 사본이 저장소보다 낡는다. 경로 구분자는 슬래시를 쓴다
   (Windows 의 `open()` 이 UNC 경로에서도 받는다). 배포판 이름이 `Ubuntu` 가 아니라
   **`Ubuntu-22.04`** 인 점에 주의한다 — `wsl -l -q` 로 확인한다.

4. 실행 결과를 **WSL 에서 읽는다.** Script Editor 출력창은 복사가 되지 않으므로
   스크립트가 같은 내용을 파일로도 남긴다.

   ```bash
   cat /tmp/isaac_setup_graph.log
   ```

   로그에는 kit 빌드 버전, 스테이지의 articulation 목록, 그리고 **SET_VALUES 를
   되읽은 값**(`domain_id` · 두 `topicName`)이 들어 있다. 되읽기를 넣은 이유는
   `og.Controller.edit` 이 속성 이름을 틀려도 조용히 넘어가는 경우가 있어, "설정한
   줄 알았는데 기본값"인 상태를 눈으로 확인하기 위해서다.

5. **로봇을 월드 원점에 맞춘다** (Stop 상태에서).

   ```python
   exec(open(path + "/scripts/isaac/sim_side/place_robot.py", encoding="utf-8").read())
   ```

   `panda_link0` 이 월드 원점에 오도록 articulation root 를 옮기고 **되읽어
   확인한다.** 멱등이라 이미 원점이면 아무것도 하지 않는다. scene 을 저장해 두면
   다음부터는 건너뛰어도 되지만, 에셋을 다시 올렸다면 반드시 돌린다.

6. **시작 자세를 못박는다** (Stop 상태에서).

   ```python
   exec(open(path + "/scripts/isaac/sim_side/set_home_pose.py", encoding="utf-8").read())
   ```

   Play 를 누르는 순간의 관절값을 `ready` 로 만든다. 안 하면 에셋 기본 자세
   (손목이 말린 자세)로 시작해 **손이 탁자 안에 들어간 채** 출발하고, 그 상태에서는
   MoveIt 이 계획 자체를 거부한다(§7 시작 자세).

7. **팔 관절 drive 를 튜닝한다** (Phase 1 이상, Stop 상태에서).

   ```python
   exec(open(path + "/scripts/isaac/sim_side/tune_drive.py", encoding="utf-8").read())
   ```

   에셋 기본값 `stiffness=400 / damping=80` 은 τ 195.6 ms 로 느리다. 스크립트가
   `10000 / 400` 을 **절대값으로** 설정한다(멱등 — 여러 번 돌려도 같은 상태).
   `DRY_RUN = True` 로 바꾸면 현재 값만 읽는다.

   > **이 값은 스테이지에 있다.** USD 를 저장하지 않으면 scene 을 다시 열 때 기본값으로
   > 돌아간다. 저장하거나, 열 때마다 이 스크립트를 다시 돌린다.

8. **파지 준비를 한다** (Phase 6 이상, Stop 상태에서).

   ```python
   exec(open(path + "/scripts/isaac/sim_side/tune_grasp.py", encoding="utf-8").read())
   ```

   손가락 drive 게인과 **손끝 마찰 재질**을 설정한다. 목표값은 `isaac_scene.json` 의
   `gripper` 블록이며, scene 물체의 마찰과 같은 파일에 있다 — 둘은 접촉면의 양쪽이라
   따로 두면 어긋난다. `tune_drive.py` 와 마찬가지로 절대값이라 멱등이다.

   `setup_scene.py` 를 다시 돌린 뒤에도 **이 스크립트는 다시 돌릴 필요가 없다** —
   로봇 에셋을 만지므로 scene 재생성과 무관하다. 그래서 손끝 재질을 scene root
   (`/World/Scene`) 가 아니라 `/World/GraspMaterials` 에 둔다. scene 안에 두면
   `setup_scene.py` 가 root 를 통째로 지울 때 **바인딩이 끊어진 재질을 가리키게**
   된다.

9. **Play(▶) 를 누른다.** `OnPlaybackTick` 은 재생 중에만 tick 을 내므로 정지
   상태에서는 토픽이 하나도 보이지 않는 것이 정상이다.

### scene 을 저장해도 **`setup_graph.py` 는 매번 돌려야 한다**

저장된 USD 에 그래프 prim 은 보존되지만 **ROS 2 bridge 확장의 활성화 상태는 저장되지
않는다.** scene 만 열면 그래프 prim 은 있는데 노드 타입이 해석되지 않아 발행이 하나도
없고, kit 로그에 `internal rclpy for ROS Distro` 줄조차 나오지 않는다. 확장을 켜는
것이 `setup_graph.py` 의 첫 동작이기 때문이다.

그래서 scene 을 여는 절차는 **항상** 이렇게 고정한다.

```
run_isaac_humble.bat → scene 열기 → setup_graph.py → Stop → Play
```

**`setup_scene.py` 는 물체가 저장된 scene 에서는 생략할 수 있다.** prim·물리 API·초기
pose 는 전부 USD 에 담기기 때문이다. 다만 아래 두 경우에는 다시 돌린다.

- **물체를 초기 위치로 되돌릴 때** — 에피소드를 반복할 때 쓰는 리셋 수단이다
  (`root_prim` 을 통째로 지우고 다시 만든다).
- **`isaac_scene.json` 을 고쳤을 때** — 물체를 추가하거나 크기를 바꾸면 scene 과 정의가
  어긋난다. 이때는 `setup_graph.py` 도 함께 돌려야 한다 (`targetPrims` 가 바뀐다).

> **저장은 Stop 상태에서 한다.** 재생 중에는 동적 물체가 움직인 자리가 그대로 굳을
> 수 있다. Stop 하면 Isaac 이 시뮬레이션 이전 상태로 되돌리므로 그때 저장한다.

### 정적 물체도 kinematic rigid body 로 만든다

collider 만 있는 prim 은 PhysX 에서 제자리에 고정되지만 **TF 발행 노드가 물리 객체로
인식하지 못한다** — 매 틱마다 아래를 60 Hz 로 쏟아낸다.

```
[PoseTree] target getObjectType eInvalid for '/World/Scene/table'
```

값이 틀리지는 않지만 kit 로그가 부풀어 **진짜 경고가 묻힌다.** `RigidBodyAPI` 를
붙이고 `kinematicEnabled=True` 로 두면 중력·충돌에 밀리지 않으면서 물리 객체로는
온전해져 경고가 사라진다 (`setup_scene.py` 가 그렇게 만든다).

### 만들어지는 그래프

```
OnPlaybackTick ─┬─> ROS2PublishClock       → /clock
                └─> ROS2PublishJointState  → /isaac/joint_states
IsaacReadSimulationTime ─> 두 노드의 timeStamp
ROS2Context(domain_id=31) ─> 두 노드의 context
```

### 출력은 영어, 주석은 한국어

`sim_side/` 스크립트의 `print` 와 예외 메시지는 **영어**다. Windows 콘솔이 cp949 라
한글이 깨지기 때문이며, CLAUDE.md 의 "로깅·예외 메시지는 영어" 규약과도 같다.
주석과 docstring 은 규약대로 한국어로 둔다 — 위 실행 방식에서는 편집기에 뜨지 않는다.

WSL 에서 도는 `is_check_phase0.py` 는 리눅스 터미널이라 한글이 정상 출력되므로
`scripts/functionbay/fb_*.py` 와 같은 한국어 리포트를 유지한다.

### 스크립트가 못박는 두 가지

| 설정 | 이유 |
|---|---|
| `useDomainIDEnvVar=False` + `domain_id=31` | Windows 에서 Isaac 을 GUI 로 띄우면 `ROS_DOMAIN_ID` 가 프로세스 환경에 없을 수 있다. 그러면 도메인 0 으로 붙어 **아무 에러 없이** 스택과 서로 보이지 않는다 |
| `PublishJointState.topicName = /isaac/joint_states` | 기본값이 `joint_states` 라 그대로 두면 우리 fusion 노드의 출력과 **같은 토픽에 publisher 가 둘** 붙는다 |

**Windows 에서는 Isaac 내장 ROS 2 라이브러리를 쓴다** (별도 ROS 2 설치 불필요).
Ubuntu 머신에서는 워크스페이스를 source 한 상태로 실행해도 되지만, D2 에 따라 어느
쪽이든 표준 메시지만 오간다.

### 로봇 에셋 — 재고품인가 URDF 변환본인가

Phase 0 은 **Isaac 재고 Franka 에셋으로 시작한다.** 설계 원칙은 "URDF 를 단일
진실원본으로 두고 USD 를 거기서 생성"이지만, 재고 에셋의 관절 이름이 이미
`panda_joint1~7` 이면 변환 없이 진행할 수 있다.

**그 판정을 Phase 0 수용 기준 2번이 대신한다** — `is_check_phase0.py` 가
`/joint_states` 의 이름을 URDF 와 대조해 다르면 그 자리에서 실패시킨다. 실패하면
그때 URDF importer 로 변환하고, 통과하면 재고 에셋을 계속 쓴다. **먼저 재보고
정하는 순서다.**


## 7. 함정 사전

**증상으로 찾는다.** 여기 있는 것은 전부 실제로 겪은 것이고, 각 항목은
*증상 · 원인 · 판별법* 순으로 적혀 있다.

| 증상 | 원인 | 절 |
|---|---|---|
| 토픽이 하나도 안 보인다 (에러 없음) | Isaac 이 **jazzy** 를 로드했다 | Windows ↔ WSL2 DDS |
| 토픽이 안 보인다 (양쪽 다 정상) | SHM 전송 오판 · 단방향 멀티캐스트 | Windows ↔ WSL2 DDS |
| **일부 노드만** 안 보인다 (넷 넘게 띄운 뒤) | `initialPeersList` 포트 누락 | `initialPeersList` |
| 큰 파라미터·이미지만 안 온다 (작은 건 옴) | MTU 초과 UDP 단편 소실 | `maxMessageSize` |
| 기동 게이트가 **즉시** 또는 **영원히** 실패 | ROS 클럭으로 시간을 쟀다 | 기동 게이트 |
| `TF_OLD_DATA` 경고 후 조회 실패 | Stop 이 sim time 을 0 으로 되돌렸다 | Stop/Play |
| 팔이 목표에 못 미치는데 강성을 올려도 그대로 | **관절 한계 클램프** (처짐이 아니다) | 관절 한계 |
| 그리퍼가 명령을 받는데 안 움직인다 | 손가락이 물체에 박혔다 | scene 물체는 MoveIt 에도 |
| 계획은 되는데 시뮬레이터에서 막힌다 | MoveIt 이 scene 을 모른다 | scene 물체는 MoveIt 에도 |
| `UnicodeDecodeError` 가 뜬다 | 진짜 원인을 cp949 가 가렸다 | 한국어 Windows |
| 저장한 scene 이 비어 보인다 | `strings` 로는 USDC 를 못 읽는다 | 저장된 USD |
| `PoseTree ... eInvalid` 가 60 Hz 로 쏟아진다 | 정적 물체에 RigidBody 가 없다 | §6 정적 물체 |
| 손가락이 **끝까지** 닫히고 물체는 1 mm 도 안 움직인다 | 로봇이 월드 원점에 없다 | 로봇 배치 |
| Play 직후 팔이 탁자 위로 뚝 떨어진다 | 에셋 기본 자세가 탁자 안이다 | 시작 자세 |
| 계획이 `-2`(INVALID_MOTION_PLAN)로 거부된다 | 시작 자세가 충돌이다 — `is_recover.py` | 시작 자세 |
| 트윈 오퍼레이션이 `RUNNING` 인 채 안 끝난다 | `externally_spun` 미전달 | 외부 spin |
| twist 를 보내는데 팔이 거의 안 움직인다 | 개루프 드리프트가 쌓였다 (scene 탓이 아니다) | servo 드리프트 |
| 명령은 29 Hz 로 나가는데 팔이 **전혀** 안 움직인다 | servo 가 **NaN** 을 내보낸다 | servo NaN |
| JSON 을 고쳤는데 ROS 쪽만 옛 값을 쓴다 | `src` 와 `install/share` 사본이 다르다 | 단일 출처 |
| 문 블록이 손가락 사이로 미끄러진다 | 마찰 재질 미바인딩 (기본 0.5) | 마찰은 조용히 |

**진단 도구 세 개면 대부분 갈린다.**

| 도구 | 무엇을 가르나 |
|---|---|
| `scripts/isaac/is_topics.py` | WSL 내부 DDS 가부 + 각 토픽 수신량. 반드시 끝난다 |
| `scripts/isaac/sim_side/check_bridge.py` | Isaac **내부**에서 보이는지. 보이면 브리지는 무죄 |
| `ROS_DOMAIN_ID` 를 바꿔 본다 | 부하 탓인지 참가자 수 탓인지 |
| `scripts/isaac/sim_side/check_colliders.py` | prim 의 **월드 좌표**·물리 설정. 배치 어긋남을 가른다 |

### `initialPeersList` 는 **포트를 명시해야 한다** — 노드 넷을 넘기면 사라진다

**증상** — 스택 노드를 하나 더 추가했더니 **rclpy 노드 다섯이 통째로 안 보이게 됐다.**
`ros2 node list` 에 C++ 노드(move_group·robot_state_publisher)와 Isaac 노드만 뜨고,
Python 노드는 프로세스가 멀쩡히 돌면서 로그도 남기는데 아무도 그 발행을 못 받는다.

**원인** — `initialPeersList` 에 주소만 적고 포트를 생략하면 Fast DDS 가 **참가자 ID
0~3 만** 훑는다(`maxInitialPeersRange` 기본 4). 참가자가 넷을 넘는 순간, 뒤에 뜬
참가자는 유니캐스트 탐색 범위 밖으로 밀려난다. 멀티캐스트가 한 방향만 통하는
mirrored networking 에서는 그것을 메울 수단도 없다.

**해결** — 포트를 열거한다. 도메인 31 기준 ID 0~15.

```
포트 = 7400 + 250*도메인 + 10 + 2*참가자ID
     = 15160, 15162, ... 15190   (도메인 31)
```

**도메인을 바꾸면 이 목록을 다시 만들어야 한다.**

> **오진의 기록** — 마침 같은 시점에 카메라(Phase 4)를 켠 터라 "이미지 트래픽이
> 디스커버리를 굶긴다"고 한참 헤맸다. 갈라낸 방법은 **도메인을 바꿔 보는 것**이었다.
> 트래픽 없는 도메인 77 에서 같은 Python 노드 둘이 서로를 즉시 보았고, 그 순간
> 트래픽 가설이 죽고 참가자 수 가설이 남았다.
>
> 부하를 의심할 때는 **부하만 없는 동일 조건**을 만들어 보는 것이 가장 빠르다.

### MTU 를 넘는 메시지가 조용히 사라진다 — `maxMessageSize`

**증상** — `/move_group/get_parameters` 로 `robot_description_semantic`(SRDF, 9 KB)을
요청하면 30초를 기다려도 응답이 없다. 같은 서비스로 `use_sim_time`(bool)은 즉시 온다.

**격리 재현** — Isaac 도 MoveIt 도 무관하다. 노드 두 개만으로 재현된다.

| 파라미터 크기 | 기본 프로파일 | `maxMessageSize=1400` |
|---|---|---|
| 100 B | 0.00s 수신 | — |
| 5 KB | **15s 타임아웃** | — |
| 9 KB | **15s 타임아웃** | **0.00s 수신** |
| 20 KB | **15s 타임아웃** | **0.08s 수신** |

**원인** — 기본값(65500)이면 9 KB 응답이 UDP 데이터그램 하나로 나가고 커널이 IP
단편화를 하는데, **WSL2 mirrored networking 에서 그 조각이 버려진다.** RELIABLE 이라도
같은 크기로 재전송하므로 영영 도착하지 않는다. 1400 으로 두면 RTPS 가 직접 조각내어
IP 단편화가 일어나지 않는다.

**보내는 쪽에도 적용해야 한다** — 받는 쪽만 바꾸면 그대로 실패한다(실측).

> **토픽은 멀쩡해 보였다** — 반복 발행이라 재시도가 손실을 가렸다. 단발성인 서비스
> 응답에서만 드러났다. 크기가 큰 채널을 시험할 때는 **한 번만 보내 보는** 것이
> 중요하다.
>
> 이 설정이 없었으면 **Phase 4 의 카메라 이미지도 같은 벽**에 부딪혔을 것이다.
> 문서에 "대역폭 문제"로 적어 뒀던 것이 실은 이 단편화 문제였다.

### Windows ↔ WSL2 DDS — 전송 프로파일이 **필수다** (해결됨)

**증상** — WSL 에서 Isaac 토픽이 하나도 안 보인다. `/clock` 조차 0건인데 어디에도
에러가 없다. Isaac 은 재생 중이고 그래프도 tick 하며, 방화벽도 전부 Allow 다.

**원인은 둘이 겹친 것이었다.**

1. **SHM 전송 오판** — mirrored networking 에서 WSL2 는 Windows 의 네트워크 정체를
   그대로 쓴다. 그래서 Fast DDS 가 양쪽 참가자를 **같은 호스트**로 판단해 공유메모리
   전송을 고른다. Windows 의 SHM 과 Linux 의 `/dev/shm` 은 다른 물건이라 아무것도
   도착하지 않는다.
2. **단방향 멀티캐스트** — SHM 을 끄자 `Windows → WSL2` 만 뚫렸다. WSL 은 Isaac 의
   구독자(`/isaac/arm_command`)를 보는데 Isaac 은 WSL 의 구독자를 못 봐서, **보낼 곳을
   모른 채 발행하지 않는** 상태가 됐다.

**해결** — `config/fastdds_wsl_bridge.xml` 을 **양쪽 모두**에 건다.

```xml
<useBuiltinTransports>false</useBuiltinTransports>   <!-- SHM 제거 -->
<transport_descriptor><type>UDPv4</type></...>       <!-- UDP 만 -->
<initialPeersList>127.0.0.1</initialPeersList>       <!-- 유니캐스트로 서로 찌름 -->
```

| 쪽 | 거는 법 |
|---|---|
| WSL2 | `export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/src/robot_control/config/fastdds_wsl_bridge.xml` |
| Isaac | `run_isaac_humble.bat` 이 `C:\isaacsim\fastdds_wsl_bridge.xml` 로 설정 |

**한쪽만 걸면 안 된다.** 전송 선택은 양쪽 합의다 — WSL 만 UDP 로 바꿔도 Isaac 이
SHM locator 만 광고하면 닿을 곳이 없다.

적용 후 실측: `/clock` **60.4 Hz** · `/joint_states` **59.9 Hz** · `/tf` **59.6 Hz**.

> **Ubuntu 22.04 단독 호스트에서는 이 프로파일을 쓰지 않는다.** 거기서는 SHM 이
> 실제로 같은 호스트를 뜻하고 UDP 보다 빠르다. §1 의 "호스트 차이는 DDS 설정뿐"이
> 바로 이 자리이며, 이번에 그 내용이 구체화됐다.

**진단 사다리 — 이 순서로 구간이 특정된다.**

| # | 도구 | 무엇을 가르나 |
|:-:|---|---|
| 1 | `scripts/isaac/is_topics.py` | 자체시험으로 **WSL 내부 DDS** 가부, 각 토픽 수신량. `ros2 topic hz` 와 달리 반드시 끝난다 |
| 2 | `scripts/isaac/sim_side/check_bridge.py` | **Isaac 내부**에서 토픽이 보이는지. 보이면 브리지는 무죄 |
| 3 | `Get-NetFirewallApplicationFilter` / `Get-NetFirewallHyperVVMSetting` | Windows·Hyper-V 방화벽 |
| 4 | 전송 프로파일 | 위 셋이 정상인데 안 통하면 여기다 |

> **스택을 띄우기 전에 `is_topics.py` 를 먼저 돌린다.** 이 상태에서 스택을 띄우면
> `readiness_gate` 가 60초를 기다린 뒤 실패하는데, 그 로그만 보면 스택 문제처럼
> 보인다. 실제로 그렇게 두 번 헛짚었다.
>
> 또 하나 — 진단 스크립트는 토픽 타입을 **미리 못박아** 구독해야 한다. 그래프에서
> 타입을 조회해 구독하면 "안 보이니 구독 안 함 → 구독이 없으니 발행 안 함" 교착에
> 빠진다. 실제로 그 교착 때문에 원인을 한 번 더 놓쳤다.

### 저장된 USD 는 `strings` 로 검증할 수 없다

저장한 scene 이 8.8 KB 라 `strings` 로 들여다봤더니 `ActionGraph` 도 `stiffness` 도
안 잡혀 "빈 파일"로 판정했다. **틀린 판정이었다.** USDC crate 는 토큰 테이블을
압축하므로 실제로 들어 있는 이름도 검색되지 않는다 (덤프가 `0dam`, `5arm`,
`P.ROS2X` 처럼 깨져 나온다).

**`scripts/isaac/sim_side/verify_saved_scene.py` 로 확인한다** — 저장된 파일을
별도 스테이지로 열어 ActionGraph prim · 타임라인 구간 · drive gain 을 찍는다.
작업 중인 스테이지를 건드리지 않으므로 아무 때나 돌려도 된다.

```
[verify] ActionGraph prims: 10
[verify] timeline: start=0.0 end=10000000.0
[verify] panda_joint4: stiffness=10000.0 damping=400.0
[verify] verdict: SAVED OK
```

> 참고로 `setup_graph.py` / `tune_drive.py` 는 실행 시 edit target 을 root layer 로
> 되돌린다. 이번 사례에서는 원래 root 였지만(오진이었다), session layer 에 쓰면
> 저장에 담기지 않는 것은 사실이므로 방어적으로 남겨 둔다.

### 기동 게이트의 시간 기준은 벽시계여야 한다

`readiness_gate` 가 **60초 한도를 113 ms 만에** 터뜨린 적이 있다. deadline 을
ROS 클럭으로 잡았기 때문이다.

`use_sim_time` 이 켜진 상태에서 첫 `/clock` 을 받기 전까지 `now()` 는 **0** 이다.
그 시점에 `deadline = 0 + 60s` 로 잡히는데, 시뮬레이터가 이미 오래 돌아 sim time 이
60 을 넘겼으면 **첫 `/clock` 이 도착하는 순간 곧바로 타임아웃**한다. `resetOnStop`
을 끈 뒤로는 sim time 이 리셋되지 않으므로 이 조건이 쉽게 성립한다.

반대 방향도 있다 — `/clock` 이 영영 오지 않으면 시간이 0 에 멈춰 **영원히 대기**한다.
실측으로 둘 다 재현했다.

`time.monotonic()` 으로 고쳤다. 기동 게이트가 재려는 것은 시뮬레이션 시간이 아니라
**사람이 기다리는 실제 시간**이다.

> 이 실패는 진단을 거꾸로 이끈다 — "타임아웃"이라 시뮬레이터가 죽은 것처럼 보이지만,
> **113 ms 만에 터졌다는 사실 자체가 `/clock` 이 오고 있었다는 증거**다. 로그의
> 시각 차이를 보는 것이 먼저다.

### Stop/Play 는 시간을 되돌린다 — TF_OLD_DATA

Isaac 을 Stop 하면 sim time 이 0 으로 리셋된다. 스택은 계속 떠 있으므로 TF 버퍼에
이미 **미래** 시각의 변환이 들어 있고, 다시 Play 하면 새 데이터가 과거로 보여 버려진다.

```
[ee_pose_node] Warning: TF_OLD_DATA ignoring data from the past for frame panda_link6
```

경고로만 끝나지 않는다 — TF 조회가 조용히 실패하고 `move_group` 이 현재 상태를 못
읽는다. **`ReadSimTime.inputs:resetOnStop = False`** 로 막는다(`setup_graph.py` 가
설정한다). 그러면 Stop 에서 시간이 **멈출 뿐 되돌아가지 않는다.**

이미 경고가 난 세션은 버퍼가 오염돼 있으므로 **스택을 재시작한다.** 순서는 언제나
**Isaac Play → 스택 기동**이다.

### 관절 한계 — URDF 가 실제 로봇보다 헐겁다 (Q6 결정)

`moveit_resources_panda` 의 URDF 는 **전 관절이 실제 Franka 스펙보다 4° 헐겁고**,
`panda_joint4` 상한은 **9°** 헐겁다. 규칙적인 패딩이라 한 관절만의 문제가 아니다.

| joint | URDF | 실제 스펙 | 차 |
|---|---|---|---|
| joint1·3·5·7 | ±2.9671 | ±2.8973 | 0.0698 (4°) |
| joint2 | ±1.8326 | ±1.7628 | 0.0698 |
| joint6 | -0.0873 ~ 3.8223 | -0.0175 ~ 3.7525 | 0.0698 |
| **joint4** | -3.1416 ~ **+0.0873** | -3.0718 ~ **-0.0698** | 상한 **0.1571 (9°)** |

**Isaac 은 실제 스펙을 강제한다.** 그래서 MoveIt 이 계획한 자세를 시뮬레이터가 한계에서
막고, 그것이 추종 오차처럼 보인다 — `extended`(j4=0)로 보냈더니 팔이 -0.0698 에서
멈췄고 그 값을 중력 처짐으로 오독했다. 강성을 **625배** 올려도 소수점 4자리까지
그대로인 것을 보고서야 갈렸다(처짐은 K 에 반비례하고 한계 클램프는 무관하다).

**결정: 실제 스펙 쪽으로 좁힌다.** 수집한 데이터가 실물로 가야 하므로, 실제 로봇이
갈 수 없는 자세가 데이터셋에 섞이면 안 된다. 계획 단계에서 막는 편이 시뮬레이터가
조용히 클램프하는 것보다 낫다.

`config/panda_real_joint_limits.yaml` 에 position 한계를 넣고 `panda_isaac.launch.py`
가 `build_moveit_config(joint_limits_file=...)` 로 넘긴다. 속도·가속도는 원본 그대로다.

**적용 범위는 Isaac 뿐이다.** mock/Gazebo/펑션베이까지 넓히는 것은 별도 결정으로 남긴다
— 그 백엔드들은 한계를 강제하지 않아 당장 문제가 없고, 공용 헬퍼 기본값을 바꾸면
검증되지 않은 영향이 다른 launch 로 번진다.

> **부작용 하나** — SRDF 의 `extended` 는 j4=0 을 요구하므로 이제 **계획이 실패한다.**
> 실물에 없는 자세이므로 실패가 옳다. `is_check_phase0.py` 의 계획 검사는 `ready` 로
> 바꾸고, SRDF 파싱은 named target **목록 조회**로 따로 확인한다.

### scene 물체는 MoveIt 에도 알려야 한다 (Q8 결정)

`/scene/objects` 를 발행하는 것만으로는 **MoveIt 이 물체를 모른다.** 빈 공간을 가정해
계획하므로 팔이 테이블을 뚫는 궤적이 나온다 — Phase 2 에서 손가락이 상판 안에 박혀
그리퍼가 물리적으로 막혔고, 이동량 0.0018 m 를 "그리퍼 고장"으로 오독했다.

(2026-09-01 삭제됨) 예전에는 `planning_scene_sync` 가 `/scene/objects` 를 planning scene 으로 옮겼다.

```
mock 백엔드   : planning scene → /scene/objects   (읽는다)
이 노드        : /scene/objects → planning scene   (쓴다)
```

**그래서 mock 백엔드에서는 띄우면 안 된다** — 자기가 읽은 것을 자기가 되쓰는 고리가
된다. 시뮬레이터가 scene 의 원본인 백엔드(Isaac·펑션베이)에서만 쓴다.

실측 — 적용 후 planning scene 에 물체 4개가 올라가고, 문제의 자세는 계획 단계에서
막힌다.

```
충돌 물체: block_a(0.05³) block_b(0.05³) block_c(0.05³) table(0.6×1.0×0.4)
ready 계획       : 성공
테이블 관통 자세  : RuntimeError code -27 (GOAL_IN_COLLISION)
```

> **부작용을 의도한 것이다.** 전에는 계획이 성공하고 시뮬레이터에서 조용히 막혔다.
> 이제는 계획이 실패한다 — 실패가 눈에 보이는 편이 낫다.
>
> **남은 과제**: 물체를 **집을 때**는 손가락과 물체의 접촉이 충돌로 잡힌다. 파지
> 시나리오에서는 `AttachedCollisionObject` 로 옮기거나 허용 충돌 행렬을 손봐야 한다.

### Isaac 은 관절 한계를 **강제한다**

펑션베이는 한계를 넘겨도 그냥 갔고(그 문서 §2.2) 그것이 복구 불능 자세(§4)의
원인이었다. Isaac 은 막는다 — 안전하지만, **MoveIt 이 계획한 자세를 시뮬레이터가
거부할 수 있다**는 뜻이기도 하다 (Q6).

막힌 것인지 처진 것인지는 **강성을 바꿔 보면 갈린다.** 처짐은 `오차 = 토크/K` 라
K 에 반비례해 줄고, 한계 클램프는 **K 를 625배 해도 소수점 4자리까지 그대로**다.
실제로 그 방법으로 갈랐다.

### 한국어 Windows 의 함정 — 오류가 오류를 가린다

ROS 2 라이브러리 로딩이 실패하면 Isaac 은 그 하위 프로세스 출력을
`grepexc.output.decode("utf-8")` 로 읽는다. 한국어 Windows 는 오류 문구를 **cp949**
로 내보내므로 여기서 `UnicodeDecodeError: 'utf-8' codec can't decode byte 0xc1` 가
터지고, **진짜 원인 대신 이 예외가 보인다.**

이 예외를 보면 인코딩 문제로 오해하지 말고 그 아래
`Failed to load system ROS 2 libraries for ROS_DISTRO='...'` 줄을 찾는다. 그쪽이
원인이다.

### 로봇이 **월드 원점에 없으면** 모든 좌표가 조용히 어긋난다

ROS 쪽 static TF 가 `world -> panda_link0` 를 **항등변환으로 못박고** 있다
(`launch_helpers/common.py`). 에셋을 스테이지에 드래그해 올리면 원점이 아닌 자리에
떨어지는데, 그러면 양쪽이 어긋난 채로 **각자 일관되게** 동작한다.

* 로봇 내부는 맞는다 — `/joint_states` → `robot_state_publisher` → TF 가 전부
  `panda_link0` 기준이라, 로봇이 어디 놓여 있든 자기들끼리는 정확하다.
* scene 물체는 `setup_scene.py` 가 **월드 좌표로** 만든다.

그래서 그리퍼는 *"목표에 0.5 mm 오차로 도달"* 이라고 보고하면서 실제로는 물체에서
수십 cm 떨어진 허공을 쥔다. **에러도 경고도 없다.**

실측 — `panda_link0` 이 월드 `(-0.035, -0.349, 0)` 에 있었다. 파지 목표는
`panda_link0` 기준 `(0.500, -0.150, 0.528)` 이었고 `panda_hand` 의 월드 위치는
`(0.465, -0.499, 0.528)` 로 정확히 그만큼 밀려 있었다. 블록은 월드 `(0.500, -0.150)`
그대로였으니 손은 **y 축으로 35 cm 떨어진** 자리를 쥔 것이다.

**증상이 마찰·콜라이더를 가리킨다.** 손가락은 0 까지 닫히고, 블록은 소수점 넷째
자리까지 움직이지 않는다 — "물체를 통과했다"로 읽히고, 실제로는 통과한 게 아니라
애초에 거기 없었다. 마찰 재질과 콜라이더 설정을 한참 뒤진 뒤에야 드러났다.

판별 — `check_colliders.py` 가 각 prim 의 **월드 좌표**를 찍는다. `panda_link0` 이
원점이 아니면 이것이다. 고치는 것은 `place_robot.py` (Stop 상태에서, 멱등).

> `check_colliders.py` 의 `collision=NONE` 과 `gravity=-inf` 는 **오탐**이다.
> 전자는 Franka 에셋이 instanceable 이라 `GetChildren()` 이 프로토타입 내부를
> 못 보는 것이고, 후자는 USD 의 '기본 중력' 센티널이다. 둘 다 정상이다.

### 시작 자세 — 에셋 기본값은 **탁자 안**이다

Franka 에셋의 기본 관절값은 손목이 말려 있다(`panda_joint6 = 3.1475`, `ready` 는
1.571). 로봇을 원점에 놓고 나면 그 자세에서 손이 탁자 상판 안으로 들어가므로,
Play 를 누르는 순간 **팔이 탁자 위로 뚝 떨어지는 것처럼** 보인다.

**증상이 두 갈래로 나타난다.** 하나는 눈에 보이는 낙하이고, 다른 하나는
`INVALID_MOTION_PLAN`(-2)이다 — 시작 자세가 충돌이면 MoveIt 은 계획 자체를 거부한다.
후자는 원인이 전혀 안 보인다.

**로봇이 원점에서 벗어나 있으면 이 증상이 숨는다.** 탁자를 비껴가 허공에 떨어지기
때문이다. 배치를 고친 뒤에야 드러나므로, 두 함정은 순서대로 만나게 된다.

`탁자가 너무 높은 것이 아니다` — 실측으로 `ready` 는 상판(z=0.400)에서 손끝이
z≈0.487 로 **8.7 cm 여유**가 있고 `/check_state_validity` 가 `valid: True` 를 준다.
높이를 낮추면 증상은 가려지지만 원인은 그대로 남는다.

고치는 것은 `set_home_pose.py` 다. **두 속성을 함께 쓴다** — `state:angular:physics:position`
(PhysX 의 시작 상태)과 `drive:angular:physics:targetPosition`(드라이브 목표). 앞의
것만 쓰면 시작하자마자 드라이브가 옛 목표로 끌고 가고, 뒤의 것만 쓰면 시작 자세에서
목표까지 한 번 크게 휘두른다.

> **USD 의 각도는 도(degree)다.** 설정에는 라디안으로 적고 스크립트가 바꾼다. 안
> 바꾸면 1.571 rad 가 1.6도가 되어, 크래시 없이 '그럴듯하게 틀린' 자세가 된다.

### 단일 출처 — `src` 와 `install/share` 는 **다른 파일**이다

`isaac_scene.json` 을 두 쪽이 읽는데 **경로가 다르다.**

| 읽는 쪽 | 경로 |
|---|---|
| Isaac 스크립트 (`setup_scene.py`, `tune_grasp.py`, `set_home_pose.py`) | `src/robot_control/config/` |
| ROS `isaac_scene_state_node` | `install/robot_control/share/robot_control/config/` |

Isaac 쪽은 ament 를 못 쓰므로 저장소 경로를 직접 읽을 수밖에 없다. 그래서 **"단일
출처"는 `colcon build` 를 해야만 성립한다.** 물체를 추가하고 `setup_scene.py` 만
다시 돌리면 Isaac 에는 생기고 ROS 는 모르는 상태가 조용히 만들어진다 — `/scene/objects`
에 그 물체만 없고, 에러는 없다.

판별 — 아래가 서로 다르면 빌드를 빠뜨린 것이다.

```bash
diff <(python3 -c "import json;print(json.dumps(json.load(open('src/robot_control/config/isaac_scene.json')),indent=1))") \
     <(python3 -c "import json;print(json.dumps(json.load(open('install/robot_control/share/robot_control/config/isaac_scene.json')),indent=1))")
```

### 외부에서 spin 되는 노드에는 **`externally_spun` 을 넘겨야 한다**

`MoveGroupClient` 의 여러 대기 지점은 기본적으로 **노드를 직접 spin** 한다
(`await_future_spin`). 스크립트에서는 맞지만, executor 가 이미 다른 스레드에서 그
노드를 돌리는 곳(로봇 트윈)에서는 콜백이 executor 쪽으로 가므로 **여기서는 영영
오지 않는다.**

**예외도 타임아웃도 없다.** 오퍼레이션이 `RUNNING` 인 채로 남고 팔은 움직이지 않으며,
로그에는 아무 단서도 없다.

실측 — `plan_joints_async` 가 이 인자를 **받지도 넘기지도 않아** 트윈의
`move_to_joints` 가 그렇게 멈췄다. 인자를 추가하고
`move_to_joints_streamed_async` 에서 전달하게 고쳤다.

`plan_named_target_async` 는 관절값을 콜백 체인으로 얻으므로 애초에 spin 하지 않는다.
그래서 **`move_to_named_target` 만 먼저 동작해** 원인이 더 헷갈렸다 — "트윈은 되는데
관절 지정만 안 된다"로 보인다.

회귀 검사는 `moveit/tests/test_externally_spun_propagation.py` 에 있다.

### servo 개루프 주행은 **드리프트가 쌓여 결국 멈춘다**

twist 를 반복해 보내면 팔이 서서히 어긋나고, 충분히 쌓이면 servo 가 움직이기를
거부한다.

```
/servo_node/status
  HALT_FOR_COLLISION        80건
  DECELERATE_FOR_COLLISION  67건
  NO_WARNING                 0건
```

**원인이 현재 명령이 아니라 이전 이력에 있다.** 방금 보낸 twist 는 정상인데 팔이
안 움직이므로, 그 명령이나 그때 켜 둔 기능을 의심하게 된다.

**실제로 그렇게 오독했다.** 처음에는 `enable_scene:=true` 를 원인으로 지목했다 —
scene 을 끄고 다시 띄우니 움직였기 때문이다. 그러나 그 재기동 과정에 `is_recover.py` 가
끼어 있어 **자세가 함께 초기화된** 것이 진짜 차이였다.

가른 방법은 **매 시행 전에 `ready` 로 되돌리고** planning scene 내용만 바꾼 것이다.

| 조건 | 이동(명령 0.2 m/s x 2s) | status |
|---|---|---|
| diff 없음 | +0.1843 m | `NO_WARNING` x103 |
| 빈 diff (물체 0) | +0.1955 m | `NO_WARNING` x103 |
| table 만 | +0.1974 m | `NO_WARNING` x103 |
| table + 블록 3 | +0.1829 m | `NO_WARNING` x95 |

**scene 물체는 무관하다.** 넷이 같다. 결정적인 것은 대조군에서도 오염된 자세로 시작하면
planning scene 이 **완전히 비어 있는데도** `DECELERATE_FOR_COLLISION` 이 나온다는
점이다 — scene 이 원인이면 나올 수 없다. 남는 것은 자기충돌뿐이다.

**대응** — 텔레오퍼레이션 세션 사이에는 `scripts/isaac/is_recover.py` 로 자세를
되돌린다. 에피소드 경계마다 되돌리는 것이 데이터 품질에도 맞다. 추종 배율이
명령 대비 0.46~0.49 인 것도 같은 개루프 성질에서 온다.

### servo 는 **NaN 을 내보내면서 `NO_WARNING` 을 유지한다**

servo 출력 일곱 값이 전부 NaN 이 되는 상태가 있다. 그런데,

```
/isaac/servo_command   29.4 Hz   전부 [nan, nan, nan, nan, nan, nan, nan]
/servo_node/status     전부 NO_WARNING
관절 변화               0.0000 (7축 모두)
```

**모든 지표가 정상으로 보인다.** 발행 주기는 `publish_period` 와 정확히 맞고, 경고는
없고, 노드도 살아 있다. 팔만 안 움직인다. 값을 직접 찍기 전까지 드러나지 않는다.

#### 전제조건 — 재현했다

무작위가 아니다. **두 가지가 겹쳐야** 한다.

1. servo 가 **충돌 정지 상태**다 (`HALT_FOR_COLLISION` / `DECELERATE_FOR_COLLISION`)
2. 그 상태에서 **외부가 로봇을 점프시킨다** — `/isaac/arm_command` 에 직접 쓰는 것

```
한 방향 주행 -> status {3: 72, 4: 5}   (HALT 진입)
그 상태에서 직접 스트리밍
  -> 이후 twist  NaN 68/68,  status {0: 68}
```

**건강한 상태에서는 같은 직접 스트리밍을 해도 NaN 이 안 난다** — 그래서 처음에는
재현이 안 돼 원인을 못 잡았다. 전제조건을 갖추자 1회에 재현됐다.

`is_recover.py` 가 바로 그 "직접 쓰기"를 하므로, **드리프트 대응책이 NaN 을
유발하는 구조**였다. 지금은 스트리밍 전후로 servo 를 멈췄다 켠다.

#### `pause`/`unpause` 를 쓰면 안 된다

처음에는 `pause_servo` / `unpause_servo` 로 고쳤는데 **더 나빴다.**

```
현재        출력 0건
unpause -> (True, '')  ->  출력  0건     성공을 반환하면서 복구하지 않는다
start   -> (True, '')  ->  출력 68건
```

`unpause_servo` 는 **성공을 반환하면서도 발행을 되살리지 못한다.** 그 쌍을 쓰면 복구할
때마다 servo 가 조용히 죽고, 증상은 다시 "팔이 안 움직인다"가 된다. `stop_servo` /
`start_servo` 를 쓴다.

수정 후 같은 전제조건에서 확인 — 복구가 정상 완료되고(관절 오차 0.0003 rad) 이후
twist 3회 모두 **NaN 0건**이다.

#### 다리가 막는다

`isaac_servo_bridge` 가 유한하지 않은 값을 **버리고 경고**한다. 원인을 못 막더라도
NaN 이 시뮬레이터까지 가지는 않게 한다 — 가면 조용히 무시되어 증상이 servo 바깥을
가리킨다.

### 마찰은 **조용히** 기본값이 된다

USD 에서 마찰은 prim 의 속성이 아니라 **별도 Material prim** 이고, 콜라이더에
`physics` purpose 로 바인딩해야 적용된다. 안 붙여도 경고가 없다 — PhysX 기본값
(정적·동적 모두 0.5)이 쓰이고, 그 값으로는 매끈한 상자를 옆에서 눌러 들지 못한다.

**증상이 '파지 실패'가 아니라 '집었는데 안 들린다'로 나온다.** 손가락은 정상적으로
닫히고, 액션은 성공으로 돌아오고, MoveIt 의 attach 도 걸린다. 팔만 올라가고 블록은
테이블에 남는다.

메시가 자식 prim 에 있는 에셋이면 링크에만 바인딩해도 새어나간다. `tune_grasp.py` 는
링크와 **그 아래 `CollisionAPI` 가 붙은 prim 전부**에 건다.

판별 — `is_check_phase6.py` 의 4번(들어올림)만 실패하고 3번(물림)은 통과한다.
3번까지 실패하면서 폭이 0 에 가까우면 마찰이 아니라 **접근 높이**가 어긋난 것이다
(`TCP_OFFSET` 0.1034 m, 또는 손이 아래를 보지 않는 경우 — 검사가 기울기를 함께 찍는다).

## 8. 열려 있는 결정

| # | 항목 | 언제 정하나 |
|:-:|---|---|
| ~~Q1~~ | **닫힘 (2026-08-29)** — Isaac 은 `name` 을 채우고 팔 7관절 + finger 2관절을 모두 보낸다. `joint_state_fusion` 을 **제거**했다. 남겨 두면 `extra_joint_*` 주입을 끌 수 없어(빈 리스트는 기본값으로 되돌아간다) `panda_finger_joint1` 이 중복 발행된다 | — |
| ~~Q2~~ | **닫힘 (2026-08-29)** — **TF** 로 확정. Isaac 이 `ROS2PublishTransformTree` 를 기본 제공하고, 좌표 변환·쿼터니언 규약을 tf2 가 대신한다. Phase 3 실측으로 검증됐다 | — |
| ~~Q3~~ | **닫힘 (2026-08-30) — 지원한다.** teleop 두 경로(`teleop_keyboard` 의 twist, `teleop_retarget` → `ee_twist_node`)가 **모두 servo 로 수렴**하므로 없으면 Isaac 을 손으로 몰 수 없다. servo 출력을 `sensor_msgs/JointState` 로 옮기는 다리(`isaac_servo_bridge`)를 두고 `enable_servo` 로 켠다(§2 Phase 10). 펑션베이에서 무산된 것은 그 백엔드에 그리퍼·컨트롤러가 함께 없었기 때문이며, 타입 문제 자체는 같은 방식으로 풀린다 | — |
| Q4 | Ubuntu 22.04 머신의 GPU 사양 | Phase 4 는 **이 PC 에서 완료**했다(640×480 @ 5 Hz). 고해상도·고주파가 필요해질 때 확인한다 |
| ~~Q6~~ | **닫힘 (2026-08-29)** — **실제 Franka 스펙으로 좁혔다.** `config/panda_real_joint_limits.yaml` 을 `panda_isaac.launch.py` 가 넘긴다. 적용 범위는 Isaac 뿐이며, mock/Gazebo/펑션베이까지 넓힐지는 별도 결정 |  — |
| ~~Q9~~ | **철회 (2026-09-01).** `attach`/`detach` 자체를 없앴다. (1) 근거였던 "rosbag 에 남는다"가 **거짓**이었다 — `/scene/commands` 는 `recording_topics.list` 에 없다. (2) `attach` 는 명시적 detach 까지 유지되는데 **물체는 미끄러져 떨어질 수 있어** 믿음이 조용히 틀린다. 떨어지면 planner 는 손에 있다고 믿으면서 탁자 위의 그것도 못 본다(붙은 물체는 world 동기화에서 제외되므로). (3) 애초에 파지 동작은 cartesian 이라 `avoid_collisions` 기본값 `False` 로 **planning scene 을 보지 않는다** | — |
| ~~Q8~~ | **닫힘 (2026-08-29) → 축소 (2026-09-01) → 철회 (2026-09-01).** 최종: **planning scene 에 아무것도 넣지 않는다.** `planning_scene_sync` 노드와 `SceneObject.fixture` 필드를 삭제했다. 경위 — 조작 대상을 넣으면 파지가 시작 자세 충돌이 되어 `INVALID_MOTION_PLAN`(-2)이므로 고정물만 넣도록 축소했는데, 이어서 **환경 물체를 `/scene/objects` 에 아예 발행하지 않기로** 정하자 넣을 것이 남지 않았다. 원래 동기("팔이 탁자를 뚫는다", `panda_link4`·`panda_link5` 가 상판을 파고든 것을 실측)는 **단순성을 위해 감수한다**. 되살릴 때는 토픽→scene 노드가 아니라 탁자만 launch 시점에 상자 하나로 정적 등록하는 편이 맞다 | — |
| ~~Q7~~ | **닫힘 (2026-08-29)** — `C:\isaacsim\scenes\panda_rdfp.usd` 에 저장 완료. ActionGraph 10개 prim · 타임라인 0..1e7 · drive gain 10000/400 이 모두 담긴 것을 `verify_saved_scene.py` 로 확인했다. 다음부터는 열고 Play 만 하면 된다 | — |
| ~~Q10~~ | **닫힘 (2026-08-30)** — `moveit.arm_command_topic` / `arm_command_format` / `arm_command_joint_names` 를 설정에 추가하고 `config/robot_twin_panda_isaac.yaml` 을 만들었다. REST 로 팔·그리퍼·세션 제어를 실측했다(§2 Phase 9). `reset_scene` 은 원리적으로 불가능해 노출하지 않는다 | — |
| Q5 | `PublishJointState.targetPrim` 이 deprecated — `IsaacReadJointState` 노드 출력을 연결하는 방식으로 교체할지 | Phase 0 통과 후 (동작에는 지장 없음) |

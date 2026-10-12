# Isaac Sim 백엔드 — 단계 계획 Phase 0~11 (보관)

> **보관본 (2026-09-21).** [../../backends/isaac/isaac_backend.md](../../backends/isaac/isaac_backend.md) §2 의 원문이다 —
> 열한 단계가 전부 끝나 현행 문서에서 떼어 냈다. 갱신하지 않는다. 각 Phase 의 수용 기준과 실측값,
> 철회 경위, 그리고 §8 에서 닫힌 결정의 판단 이력이 여기 있다.

---

## 2. 단계 계획

> **Phase 0~10 은 2026-08~09 초의 기록이다.** 각 단계의 **수용 기준과 실측**이
> 지금 돌아가는 검사 7종의 근거이므로 그대로 둔다. 기동 순서는 §3, 현재 구성은
> Phase 11 을 본다.

**진행 상황 (2026-09-05)** — **Phase 0~11 완료.** 연동 방식은 하나로 정리됐다(§0 D1″ —
ros2_control 없이 토픽만으로 잇던 bridge 백엔드를 삭제했다). 검사 7종이 초록이고
**수집 전 과정**(세션 → `/scene/reset` → 에피소드 → rosbag → mp4 → DB)이 통과했다.

**미검증** — ① **Windows 에서의 기동**(§1 부록). 2026-08-29 의 "검증 완료"는 ros2_control
없이 토픽만으로 잇던 옛 구성 기준이다. ② **배포 사이의 노드 그래프 diff**(§1 불변식) — 처방만 있고 해 본 적이 없다.
열린 결정은 §8 의 **넷**이다 — Q5·Q17·Q18·Q19.

> ## Isaac Sim 6.0 · Ubuntu 재검증 — 2026-09-02
>
> **기동 절차는 [isaac_bringup_runbook.md](../../backends/isaac/isaac_bringup_runbook.md) 에 따로 있다.** 기동은 [`scripts/run_isaac_sim.sh`](../../../scripts/run_isaac_sim.sh) 로 한다 — 환경 변수를 틀리면 ROS 확장이 조용히 죽는다(②).
>
> | | |
> |---|---|
> | `--gui` | **창 + 전 과정 자동.** 평소 쓰는 것 — Script Editor 를 안 쓴다 |
> | `--headless` | 창 없이 같은 일 (검사·CI). 아래 검증이 이것으로 재현됐다 |
> | (인자 없음) | 전체 편집기. 자산 브라우저·프로퍼티 편집이 필요할 때만 |
>
> **스테이지를 저장해 두는 방식은 쓰지 않는다.** 저장하면 물체 이름·크기가 USD 와 `isaac_scene.json` 두 곳에 살게 되고 조용히 갈라진다 — JSON 은 `isaac_scene_state_node` 도 읽는 정본이다.
>
> Phase 0~9 는 **Windows Isaac 5.1 + WSL2**(§1 부록)에서 통과한 것이라, Ubuntu 단일 머신(§1 기본) + Isaac 6.0 에서 다시 돌렸다.
>
> | Phase | 결과 | 비고 |
> |:-:|---|---|
> | 0 골격 | ✅ **5/5** | 배속 0.97, `/joint_states` 9관절 |
> | 1 팔 명령 | ⚠️ **4/5** | τ 59.9 ms (기준 ≤50). `tune_drive.py` 로 177→60 ms. 정착 오차는 0.0001 rad 로 통과 |
> | 2 그리퍼 | ✅ **6/6** | |
> | 3 scene 객체 | ✅ **5/5** | 검사 쪽 수정 후 (아래) |
> | 4 카메라 | ✅ **6/6** | 4.7 Hz · 640×480 rgb8 · **내용 확인** · camera_info. 크래시는 재현되지 않았다 (아래) |
> | 6 물리 파지 | ✅ **6/6** | 폭 0.0251 m·`stalled`, +9.8 cm 들어올림, 3초 유지 낙하 0.6 mm |
>
> **pick-and-place 는 Isaac 6.0 에서 동작한다.** 카메라만 별도 작업이다.
>
> **① Phase 4 카메라 — 크래시는 재현되지 않았고, 카메라는 정상이다.** 처음에는 "Play 시 segfault" 로 판단해 `PHASE` 를 3 으로 낮췄다. 그 크래시는 `isaacsim.exp.full.kit` GUI 세션에서 **한 번** 났을 뿐이고, 스크립트 기동에서는 **헤드리스와 창 모드 둘 다 PHASE=4 로 통과**한다. 검사도 5/5 다 (4.57 Hz, 640×480 rgb8, camera_info fx=733). `PHASE` 기본값을 4 로 되돌렸다.
>
> 다만 크래시가 났던 것 자체는 사실이다 — 로그의 마지막 활동이 렌더 프로덕트 attach였고 파이썬 스레드는 전부 idle 이었다. 전체 GUI 앱에서 다시 겪으면 `--phase 3` 으로 조작 계열만 돌릴 수 있다.
>
> **`frameSkipCount` deprecation 은 그대로 둔다.** 경고가 안내하는 대체 수단 (`omni:sensor:tickRate` + `frameSkipCount=0`)을 실측했는데 **듣지 않았다** — `tickRate=5.0` 인데 5 Hz 가 아니라 9.9 Hz 가 나왔다(매 프레임 발행으로 파이프라인 포화). Isaac 트리에서도 그 속성은 라이다·음향 센서에만 쓰이고 `UsdGeom.Camera` 에는 배선돼 있지 않다.
>
> **② 로봇 배치가 자동화돼 있지 않았다 — 해소.** §7 은 "asset browser 에서 올린다"는 수동 단계로만 적어 두었고 경로도 없어 스크립트만으로는 스테이지를 재현할 수 없었다. `scripts/isaac/sim_side/load_robot.py` 를 만들어 GUI·헤드리스가 **같은 목록**을 돌린다. 6.0 경로는 `{assets_root}/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd` 다 (5.x 의 `/Isaac/Robots/Franka/franka.usd` 에서 재편됐다).
>
> **③ 기본 자산은 튜닝이 안 돼 있다.** `tune_drive.py`(τ) 와 `tune_grasp.py`(마찰)를 돌리지 않으면 Phase 1 τ 가 177 ms 이고 Phase 6 은 블록이 미끄러진다. 기동 순서에 넣어야 한다.
>
> **④ 6.0 이 예고한 API 이동 셋** (아직 동작하나 경고):
> `ROS2PublishJointState.targetPrim` → `IsaacReadJointState` 연결,
> `ROS2PublishTransformTree.targetPrims` → `OgnIsaacComputeTransform` 연결,
> `frameSkipCount` → `omni:sensor…` 발행 주기.
>
> **⑤ `is_check_phase3.py` 가 낡아 있었다** — `isaac_scene.json` 의 물체 전부를 기대해서 정상 동작이 `누락: table` 로 나왔다. `/scene/objects` 가 조작 대상만 싣게 된 2026-09-01 결정이 반영 안 된 것이라 검사 쪽을 고쳤다.
>
> **⑦ 시뮬레이터를 재시작하면 ROS 스택도 재시작한다 — 그리고 런치를 죽여도 노드는 남는다.** Isaac 을 다시 띄우면 sim 시계가 0 으로 돌아가는데, 살아 있던 스택의 TF 버퍼에는 **더 큰 타임스탬프의 옛 데이터**가 남아 새 데이터가 `TF_OLD_DATA` 로 버려진다. 증상은 `/scene/objects` 가 **이전 실행에서 물체를 놓았던 자리**를 계속 말하는 것이다 — 실측에서 스테이지는 z=0.425 인데 토픽은 z=0.524(직전 Phase 6 가 들어올린 높이)를 냈고, Phase 6 접근 목표가 10 cm 어긋나 파지가 실패했다. `ros2 run tf2_ros tf2_echo panda_link0 block_a` 로 **TF 를 직접 보면** 갈린다.
>
> 더 고약한 것은 **`ros2 launch` 프로세스를 죽여도 자식 노드가 살아남는다**는 점이다. 스택을 다시 띄우면 `isaac_scene_state` 나 `gripper_action_node` 가 **둘**이 되고, 옛 것이 오염된 버퍼로 계속 발행한다. 액션 서버가 둘이면 검사 로그에 `There may be more than one action server` 가 나온다. 정리는 `ros2 node list | sort | uniq -c` 로 중복을 먼저 확인하고, 정리는 [`scripts/kill_stack.sh`](../../../scripts/kill_stack.sh) 로 한다 — 이 네 종류를 한 번에 걷어낸다 (`--dry-run` 으로 대상부터 본다). 펑션베이의 `/ee_pose` 정지 함정과 **같은 종류**다 (CLAUDE.md).
>
> **⑥ 헤드리스는 실시간보다 빨리 돈다** — 렌더링이 없어 배속 2.2 가 나왔고 Phase 0 `/clock` 검사가 그것을 잡았다. 물리 dt(1/60)에 맞춰 업데이트를 60 Hz 로 묶으면 0.97 이 된다. 검사 자체는 정상 동작한 셈이다.

각 단계는 **이전 단계의 수용 기준을 회귀로 계속 돌린다.** 그리퍼를 붙였다가 팔 관절 순서가 밀리는 종류의 사고를 그 자리에서 잡기 위해서다.

### Phase 0 — 골격 (로봇은 움직이지 않는다)

| | |
|---|---|
| 켜는 것 | 기본값 그대로 (`enable_*` 전부 false) |
| Isaac 쪽 | Panda USD + OmniGraph: `ROS2Context` · `ROS2PublishClock` · `ROS2PublishJointState`(팔 7관절) |
| 우리 쪽 | `panda_isaac.launch.py` — `static_transform_publisher` · `robot_state_publisher` · `readiness_gate` → (게이트 통과 후) `move_group` · `rviz` · `ee_pose` |
| 검증 | `scripts/isaac/is_check_phase0.py` |

**수용 기준**

1. `/clock` 이 흐르고 **배속이 1.0 근처**다
2. `/joint_states` 가 목표 주파수로 오고 **관절 이름 7개가 URDF 와 정확히 일치**한다
3. `move_group` · `rviz` · `ee_pose` 의 `use_sim_time` 이 **true** 다
4. TF 트리가 `world → panda_link0 → … → panda_hand` 로 성립한다
5. named target `extended` **계획이 성공**한다 (실행은 하지 않는다)

3번을 수용 기준에 넣은 이유: `use_sim_time` 이 한 노드라도 어긋나면 `move_group` 이 `Failed to fetch current robot state` 로 조용히 무력해진다. 증상이 원인을 가리키지 않는 대표적인 함정이라 **첫 단계에서 못을 박는다.**

5번까지 통과하면 **설계 문서가 "Isaac 백엔드의 핵심 계약 항목"으로 꼽은 URDF↔USD 관절 이름·순서 정합이 확인된 것**이다.

### Phase 1 — 팔 관절 명령

| | |
|---|---|
| Isaac 쪽 | `setup_graph.py` 의 `PHASE = 1` — `ROS2SubscribeJointState`(`/isaac/arm_command`) → `IsaacArticulationController` |
| 우리 쪽 | 새 노드 없음 — `mode='jtc'` 클라이언트가 전부다. JTC 가 보간하고 `TopicBasedSystem` 이 같은 토픽으로 내보낸다. `is_backend.create_arm_client()` 가 만든다 |
| 검증 | `scripts/isaac/is_check_phase1.py` |

**수용 기준**

| 항목 | 기준 |
|---|---|
| named target 도달 | `extended` 계획 → 스트리밍 → 목표 오차 ≤ 0.005 rad |
| **정착 오차** | 자세 3종(`ready`/중간/펴짐)에서 **자세 의존성이 없을 것** |
| **시정수 τ** | 측정해 기록. 필요하면 drive `stiffness`/`damping` 튜닝 |
| dead time | 측정해 기록 |

**펑션베이의 교훈을 앞당겨 적용하는 지점이다.** 거기서는 처짐과 응답 지연을 §3·§6 에 가서야 쟀고, 원인인 제어기 강성을 우리가 못 고쳐 벤더 요청서까지 갔다. Isaac 은 articulation drive 의 `stiffness`/`damping` 을 **우리가 직접 조정할 수 있으므로**, 같은 값을 Phase 1 수용 기준에 넣어 그 자리에서 닫는다.

> **실측 (2026-09-05)** — `phase1` **5/5**. τ **35.5 ms** · dead time **80.3 ms** · 정착 오차 **0.0000 rad**(`ready`) / 0.0001 rad(`stretch`) · 오버슈트 **0.0%**.
>
> **한동안 τ 가 미달이었는데 로봇이 아니라 측정 방법 탓이었다.** 계단 입력을 주려면
> 도달 시간 0 을 줘야 하는데 JTC 가 그것을 거부해서 `is_backend.MIN_REACH_SEC` 를 대신
> 쓴다. 그 상수가 **τ 기준과 똑같은 50 ms** 였으므로 잰 값 52.6 ms 는 사실상
> 「상수 + 2.6 ms」였다 — 로봇이 아무리 빨라도 통과할 수 없는 항목이었다.
> `controller_manager` 의 update 주기(100 Hz = 10 ms)가 어차피 하한이라 상수를
> **10 ms 로 낮췄고**(§8 Q13), 그 뒤 재실행에서 **5/5** 가 나왔다.
>
> **중력 처짐은 사실상 없다.** 펑션베이의 0.0983 rad 과 300배 차이라, 벤더 요청서 A-1에 해당하는 문제가 이 백엔드에는 없다. drive gain 은 `tune_drive.py` 가 `τ = B/K` 로 **역산해서** 정한다 — 시행착오가 아니다 (그 스크립트 docstring 에 유도가 있다).

### Phase 2 — 그리퍼

| | |
|---|---|
| 켜는 것 | `enable_gripper:=true` |
| 우리 쪽 | 새 노드 없음 — ros2_control 의 `panda_hand_controller` 가 액션 서버다. `enable_gripper:=true` 로 `GripperNode` 와 함께 뜬다 |

**수용 기준** — open/close 왕복, `/joint_states` 에 finger 포함, `<mimic>` 처리 확인
(`panda_finger_joint2` 는 URDF mimic 이라 MoveIt 이 채운다), planning scene 조회로
두 finger 값 일치.

> **액션 이름이 백엔드 경계다.** 상위 경로(teleop · robot twin · 데이터셋 재생)는
> 모두 `GripperNode` 를 거치고, 그 노드는 `/panda_hand_controller/gripper_cmd`
> **액션**을 부른다. mock 도 Isaac 도 그 서버는 ros2_control 의
> `GripperActionController` 이므로 **상위는 백엔드가 무엇인지 몰라도 된다.**
>
> 시간 안에 목표에 닿지 못하면 **실패가 아니라 `stalled`** 로 보고한다 — 물체를 쥐어
> 더 닫히지 않는 상태가 정상 동작이기 때문이다.

> **실측 (2026-09-05)** — `phase2` **6/6**. 같은 날 수집 실행에서 폭이 열림
> **0.0700 m** / 파지 **0.0448 m** 였고, 파지 시 손가락 effort **22.4 N·m** 로
> `stalled`·`at_goal` 이 선다.

### Phase 3 — scene 객체

| | |
|---|---|
| 켜는 것 | `enable_scene:=true` |
| Isaac 쪽 | 물체 pose 를 **표준 타입**(TF 또는 `PoseArray`)으로 발행 |
| 우리 쪽 | `isaac_scene_state_node` — 표준 타입 → `rdfp_msgs/SceneObjects` 변환 |

D2 에 따라 변환은 **우리 쪽에서** 한다. 좌표계·단위·쿼터니언 순서를 맞추는 책임은
전부 이 노드에 있고, 순수 함수는 `robot_control/scene/pose_math.py` 를 재사용해 ROS
없이 테스트한다.

**수용 기준** — Isaac 에서 물체를 옮기면 `/scene/objects` 에 반영, 좌표계·쿼터니언
순서 검증, `mock_scene_state_node` 와 동일한 메시지 계약.

> **실측 (2026-09-05)** — `phase3` **5/5**. 같은 날 수집 실행의 rosbag 에
> `/scene/objects` 가 **1.82 Hz** 로 담겼다.

### Phase 4 — 카메라

| | |
|---|---|
| 켜는 것 | 없음 — Isaac 그래프(`PHASE>=4`)가 발행한다. 보기·녹화는 수집 계층의 몫 |
| Isaac 쪽 | RTX 카메라 → `Image` + `CameraInfo` |

**수용 기준** — 발행 주파수, encoding, **`header.stamp` 이 sim time 으로 채워질 것**,
recorder 가 mp4 를 만들 것, **VRAM 사용량 관측**.

**이 단계는 이 PC(RTX 4060 8GB)에서 가장 위험하다** — §5. Ubuntu 22.04 머신에서
수행하는 것을 전제로 잡는다.

---

> **실측 (2026-09-05)** — `phase4` **6/6** (내용 확인 포함). 같은 날 수집 실행의
> rosbag 에 `/isaac/camera/image_raw` 가 **4.50 Hz** 로 담겼다.
>
> **스탬프가 sim time 으로 채워진다** — 펑션베이는 이 값이 0 이라 이미지를 시간축에
> 놓지 못했고(그 문서 §6.5) 데이터셋 적재가 통째로 깨졌다. 이 백엔드에는 그 문제가 없다.

### Phase 5 — **철회됨 (2026-09-01)**

`attach`/`detach` 로 파지 중 충돌을 다루려 했으나 채택하지 않았다. 무너진 전제 셋은
**§8 Q9**, 그 위의 planning scene 결정은 **Q8** 에 있다. `is_check_phase5.py` 도
`setup_graph.py` 의 `PHASE = 5` 도 없다 — **번호만 비워 둔다** (뒤 단계를 당기면
`is_check_phase6.py` 부터 이름이 어긋난다).

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

**유리하게 맞아떨어지는 것 하나** — `GripperNode` 는 목표 미도달을 실패가
아니라 `stalled` 로 보고한다. 물체를 문 상태가 정확히 그것이므로 **파지 성공이 액션
실패로 뒤집히지 않는다.** Phase 2 를 위해 그렇게 만든 판정이 여기서 그대로 맞는다.

**`block_a` 를 집는다.** 다른 물체 `cylinder_a` 는 실린더라 판정 산식(`BLOCK_HALF_WIDTH`
= 0.025 인 정육면체)이 그대로 맞지 않는다. 그것이 갖고 있는 z축 45° 회전은 Phase 3 의
쿼터니언 규약(ROS xyzw ↔ USD wxyz) 검증용이므로 건드리지 않는다 — **실린더는 z축
대칭이라 그 회전이 물리적으로 무의미해서**, 예전 `block_b` 처럼 평행 조에 모서리를
물려 파지 판정을 흐리는 부작용 없이 검증만 남는다.

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

> **실측 (2026-09-05)** — `is_check_phase6.py` **6/6 통과** (수집 실행 안에서).
>
> ```
> 1. 기준 자세  ready 관절오차 0.0003 rad, 그리퍼 폭 0.0700 m, 충돌없음 True
> 2. 접근·하강  목표 (0.500,-0.150,0.528) 오차 0.0005 m, 손 기울기 0.0도
> 3. 물림       폭 0.0448 m (기준 0.03~0.07, 블록 한 변 0.05), stalled=True at_goal=True
> 4. 들어올림   블록 z 0.4256 -> 0.5254  (+0.0999 m, 명령 +0.100, 기준 >= 0.07)
> 5. 유지(3초)  z 0.5254 -> 0.5254  (낙하 0.0000 m, 기준 <= 0.02)
> ```
>
> **물림 폭 0.0448 이 블록 한 변 0.05 보다 5.2 mm 작다.** 손가락이 그만큼 파고든
> 것이며, 접촉 강성이 유한하다는 뜻이다. 파지력은 stiffness × 오차이므로 **이 상태가
> 곧 힘이 실린 상태다** — 정확히 0.05 에서 멈췄다면 오히려 힘이 0 이다.
>
> **같은 날 첫 실행은 3/4 였다.** 검사가 `/panda_hand_controller/gripper_cmd` 액션을
> 직접 불렀는데 **파지에서는 그 액션이 완료되지 않는다**(§7). production 이 쓰는
> `gripper_cmds`/`gripper_states` 계층으로 옮기자 통과했다 — 로봇이 아니라 검사가
> 틀린 계층을 때리고 있었다 (§8 Q14).
>
> `panda_finger_joint2` 에는 drive 가 없다(mimic). **한쪽 drive 만으로 파지가 성립하는
> 것을 확인했으므로 그대로 둔다.**

### Phase 7 — 수집 계층 연동

Phase 0~6 은 제어 계층만 다뤘다. 여기서 그 위에 수집 계층을 얹어 **에피소드가
실제로 기록되는지** 확인한다.

| | |
|---|---|
| Isaac 쪽 | **없음** |
| 우리 쪽 | `robot_control/launch/panda_isaac.launch.py` + `rdfp/launch/rdfp_collect.launch.py backend:=isaac` (묶음 launch 는 2026-09-13 삭제) |
| 검증 | rosbag2 기록 → `_detect_all_episodes` 로 분절 확인 |

**카메라 설정의 출처가 다른 백엔드와 다르다.** 다른 launch 는 `image_pipeline.yaml`
을 읽지만 Isaac 은 이미지를 카메라 노드가 아니라 **시뮬레이터가 직접** 발행한다.
해상도·주파수·토픽을 정하는 쪽이 Isaac 의 렌더 프로덕트이므로, `isaac_scene.json` 의
`camera` 블록을 launch 가 그대로 읽어 뷰어·레코더의 기본값으로 쓴다. 해상도를 바꾸려면
JSON 한 곳만 고치고 **`colcon build` + `setup_graph.py` 재실행**을 하면 된다.

**arm 명령 채널은 `joint_state` 다.** Isaac 이 `/isaac/arm_command` 로
`sensor_msgs/JointState` 를 받으므로 `target_joint_cmds_publisher` 의 `source` 가
`joint_state` 이고, 이 경로는 **컨트롤러의 `joints` 파라미터를 조회하지 않는다** —
`TopicBasedSystem` 이 내는 `JointState` 에 이름이 이미 실려 있기 때문이다 (§8 Q16).

> **실측 (2026-09-05)** — 당시의 `rdfp_panda_isaac.launch.py` 로 **세션 → `/scene/reset` →
> 에피소드 → 집기 → rosbag → mp4 → DB 적재**를 통과시켰다. 집기는 `is_check_phase6.py`
> 를 그대로 썼다 — 이미 검증된 실제 조작이라 에피소드 안에서 팔·그리퍼·scene 이 모두
> 움직인다.
>
> ```
> rosbag   331 MB, metadata.yaml 정상, split 1개 / 토픽 8개
>            /joint_states           4298   53.8 Hz
>            /target_joint_cmds      2004   51.5 Hz
>            /ee_pose                3718   44.9 Hz
>            /gripper_states          732    9.0 Hz
>            /isaac/camera/image_raw  372    4.5 Hz
>            /scene/objects           148    1.8 Hz
>            /session                   5
>            /gripper_cmds              1
> scene    /scene/reset applied_count=1, 블록이 지정 좌표로
> import   에피소드 1건 · joint_states 2174 · pose_stampeds 968
>            gripper_states 193 · image_frames 96 · mp4 1 · warnings 0
> DB       success=t, metadata 보존, 길이 19.3 sim-초
> ```
>
> **`/target_joint_cmds` 가 51 Hz 조밀한 스트림으로 담겼다.** action 채널을 계획
> (`joint_trajectory`)이 아니라 `TopicBasedSystem` 이 내는 보간값에서 뽑기로 한 결정
> (§8 Q16)의 실측 확인이다. mock 의 JTC 경로에서는 이 토픽이 **0 건**이다 — MoveIt
> 액션이 컨트롤러 안에서 끝나 토픽에 아무것도 흐르지 않기 때문이다. 즉 **action 채널이
> 채워지는지는 백엔드가 결정한다.**
>
> **타임스탬프가 sim time 이다.** `use_sim_time` 이 수집 계층까지 전파됐다는 뜻이며,
> 벽시계였다면 에피소드 경계가 시뮬레이터 시간과 어긋나 재생이 불가능해진다.
> 관련 함정: **`header.stamp` 을 안 채운 메시지는 rosbag 에는 남고 데이터셋에는
> 안 들어간다** — 적재가 그 값으로 에피소드 창을 거르므로 시각 0 은 창 밖이다.
>
> **녹화 토픽 목록은 백엔드마다 다르다.** `config/recording_topics.list` 는 mock 기준
> (`/camera/image_raw`)이라 Isaac 에 그대로 쓰면 **이미지가 한 장도 안 담긴다** —
> rosbag2 는 없는 토픽을 조용히 건너뛰고, mp4 레코더는 launch 가 remap 해 주므로 증상이
> "mp4 는 있는데 rosbag 에 이미지가 없다"로만 보인다. `config/recording_topics_isaac.list`
> 를 `RECORDING_TOPICS_FILE` 로 고른다. `dataset_config.yaml` 의 `topics_file` 도 같은
> 파일을 가리켜야 한다.
>
> **`set_task_label` 은 `IN_EPISODE` 중에 거부된다** — 에피소드를 시작하기 **전에**
> 설정해야 한다. 놓치면 라벨이 빈 문자열인 채로 기록되고, **재적재로는 못 고친다.**

### Phase 8 — 테스트와 토폴로지 계약

Phase 0~7 의 검증은 전부 **수동 체크 스크립트**였다. 사람이 Isaac 을 띄우고 Play 를
눌러야 돌아가므로 `colcon test` 에는 잡히지 않는다. 여기서 그 아래를 자동 테스트로
받친다.

| | |
|---|---|
| 대상 | 새로 만든 노드 3종 + Isaac 쪽 스크립트 9개의 경로 계약 |
| 검증 | `colcon test --packages-select robot_control` (Isaac 불필요) |

> **실측 (2026-09-05)** — `robot_control` **289건 수집 / 288 통과 · 1 skip · 실패 0**.
> (참고: `robot_twin` 235 · `rdfp` 402 — 워크스페이스 합 **926**.)
>
> | Isaac 관련 스위트 | 수 | 무엇을 지키나 |
> |---|--:|---|
> | `isaac/tests/test_scene_state_node.py` | **24** | TF→SceneObject, 쿼터니언 무변환, 누락 물체 |
> | `isaac/tests/test_servo_command_bridge_node.py` | **19** | 배열 순서·stamp·NaN 방어 |
> | `tests/test_isaac_sim_side_scripts.py` | **57** | 어느 배포에서도 통하는 경로 해석 |
> | (`isaac/tests` 합계 43 · `tests` 합계 163) | | |
>
> **지금 환경에서는 `-p no:anyio` 가 필요하다.** pytest 6.2.5 에 `anyio` 플러그인이
> `from _pytest.scope import Scope` 를 하는데 그 모듈이 없어, **수집이 시작되기도 전에**
> `ModuleNotFoundError` 로 죽는다. 테스트가 깨진 것처럼 보이지만 한 건도 안 돌았다.
>
> **테스트가 실제 버그를 하나 잡았다.** 두 노드의 경고 스로틀이
> `now - _logged.get(key, 0.0) >= interval` 형태라 **`now < interval` 인 동안 첫
> 경고를 삼킨다.** Isaac 은 Stop 마다 sim time 이 0 으로 되돌아가므로 기동 직후
> 5 초가 정확히 그 구간이고, 하필 "TF 가 안 온다"를 가장 보고 싶은 때다. 기본값을
> `-inf` 로 바꿔 첫 번째는 반드시 남게 했다.
>
> **대역을 만들다 배운 것 둘.**
>
> `_execute` 의 루프 조건이 `rclpy.ok()` 인데, `rclpy.init()` 을 부르지 않은
> 프로세스에서는 False 다. 그러면 루프가 한 번도 안 돌아 **어떤 목표를 줘도
> `reached_goal=False`** 가 나오고, 테스트는 통과하는 것처럼 보이면서 아무것도
> 검증하지 않는다.
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
| 우리 쪽 | `config/robot_twin_panda_isaac.yaml` — `move_group_mode: jtc` 하나로 끝난다. 명령 채널을 안 준다 |
| 검증 | 트윈을 띄우고 REST 로 팔·그리퍼·세션을 실제로 움직인다 |

**명령 채널을 설정으로 뚫는 길이 이 단계에서 생겼다.** `runtime.py` 가
`create_move_group_client(node, mode=mode)` 만 부르고 있어, 시뮬레이터가 직접
`JointState` 를 받는 백엔드로는 내보낼 방법이 없었다 — **토픽 remap 으로는 못 고친다.
메시지 타입이 다르다.**

그래서 설정에 짝 검사를 넣었다. `jtc` 에 이 키들을 주면 거부하고(그 모드는 보지 않으므로
조용히 무시되는 키가 생긴다), `joint_state` 형식에 `arm_command_joint_names` 가
없으면 거부한다(조회할 컨트롤러가 없어 첫 스트리밍에서 멈춘다).

> **Isaac 은 그 키를 쓰지 않는다** — JTC 라 MoveGroup/ExecuteTrajectory 액션으로
> 실행된다. 짝 검사 자체는 그대로 유효하다
> ([`config.py`](../../../src/robot_twin/robot_twin/config.py) 의 `_check_arm_command`) —
> `jgpc` 를 쓰는 펑션베이·`panda_mock_jgpc` 설정이 그 검사를 탄다. 지금은 **Isaac yaml
> 에 그 키가 생기면 거부되는 것**이 안전장치다.

> **실측 (2026-09-05)** — 트윈 REST 로 **집기 전 과정**이 돌았다.
>
> ```
> open -> move_linear -> grasp -> lift    전부 COMPLETED
>                                          블록이 +122 mm 따라 올라옴
> move_gripper_to_target grasp             통과 (빈손이면 FAILED/TIMEOUT 으로 갈린다)
> 배포 설정 계약 (Isaac 없이 검사)          14건 통과
>   move_group_mode: jtc · 명령 채널 없음 · use_sim_time: true
>   오퍼레이션 목록이 mock 과 동일 (reset_scene 포함)
> ```
>
> **`move_gripper_to_target grasp` 이 통과하는 것이 이 백엔드의 특징이다** — mock 에서는
> 타임아웃한다. mock 은 손가락에 effort state interface 가 없어 `stalled` 을 판정할 수
> 없고, `grasp` 의 `at_goal` 이 영영 서지 않기 때문이다.
>
> **트윈 설정이 비어 있는 것이 계약이다.** `arm_command_*` 에 값이 생기면 누군가 옛
> bridge 설정을 되살린 것이므로, 짝 검사가 그 조합(`jtc` + 명령 채널)을 **설정 로드에서
> `ValidationError` 로 거부한다** — 조용히 어긋나지 않고 기동 자체가 실패한다.
>
> **여기서 제어 계층의 버그를 하나 찾았다.** `plan_joints_async` 가 `externally_spun`
> 을 **받지도 넘기지도 않아서**, executor 가 다른 스레드에서 노드를 돌리는 트윈에서는
> `_complete_joint_values` 가 노드를 직접 spin 하려다 영원히 멈췄다. 콜백은 executor
> 쪽으로 가므로 여기서는 영영 오지 않는다 — **예외도 타임아웃도 없이 오퍼레이션이
> RUNNING 인 채로 남는다.** `plan_named_target_async` 는 관절값을 콜백 체인으로 얻어
> 애초에 spin 하지 않으므로 멀쩡했고, 그래서 `move_to_named_target` 만 먼저 동작해
> 원인이 더 헷갈렸다. 회귀는 `moveit/tests/test_externally_spun_propagation.py` 5건
> (2026-09-05 통과 확인).
>
> **트윈은 시작 자세 충돌에서 스스로 빠져나올 수 없다.** 모든 팔 오퍼레이션이 MoveIt
> 계획을 거치는데, 시작 자세가 충돌이면 계획 자체가 거부된다(§7 시작 자세). 계획을
> 거치지 않는 복구 경로가 `scripts/isaac/is_recover.py` 다.

### Phase 10 — servo(twist) 경로

**두 텔레오퍼레이션 경로가 모두 servo 로 수렴한다.** 그래서 이 경로가 막히면 Isaac 을
손으로 몰 수 없고, 손으로 못 몰면 파지 에피소드를 모을 수 없다 — 이 프로젝트의 목적이
바로 그것이므로 Q3 은 "있으면 좋은 것"이 아니었다.

```
teleop_keyboard  -> delta_twist_cmds ─┐
teleop_retarget  -> ee_twist_node ────┴─> servo -> JTC -> /isaac/arm_command
```

| | |
|---|---|
| Isaac 쪽 | **없음** |
| 우리 쪽 | **없음** — servo 가 기본 경로(`JointTrajectory` → `panda_arm_controller`)로 나가고 `TopicBasedSystem` 이 그 뒤를 잇는다. `panda_isaac.launch.py` 의 `enable_servo` 로 `servo_node` · `servo_auto_start` 둘만 띄운다 |
| 검증 | `enable_servo` 기동 검사. servo 파라미터 오버라이드가 없으므로 launch 쪽 회귀 대상도 없다 |

**변환 노드가 필요 없어진 것이 ros2_control 을 얹어 얻은 것 중 하나다.** 그 전에는
servo 의 출력 타입(`JointTrajectory` / `Float64MultiArray`)과 Isaac 의 입력
(`ROS2SubscribeJointState`)이 어느 쪽도 상대를 못 내서 `servo_command_bridge` 를
끼워야 했다. 그 노드는 [펑션베이 전용으로 남았고](../../../src/robot_control/robot_control/isaac/servo_command_bridge_node.py),
`publish_joint_velocities: false` 같은 servo 파라미터 오버라이드도 그쪽으로 옮겨 갔다.

> ## 🔶 Phase 10 — **오늘(2026-09-05) 실기 실측이 없다**
>
> **servo 를 태운 실행이 오늘 없었다** — 수집 실행의 집기는 MoveIt 카테시안이고 검사
> 7종에도 servo 항목이 없다. 아래 수치는 전부 **2026-08-30**, 그것도 servo 출력이
> 다리 노드를 거치던 시절 것이다. 지금은 JTC 로 직행하므로 **NaN 방어가 어디에도
> 없다** (§8 Q20, §7).
>
> **구성만 오늘 확인했다 (정적).** `enable_servo` 기본 `true` 로 `servo_node` +
> `servo_auto_start` **둘만** 뜨고 다리 노드가 없다. servo 파라미터는
> `build_servo_params()` 그대로다 — `panda_mock` 과 같고, 재정의가 필요한
> `panda_functionbay` / `panda_mock_jgpc` 과 갈리는 지점이다.
>
> **입력은 m/s 가 아니다.** `command_in_type: unitless` 이고 `scale.linear: 0.4`,
> `publish_period: 0.034` 이다. 즉 `linear.z = 0.05` 는 0.02 m/s 를 뜻한다 —
> 처음에 m/s 로 읽어 "명령보다 훨씬 덜 움직인다"고 오독했다.
>
> **scene 을 켜도 정상이다.** 매 시행 전에 `ready` 로 되돌리고 planning scene 내용만 바꿔
> 비교하면 `diff 없음 / 빈 diff / table / table+블록3` 넷이 모두 `NO_WARNING` 이고
> 이동량도 같다(0.18~0.20 m). 처음에는 scene 을 원인으로 지목했는데 **틀렸다** — 재기동
> 과정에 자세 초기화가 섞여 든 것이었다(§7 servo 드리프트).

| 항목 | 실측 | 상태 |
|---|---|---|
| 2초 구간 추종 배율 | 기대 대비 0.46~0.49 (scene on/off 무관) | 필터 램프업 + 개루프 지연이 후보. **분리 못 했다** |
| 반복 주행 시 정지 | 누적 드리프트 | §7 servo 드리프트. 세션 사이 `is_recover.py` |
| NaN 출력 | 7축 전부 NaN, status 는 `NO_WARNING` | **닫힘 (2026-09-05).** 사슬에 필터는 없지만 **Isaac 이 흡수하고 스스로 복구한다** — §8 Q20 |
| `ready` 에서 `DECEL_COLLISION` | 팔이 탁자와 실제 충돌한 뒤 발생 | **미규명.** `/check_state_validity` 는 충돌 없음으로 본다 |
| `teleop_keyboard` 실주행 | — | **미검증** (지금까지는 raw twist 로만 확인) |


### Phase 11 — ros2_control 연동 — **실측 완료 (2026-09-05)**

`topic_based_ros2_control/TopicBasedSystem` 으로 `controller_manager` 를 ROS 쪽에서 돌린다.
launch 는 `panda_isaac.launch.py`, 시뮬레이터는 `run_isaac_sim.sh --gui` (또는 `--headless`).

**다리 노드도 servo 파라미터 재정의도 없다.** servo 는 기본 경로
(`JointTrajectory` → `/panda_arm_controller/joint_trajectory`)로 나가고, 그리퍼는
`panda_hand_controller` 가 받는다.

#### 토픽 — Isaac 의 관절 상태는 비켜선다

| 채널 | 토픽 |
|---|---|
| Isaac 의 관절 상태 | **`/isaac_joint_states`** |
| `/joint_states` | `joint_state_broadcaster` 가 낸다 |
| 팔 명령 | `/isaac/arm_command` |
| 그리퍼 명령 | `/isaac/gripper_command` |

**Isaac 의 관절 상태를 `/joint_states` 에 그대로 두면 `TopicBasedSystem` 이 자기 출력을
되읽는 고리가 생긴다.** 명령 토픽을 팔·손으로 나눌 수 있는 것은 xacro 의
`<ros2_control>` 이 둘로 갈려 있어 시스템마다 `joint_commands_topic` 을 따로 주기
때문이다.

연동 방식이 하나뿐이므로 고를 것이 없다 — `setup_graph.py` 와 `is_backend.py` 가 같은
토픽 이름을 못박는다.

#### 실측 (Isaac 6.0.1 · RTX 4000 Ada · headless)

| 항목 | 값 |
|---|---|
| 컨트롤러 | `joint_state_broadcaster` · `panda_arm_controller`(**JTC**) · `panda_hand_controller`(GripperActionController) 전부 active |
| 정착 오차 | 최악 **0.0003 rad** (`ready`) / 0.0001 rad (stretch) |
| 카테시안 접근 | 오차 **0.2 mm**, 손 기울기 0.0° |
| 손가락 effort | `TopicBasedSystem` 이 그대로 넘긴다 (파지 시 **22.4 N·m**) |
| 트윈 REST | `open → move_linear → grasp → lift` 전 과정 COMPLETED, 블록이 **+122 mm 따라 올라옴** |
| `move_gripper_to_target grasp` | **통과** — mock 에서 타임아웃하던 그 연산이다. 빈손은 `FAILED/TIMEOUT` 으로 갈린다 |
| 검사 | **7종 전부 통과** — phase0 5/5 · phase1 5/5 · phase2 6/6 · phase3 5/5 · phase4 6/6 · phase6 6/6 · `is_recover` 복구 완료 (런북 회귀 실행) |

#### ros2_control 을 얹는 대가 — 실측 (2026-09-05)

**공짜가 아니다.** 같은 머신·같은 씬에서, `controller_manager` **없이** 토픽만으로 잇던
구성을 대조군으로 잰 값이다.

| | Isaac 단독 | + CM 없는 스택 | + 지금 스택 |
|---|---|---|---|
| `/clock` 배속 | 0.989 | **0.989** | **0.937** |
| load average | 2.5 | 2.5 | **8.4** |
| dead time (`phase1`) | — | **30 ms** | **131 ms** |
| τ (`phase1`) | — | 35.3 ms | 35.8 ms |

`controller_manager` 의 실시간 루프 + JTC 가 **배속 −5% · load 3배 · dead time +100 ms**
를 얹는다. τ 는 사실상 같으므로 **추종 정확도가 아니라 지연과 부하가 대가다.**
그래도 정착 오차는 지금 쪽이 더 좋았다(`ready` 에서 0.0000 vs 0.0003 rad).

**부하의 출처는 Isaac 이 아니라 ROS 쪽이다** — 씬도 물리 계산량도 같은데 스택이 CPU 를
가져간다. 대조군이 load 2.5 로 배속을 전혀 안 깎는 것이 그 근거다.

#### 수집 전 과정 — 통과 (2026-09-05)

당시의 `rdfp_panda_isaac.launch.py` 로 **세션 → `/scene/reset` → 에피소드 → 집기 → rosbag →
mp4 → DB 적재**가 한 번에 통과했다. **수치와 함정은 Phase 7 실측에 있다** — 수집 계층
연동이 그쪽 주제다.

**검사는 계약 계층에 있다.** `phase6` 를 옮긴 근거는 삭제 직전 두 방식에서 나란히 돌려
**같은 값이 나온 것**이다 — 열림 0.0700 / 물림 0.0448 / 들어올림 +0.0999 vs +0.0998 /
낙하 0.0000. 검사가 보는 것이 연동 방식이 아니라 물리 결과라는 뜻이고, 그것이 §0 D1″ 에서
연동 방식을 하나로 줄인 근거이기도 하다.

> `phase0` 의 「계획」 항목이 **간헐적으로** `/move_group/get_parameters` 30 초
> 타임아웃을 낸다. 재실행하면 통과하고, `is_probe_srdf.py` 로 재 보면 SRDF(9,314 B)
> 조회는 네 조건 모두 1 초 안에 응답한다 — 스택 기동 직후 DDS 디스커버리가 덜 앉은
> 상태로 보인다. **연동 방식과 무관하며 예전부터 있던 것이다.**

**한때 미달이던 셋은 전부 해소됐다 — 셋 다 로봇이 아니라 검사 쪽 문제였다.**

| 항목 | 무엇이었나 | 어떻게 닫혔나 |
|---|---|---|
| `phase0` 배속 0.872 | 부하 (load average 7.7 에 Isaac + MoveIt + 검사가 함께 돌았다) | 조용한 재실행에서 **0.9355** (허용 0.90~1.10). 다만 부하와 단조롭지 않은 점은 §8 Q19 로 남았다 |
| `phase1` τ 52.6 ms | **측정 하한이 기준과 같았다** — `MIN_REACH_SEC` 50 ms + 2.6 ms | 상수를 10 ms 로 (§8 Q13) → **35.5 ms** |
| `phase6` 물림 | **검사가 틀린 계층을 때렸다** — `gripper_cmd` 액션은 파지에서 완료되지 않는다 | `gripper_cmds`/`gripper_states` 계약 계층으로 이전 (§8 Q14) → **6/6** |

---


---

## 닫힌 결정 (이력) — isaac_backend.md §8 에서 옮김

| # | 항목 | 언제 정하나 |
|:-:|---|---|
| ~~Q1~~ | **닫힘 (2026-08-29)** — Isaac 은 `name` 을 채우고 팔 7관절 + finger 2관절을 모두 보낸다. `joint_state_fusion` 을 **제거**했다. 남겨 두면 `extra_joint_*` 주입을 끌 수 없어(빈 리스트는 기본값으로 되돌아간다) `panda_finger_joint1` 이 중복 발행된다 | — |
| ~~Q2~~ | **닫힘 (2026-08-29)** — **TF** 로 확정. Isaac 이 `ROS2PublishTransformTree` 를 기본 제공하고, 좌표 변환·쿼터니언 규약을 tf2 가 대신한다. Phase 3 실측으로 검증됐다 | — |
| ~~Q3~~ | **닫힘 (2026-08-30) — 지원한다.** teleop 두 경로(`teleop_keyboard` 의 twist, `teleop_retarget` → `ee_twist_node`)가 **모두 servo 로 수렴**하므로 없으면 Isaac 을 손으로 몰 수 없다. ~~servo 출력을 `sensor_msgs/JointState` 로 옮기는 다리(`isaac_servo_bridge`)를 두고~~ **→ D1″ 이후 다리가 없다** — ros2_control 이 있어 servo 가 기본 경로(`JointTrajectory` → JTC)로 나간다. `enable_servo` 는 기본 `true` 다. 펑션베이에서 무산된 것은 그 백엔드에 그리퍼·컨트롤러가 함께 없었기 때문이며, 타입 문제 자체는 같은 방식으로 풀린다 | — |
| ~~Q4~~ | **닫힘 (2026-09-05)** — 검증 머신 사양을 §5 에 적었다: **RTX 4000 SFF Ada · VRAM 20 GB** · RAM 62 GB · 20 코어 · Ubuntu 22.04.5. Phase 4 는 이 머신에서 640×480 @ 5 Hz 로 통과했고, Phase 11 실측도 전부 여기서 나왔다. 고해상도·고주파가 필요해지면 그때 다시 재면 되는 **조건부 후속 작업**이지 미지수가 아니다 | — |
| ~~Q6~~ | **닫힘 (2026-08-29)** — **실제 Franka 스펙으로 좁혔다.** `config/panda_real_joint_limits.yaml` 을 `panda_isaac.launch.py` 가 넘긴다. 적용 범위는 Isaac 뿐이며, mock/Gazebo/펑션베이까지 넓힐지는 별도 결정 |  — |
| ~~Q7~~ | **철회 (2026-09-02)** — ~~`C:\isaacsim\scenes\panda_rdfp.usd` 에 저장 완료. 다음부터는 열고 Play 만 하면 된다~~ **저장한 스테이지를 쓰지 않기로 했다** — 물체 이름·크기가 USD 와 `isaac_scene.json` 두 곳에 살면 조용히 갈라지고, JSON 은 `isaac_scene_state_node` 도 읽는 정본이다(§6 「저장한 스테이지를 쓰지 않는다」). `verify_saved_scene.py` 는 **저장된 파일이 있을 때의 진단 도구**로 남았다 | — |
| ~~Q8~~ | **닫힘 (2026-08-29) → 축소 (2026-09-01) → 철회 (2026-09-01).** 최종: **planning scene 에 아무것도 넣지 않는다.** `planning_scene_sync` 노드와 `SceneObject.fixture` 필드를 삭제했다. 경위 — 조작 대상을 넣으면 파지가 시작 자세 충돌이 되어 `INVALID_MOTION_PLAN`(-2)이므로 고정물만 넣도록 축소했는데, 이어서 **환경 물체를 `/scene/objects` 에 아예 발행하지 않기로** 정하자 넣을 것이 남지 않았다. 원래 동기("팔이 탁자를 뚫는다", `panda_link4`·`panda_link5` 가 상판을 파고든 것을 실측)는 **단순성을 위해 감수한다**. 되살릴 때는 토픽→scene 노드가 아니라 탁자만 launch 시점에 상자 하나로 정적 등록하는 편이 맞다 | — |
| ~~Q9~~ | **철회 (2026-09-01).** `attach`/`detach` 자체를 없앴다. (1) 근거였던 "rosbag 에 남는다"가 **거짓**이었다 — `/scene/commands` 는 `recording_topics.list` 에 없다. (2) `attach` 는 명시적 detach 까지 유지되는데 **물체는 미끄러져 떨어질 수 있어** 믿음이 조용히 틀린다. 떨어지면 planner 는 손에 있다고 믿으면서 탁자 위의 그것도 못 본다(붙은 물체는 world 동기화에서 제외되므로). (3) 애초에 파지 동작은 cartesian 이라 `avoid_collisions` 기본값 `False` 로 **planning scene 을 보지 않는다** | — |
| ~~Q10~~ | **닫힘 (2026-08-30), 이후 두 번 뒤집힘** — 트윈으로 REST 팔·그리퍼·세션 제어를 실측했다(§2 Phase 9). 다만 ① ~~`arm_command_topic`/`format`/`joint_names` 3종을 설정에 추가~~ **→ D1″ 에서 제거** (JTC 라 명령 채널을 안 준다), ② ~~`reset_scene` 은 원리적으로 불가능해 노출하지 않는다~~ **→ 2026-09-02 에 노출** (Isaac 6.0 의 `simulation_interfaces` 서비스를 `isaac_scene_state_node` 가 대신 부른다). 현재 설정은 `move_group_mode: jtc` 하나이고 `reset_scene` 을 포함한다 | — |
| ~~Q11~~ | **닫힘 (2026-09-05)** — `ros-humble-simulation-interfaces` 1.4.0 을 설치했다. `isaac_scene_state_node` 가 ERROR 대신 `reset: /scene/reset -> /set_entity_state (root '/World/Scene')` 를 찍는 것을 확인했다. **새 머신에서는 매번 필요하다** — 없어도 스택은 정상 기동하고 에피소드를 시작하고 나서야 물체를 못 놓는다는 것을 알게 되므로, 런북 §2 에 선행 조건으로 적어 두었다 | — |
| ~~Q12~~ | **닫힘 (2026-09-05)** — 두 사건이 다르다는 것이 실측으로 갈렸다(§7). 타임라인 Stop/Play 는 `resetOnStop` 이 꺼져 있어 **시계를 되감지 않고 스택이 자동 복구**된다. 프로세스 재시작만 파손을 만들며, 그때는 `kill_stack.sh --ros` + 재기동으로 **완전히 복구된다**(게이트 통과, `TF_OLD_DATA` 0 건) | — |
| ~~Q13~~ | **닫힘 (2026-09-05)** — `MIN_REACH_SEC` 을 **10 ms** 로 낮췄다(`controller_manager` update 주기가 어차피 하한이다). 항목을 빼지 않은 이유는 plugin 사용자에게는 **명령 사슬 전체의 응답**이 재려는 값이기 때문이다. 다만 컨트롤러 보간이 남으므로 **bridge 수치와 직접 비교하면 안 되고**, τ 초과 시 출력이 그 사실을 말한다 | — |
| ~~Q14~~ | **닫힘 (2026-09-05) — 양쪽 모두에 적용했다.** `is_check_phase6.py` 가 `gripper_cmds` 로 심볼을 보내고 `gripper_states` 의 `goal` 일치 + `at_goal` 로 판정한다. 방식 분기가 없다 — production 이 쓰는 경로가 하나이므로 검사도 하나다. 딸려 나온 것 셋: ① 명령이 `close`(위치 0) 에서 **`grasp`** 로 바뀌었다 — `close` 는 힘을 주지 않아 물체를 놓친다, ② 폭의 출처가 `/joint_states` 에서 `GripperState.width` 로 바뀌어 **단위가 관절값에서 개구 폭으로** 달라졌다(임계값을 2배로 조정), ③ 노드 존재 확인이 액션 서버에서 상태 수신으로 바뀌었다 | — |
| ~~Q15~~ | **닫힘 (2026-09-05) — plugin 이 실제로 더 무겁다.** 같은 머신·같은 씬에서: Isaac 단독 **0.989**(load 2.5) → **bridge** 스택을 얹어도 **0.989**(load 2.5, 사실상 0 비용) → **plugin** 스택은 **0.937**(load 8.4). dead time 도 갈린다 — `phase1` 기준 bridge **30 ms** / plugin **131 ms**. 즉 `controller_manager` 의 실시간 루프 + JTC 가 **배속 −5%, load 3배, dead time +100 ms** 를 얹는다. 부하의 출처는 Isaac 이 아니라 ROS 쪽이다 — 씬도 물리 계산량도 같은데 스택이 CPU 를 가져간다. **다만 `phase0` 의 0.872 는 이것으로 설명되지 않는다** (Q19) | — |
| ~~Q16~~ | **닫힘 (2026-09-05)** — `rdfp_panda_isaac.launch.py` 를 만들었다 (2026-09-13 삭제, `rdfp_collect backend:=isaac` 이 대체). bridge 판과 갈리는 것은 include 대상 하나뿐이다. **action 채널은 양쪽이 같다** (`/isaac/arm_command`) — plugin 에서 그 토픽을 채우는 것은 `TopicBasedSystem` 이고 컨트롤러 update 주기로 보간된 값이 흐른다. 드문드문한 계획(`joint_trajectory`)이 아니라 **실제로 로봇에 들어간 값**이라 모방학습에 맞고, 두 방식으로 모은 데이터셋이 섞이지 않는다 | — |
| ~~Q20~~ | **닫힘 (2026-09-05) — 방어 노드는 만들지 않는다. 감시로 충분하다.** 사슬 어디에도 NaN 필터가 없다는 것은 실측으로 확인했다 — JTC 는 로그 없이 받아들여 `controller_state.desired` 를 NaN 으로 만들고(mock), `TopicBasedSystem::write()` 는 발행 억제 조건이 `diff <= threshold` 인데 **NaN 비교가 거짓**이라 조기 반환을 건너뛰어 **오히려 반드시 발행한다**. Isaac 에서 `/isaac/arm_command` 에 NaN 이 실리는 것도 확인했다. **그런데 Isaac 이 흡수한다** — `ArticulationController` 가 NaN 목표를 관절별로 무시하고 마지막 목표를 유지하며, `/joint_states` 는 유한하게 남고 **다음 정상 궤적에 스스로 복구한다**(7축 전부 NaN·부분 NaN 둘 다). 그래서 필터 노드를 되살릴 이유가 없다. **남는 위험은 부분 NaN 하나** — 정상 6축은 목표로 가고 NaN 축만 제자리라 **'그럴듯하게 틀린 자세'가 조용히 나온다**(실측). 다만 servo 의 관측된 고장은 7축 전부 NaN 이고 그 경우는 팔이 멈출 뿐이다. **`mock` 으로는 이 문제가 안 보인다** — `GenericSystem` 이 NaN 을 흡수해 `desired` 를 봐야만 갈린다 | — |
| ~~Q21~~ | **닫힘 (2026-09-05) — `servo_command_bridge_node.py` 를 `isaac/` 에 그대로 둔다.** 소비자가 펑션베이뿐인데 파일은 `isaac/` 아래라 위치가 어긋나 보인다. 그래도 옮기지 않는 이유는 `functionbay/readiness_gate_node.py` 를 Isaac 이 쓰는 것과 **같은 상황**이고(백엔드 중립 노드가 한쪽 디렉터리에 있을 뿐), **옮기면 유일한 소비자의 회귀를 확인할 사람이 없기** 때문이다 — 펑션베이 작업이 중단 상태다. docstring 에 '현재 소비자는 펑션베이뿐'을 명시했다. 세 번째 소비자가 생기거나 펑션베이가 재개되면 두 파일을 중립 위치로 함께 옮긴다 | — |

# Isaac scene 리셋 — `/scene/reset`

Isaac 백엔드에서 물체를 다시 놓는 경로. 계약은 [scene_objects_guide.md](scene_objects_guide.md) §5 와 **같고**, 다른 것은 그 아래에서 누가 스테이지를 쓰느냐다.

```text
트윈 → /scene/reset  (rdfp_msgs/srv/ResetScene)      ← mock 과 같은 계약
         │  isaac_scene_state_node  (ROS 쪽, Linux)
         │  물체마다
         ▼
      /set_entity_state  (simulation_interfaces)     ← Isaac 이 여는 표준 서비스
         ▼
      USD 스테이지에 pose + twist 적용
         │
         ▼
      /get_entity_state 로 읽어서 확인 → success
```

**시뮬레이터 쪽에 상주 스크립트를 두지 않는다.** Isaac 6.0 의 `isaacsim.ros2.sim_control` 확장이 표준 `simulation_interfaces` 서비스를 이미 열어 주므로, ROS 쪽 노드가 그것을 부르기만 하면 된다.

---

## 1. 왜 이 구조인가

**ROS 쪽 노드는 USD 스테이지를 직접 못 쓴다.** Isaac 은 별도 프로세스이고 스테이지는 그 안에 있다. 그래서 오랫동안 "Isaac 에서 `reset_scene` 은 불가능하다"로 기록돼 있었다 ([isaac_backend_skeleton.md](../simulation/isaac_backend_skeleton.md) Phase 9).

**전제가 바뀐 것은 Isaac 이 창구를 열었기 때문이다.** `sim_control` 확장이 서비스 19개를 제공한다.

```
set_entity_state    get_entity_state    get_entities
spawn_entity        delete_entity       reset_simulation      ...
```

`SetEntityState.Request` 는 `entity`(string) + `state`(pose · twist · acceleration)라 **옮기면서 속도를 0 으로 만드는 것까지 한 번에** 된다. 그것이 리셋에 필요한 전부다.

### 대안을 택하지 않은 이유

| 대안 | 왜 아닌가 |
|---|---|
| Isaac 안에서 `rdfp_msgs/ResetScene` 을 직접 서빙 | Isaac 내장 파이썬은 **3.12**, Humble 은 **3.10** 이다. `rdfp_msgs` 를 3.12 용으로 따로 빌드해야 한다 |
| 표준 타입 토픽으로 배치를 넘기고 상주 스크립트가 적용 | 스크립트를 계속 띄워 둬야 하고, Isaac 이 이미 같은 일을 하는 서비스를 연다 |
| `setup_scene.py` 재실행 | scene root 를 지웠다 다시 만들어 `setup_graph.py` 도 함께 돌려야 하고, ROS 에서 부를 수 없다 |

---

## 2. ⚠️ `result=1` 이 "적용됐다"를 뜻하지 않는다

실측(2026-09-02)에서 **성공을 보고하고 아무것도 하지 않는** 경우를 확인했다.

| 대상 | 응답 | 실제 |
|---|---|---|
| `/OmniverseKit_Persp` (카메라) | `result=1`, `Successfully set state ... Entity doesn't have rigid body API` | **pose 그대로** |
| `/World/Scene/block_a` (rigid body) | `result=1`, `Successfully set state ... Position, orientation, and velocities set` | 이동함 |

같은 `result=1` 이고 메시지 문구도 겹친다. **그래서 이 노드는 응답을 판정에 쓰지 않고 `/get_entity_state` 로 읽어서 확인한다.**

비교는 위치와 자세를 **둘 다** 본다 (`_same_pose`). 쿼터니언은 `q` 와 `-q` 가 같은 회전이므로 **내적의 절대값**으로 견준다 — 부호를 그대로 비교하면 같은 자세를 다르다고 판정한다. 허용 오차는 `1e-3` 인데, Isaac 이 float32 로 돌려주기 때문이다 (`0.42` → `0.41999998688697815`).

---

## 3. Isaac 은 만들지도 지우지도 않는다

mock 은 planning scene 을 **통째로 교체**하지만, Isaac 의 스테이지 구성은
`setup_scene.py` 가 정한다. 이 서비스가 하는 일은 **다시 놓기**뿐이다.

| 요청 | 결과 |
|---|---|
| `objects` 가 비었다 | ❌ `Isaac cannot clear the scene; it can only reposition objects defined in isaac_scene.json` |
| `isaac_scene.json` 에 없는 이름 | ❌ `unknown object 'peg'; Isaac scene is fixed by isaac_scene.json: [...]` |
| `type` 이 다르다 | ❌ `type 'cylinder' != 'box' ...; Isaac cannot change geometry` |
| `dimensions` 가 다르다 | ❌ `dimensions [0.07,...] != [0.05,...] ...` |
| `type`/`dimensions` 생략 | ✅ 위치만 바꾸겠다는 뜻으로 읽는다 |

**조용히 넘기지 않는 것이 요점이다.** 성공으로 돌려주면 "그 배치로 놓았다"는 거짓이
에피소드 metadata 에 남고, 나중에 데이터를 열어보기 전까지 드러나지 않는다.

기하 불일치를 거부하는 이유도 같다 — 트윈이 5 cm 블록으로 알고 파지 좌표를 계산하는데
실제가 7 cm 면 **에러 없이 빗나간다.**

---

## 4. 선행 조건 두 가지

둘 다 갖추지 않으면 `/scene/reset` 이 열리지 않거나 호출이 실패한다.

### 4.1 Isaac 을 `sim_control` 확장과 함께 띄운다

`isaacsim.exp.full.kit` 은 이 확장을 **기본으로 켜지 않는다.**

### 4.2 시스템 ROS 를 소싱하지 않는다

Isaac 내장 파이썬은 **3.12**, `/opt/ros/humble` 은 **3.10** 이다. 시스템 ROS 를 소싱하면
3.10 `site-packages` 가 `PYTHONPATH` 에 올라 Isaac 의 번들 3.12 ROS 를 **가린다.**

```
[Error] Failed to import ROS2 Python libraries: No module named 'rclpy._rclpy_pybind11'
[Error] Failed to initialize ROS2 service manager
```

확장은 "startup" 까지 찍고 죽으므로 **로그를 보지 않으면 성공한 것처럼 보인다.**
증상은 `/isaac_sim_control` 노드가 없는 것뿐이다.

> ⚠️ `~/isaac_ros2_env.sh` 가 정확히 이 문제를 만든다 — `/opt/ros/humble` 을 소싱한다.
> Windows + WSL2 구성에서 쓰던 방식이라 Ubuntu 에서는 반대로 작동한다.

**동작하는 실행 명령:**

```bash
cd ~
ISAAC_ROS=$HOME/isaacsim/exts/isaacsim.ros2.core/humble
env -u AMENT_PREFIX_PATH -u CMAKE_PREFIX_PATH \
    ROS_DISTRO=humble ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    PYTHONPATH=$ISAAC_ROS/rclpy \
    LD_LIBRARY_PATH=$ISAAC_ROS/lib \
    ./isaacsim/isaac-sim.sh --enable isaacsim.ros2.sim_control
```

확인:

```bash
ros2 node list | grep isaac_sim_control      # 있어야 한다
ros2 service list | grep set_entity_state
```

### 4.3 Linux 쪽 패키지

```bash
sudo apt install ros-humble-simulation-interfaces
```

없으면 노드는 **정상 기동하되 `/scene/reset` 만 열지 않고** 기동 로그에 에러를 남긴다.
관측(`/scene/objects`)까지 막을 이유는 없기 때문이다.

```
[ERROR] 'simulation_interfaces' is not installed — /scene/reset stays closed.
        Install it with: sudo apt install ros-humble-simulation-interfaces
```

---

## 5. `entity` 이름 규약

**USD prim 절대 경로**다. `isaac_scene.json` 의 `root_prim` 과 물체 이름을 잇는다 —
`setup_scene.py` 가 prim 을 만드는 규칙(`f"{root_prim}/{spec['name']}"`)과 같다.

```
root_prim  "/World/Scene"  +  name "block_a"  →  "/World/Scene/block_a"
```

`ros2 service call /get_entities simulation_interfaces/srv/GetEntities '{}'` 로 실제
목록을 확인할 수 있다.

---

## 6. 검증 기록 (2026-09-02, 실기)

```
before block_a: (0.5,  -0.15, 0.425)      ← isaac_scene.json 값
before block_b: (0.5,   0.0,  0.425)

/scene/reset {block_a: (0.42, 0.18, 0.425), block_b: (0.58, -0.22, 0.425)}
  → success=True, applied_count=2

after  block_a: (0.42,  0.18, 0.425)      ← 정확히 이동
after  block_b: (0.58, -0.22, 0.425)
```

거부 경로 네 가지(빈 배열 · 모르는 이름 · 치수 불일치 · 타입 불일치)도 모두 사유와 함께
거부되는 것을 확인했다.

### 재생 중 검증 — 읽기 검증에 물리를 넣어야 했다 (2026-09-02)

재생 상태에서 다시 해 보니 **공중 배치가 거부됐다.**

```
/scene/reset {block_c: (0.5, 0.15, 0.9)}   → success=False
   'did not move: asked (0.5000, 0.1500, 0.9000) but read (0.5000, 0.1500, 0.8956)'
```

물체는 정확히 놓였는데, **놓자마자 떨어지기 시작해** 쓰고 읽는 사이에 허용 오차(1 mm)를
벗어났다. 자유낙하는 30 ms 면 4.4 mm 다.

그래서 `_same_pose` 가 **왕복 시간만큼의 자유낙하를 허용**한다
(`0.5·g·Δt²`). 고정 오차로는 지지면 없는 배치가 늘 실패한다. 낙하 허용이 "아무 일도
안 했다"까지 통과시키지는 않는다 — 그 경우 목표와의 거리가 자유낙하로 설명되지 않는다.

고친 뒤 재확인:

```
/scene/reset {block_c: z 0.425 → 0.9}  → success=True
  0.3초 후 : z = 0.425   ← 테이블에 안착 (√(2×0.475/9.81) = 0.31 s 와 일치)
```

**`twist=0` 도 확인됐다** — 리셋 직후 속도가 0 이고, 낙하는 그 뒤 중력으로 다시 시작한다.

---

## 7. 관련 문서

- [scene_objects_guide.md](scene_objects_guide.md) — scene 계약 전반 (§5 가 `/scene/reset`)
- [../simulation/isaac_backend_skeleton.md](../simulation/isaac_backend_skeleton.md) —
  Isaac 백엔드 Phase 0~10, 배포 구성
- `src/rdfp_msgs/srv/ResetScene.srv` — 서비스 정의 (정본)
- `src/robot_control/robot_control/isaac/scene_state_node.py` — 구현

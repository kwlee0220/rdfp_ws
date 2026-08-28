# Scene 물체 상태 — `SceneObject` / `SceneObjects` 계약

작성 2026-08-23 · 현행

씬(scene) 안 물체의 종류·크기·자세가 — 시뮬레이터 종류와 상관없이 똑같은 형식으로 — 어디서 나와 어디로 가는지 설명한다. 그 "똑같은 형식"이 `rdfp_msgs/SceneObjects` 메시지와 `/scene/objects` 토픽이며, 이 문서에서 **계약**이라 부르는 것이 그것이다 — 필드 이름부터 단위·좌표계·쿼터니언 순서·QoS 까지 포함한 약속이라, 양쪽이 그것만 지키면 서로의 구현을 몰라도 된다.

계약의 정본은 `src/rdfp_msgs/msg/*.msg` 의 주석이며, 이 문서는 그 내용에 발행 노드·트윈·launch·데이터셋 쪽 사정을 합쳐 한 곳에 모은 것이다.

---

## 1. 왜 있는가 — 백엔드 추상화 경계

mock(MoveIt planning scene) / Gazebo / Isaac Sim 은 물체를 표현하는 방식이 전혀 다르다.
그 차이를 **발행 노드 안에서 흡수**하고, 바깥에는 토픽 하나만 보인다.

```
mock    /monitored_planning_scene diff  ─┐
Gazebo  ros_gz_bridge (Pose_V)          ─┼→  *_scene_state_node  →  /scene/objects
Isaac   Python/USD publisher            ─┘                            (rdfp_msgs/SceneObjects)
                                                                          │
                                              ┌───────────────────────────┼──────────────┐
                                              ▼                           ▼              ▼
                                        robot_twin                    rosbag2      RViz 등
                                     (scene_objects 변수)            → DB 적재
```

**트윈과 후처리기는 환경 구현을 모른다.** 백엔드가 늘어도 그 둘은 고치지 않는다 — 발행 노드를 하나 더 만들 뿐이다. 좌표계·단위·쿼터니언 순서를 맞추는 책임은 **전부 발행 노드에 있다**.

토픽으로 낸 이유가 하나 더 있다. **rosbag2 에 그대로 기록되어 학습 데이터의 일부가 된다.** 트윈만 보는 채널(서비스 등)로 만들면 물체 위치가 데이터셋에 남지 않는다 — `rdfp_msgs/GripperCommand` 를 만든 것과 같은 이유다.

---

## 2. 계약 한눈에

| 토픽 | 타입 | 방향 | QoS |
|---|---|---|---|
| `/scene/objects` | `rdfp_msgs/SceneObjects` | 노드 → 소비자 (주기 발행) | RELIABLE / **TRANSIENT_LOCAL** / depth 1 |
| `/scene/commands` | `rdfp_msgs/SceneCommand` | 트윈 → 노드 | 기본 |
| `/scene/command_results` | `rdfp_msgs/SceneCommandResult` | 노드 → 트윈 | RELIABLE / TRANSIENT_LOCAL / depth 1 |

> **구독 측도 durability 를 맞춰야 한다.** `TRANSIENT_LOCAL` 발행을 volatile 로 구독하면 **매칭 자체가 되지 않아** 값이 영영 오지 않는다. `ros2 topic echo` 도 마찬가지다 (`--qos-durability transient_local`).

---

## 3. `SceneObject` — 물체 하나

```
string          name
string          type
float64[]       dimensions
geometry_msgs/Pose  pose
```

### 3.1 `name` — 인덱스가 아니라 이름으로 지목한다

물체들은 `SceneObjects.objects` (`SceneObject[]`, §4) 에 배열로 담겨 오는데, **그 순서는 보장되지 않는다.** 그래서 "두 번째 물체"가 아니라 `name` 으로 지목한다. 백엔드가 바뀌어도 같은 물체는 같은 이름을 유지해야 한다.

트윈이 이 배열을 **맵으로 투영**(`projection: scene_object_map`, §7)해 `{이름: {...}}` 형태로 내보내는 것도 같은 이유다 — 클라이언트가 인덱스 대신 이름으로 접근하게 한다.

### 3.2 `type` — 문자열이다

값은 `shape_msgs/SolidPrimitive` 의 이름을 **소문자로** 쓰거나 `'mesh'` 다. 흔한 값은 `'box'` / `'sphere'` / `'cylinder'` / `'mesh'` 이며 `'cone'` / `'prism'` 도 올 수 있다.

`type` 을 uint8 상수가 아니라 문자열로 표현하는 이유는 둘이다.

- 트윈의 `enums` 변환기가 **최상위 필드만** 훑으므로, 배열 안에 중첩된 이 필드는 숫자인
  채로 노출된다. 변환기를 중첩 지원으로 확장하면 경로 표기가 필요해져 설정만 복잡해진다.
- 기록되는 채널이라 데이터셋 가독성 면에서도 문자열이 낫다.

### 3.3 `dimensions` — `SolidPrimitive` 순서를 그대로 따른다

단위는 m 이고, 길이와 의미가 `type` 에 따라 달라진다.

| `type` | 길이 | 의미 |
|---|:-:|---|
| `box` | 3 | x, y, z |
| `sphere` | 1 | 반지름 |
| `cylinder` | 2 | **높이, 반지름** |
| `cone` | 2 | 높이, 반지름 |
| `mesh` | 0 | 비어 있다 |

> ⚠️ **`cylinder` 는 높이가 먼저다.** `SolidPrimitive` 의 `CYLINDER_HEIGHT=0`, `CYLINDER_RADIUS=1` 순서이며 직관과 반대다.

자체 순서를 정하지 않고 ROS 표준을 따르는 이유는, mock 백엔드가 planning scene 의 `SolidPrimitive` 를 **재배열 없이 그대로 옮길 수 있게** 하기 위해서다. 재배열이 들어가는 순간 그것이 조용히 틀릴 수 있는 지점이 된다.

**길이가 종류마다 다르다는 점이 이 메시지가 배열-of-구조체인 이유다.** ROS IDL 에 ragged array 가 없어 `JointState` 식 병렬 배열(`string[] name` + `Pose[] pose` + …)로는 표현할 수 없다. 이 성질은 DB 적재(§8)와 학습 입력 인코딩에도 그대로 따라온다.

### 3.4 `pose` — position 은 m, orientation 은 **xyzw**

`geometry_msgs/Pose` 이며 orientation 은 **ROS 규약 xyzw · unit norm** 이다.

> ⚠️ **Isaac Sim 은 wxyz(스칼라 우선)이므로 발행 노드가 반드시 변환해야 한다.** 순서를 틀려도 4개 float 에 unit norm 이라 **타입 검사도 정규화 검사도 전부 통과**하고, 결과는 크래시가 아니라 '그럴듯하게 틀린 자세'다. 예를 들어 Isaac 의 '회전 없음' `(w,x,y,z)=(1,0,0,0)` 을 그대로 옮기면 `(x,y,z,w)=(1,0,0,0)`, 즉 x축 180° 회전이 되는데 이는 '그리퍼가 아래를 본다'는 **지극히 정상적인 자세**라 눈으로는 드러나지 않는다.

**검증은 norm 이 아니라 알려진 자세로 한다** — 회전 없는 물체가 `(0,0,0,1)` 로 나오는지, 그리고 축이 섞이는 비대칭 회전(z축 90° 등)이 맞게 나오는지 확인한다.

---

## 4. `SceneObjects` — 한 시점의 씬 전체

```
std_msgs/Header  header
SceneObject[]    objects
```

### `header.stamp` — 비워 두면 안 된다

적재 코드가 이 값을 그대로 쓰므로(`dataset/db/writers/base.py` 의 `extract_stamp`), 비면 **epoch 0 에 적재되어 어느 에피소드에도 속하지 못한다.** 로봇은 정상 동작하므로 데이터를 열어보기 전까지 드러나지 않는다.

### `header.frame_id` — 로봇 베이스 프레임

현재 스택에서는 `panda_link0` 다. Gazebo 의 world frame 은 로봇 베이스와 다르므로 **변환 책임은 발행 노드가 진다.** 여기서 안 맞추면 백엔드마다 다른 좌표가 섞여 데이터셋이 조용히 오염된다.

DB 의 `scene_objects` 테이블이 다른 테이블과 달리 `frame_id` 를 저장하는 이유가 이것이다 — 변환을 빠뜨려도 pose 값 자체는 그럴듯해서, 프레임을 함께 적지 않으면 오염이 드러나지 않는다.

### `objects` — 빈 배열도 유효한 상태다

물체가 없으면 빈 배열이며, 그것도 **'씬이 비었다'는 유효한 상태**다. "아직 못 받았다"와 구분된다.

---

## 5. 명령 경로 — `SceneCommand` / `SceneCommandResult`

### 5.1 `SceneCommand` — `reset` 하나뿐이다

```
std_msgs/Header  header
string           command      # 'reset' — objects 로 씬을 통째로 교체
SceneObject[]    objects      # 빈 배열은 '씬을 비운다'
string           scene        # 레시피 이름 (기록용)
int64            seed         # 추출 seed (기록용)
```

개별 추가/삭제 명령을 두지 않는 이유는 **그 조합 로직이 클라이언트로 새어 나가고, 그러면 "어떤 배치였는지"가 데이터셋 밖에 남기** 때문이다.

**물체 배치는 이미 정해진 상태로 온다.** 무작위 추출은 트윈이 하고 여기에는 결과만 실린다. 백엔드 노드가 뽑으면 (1) 같은 seed 로도 백엔드마다 다른 배치가 나오고, (2) 트윈이 실제 배치를 몰라 에피소드 metadata 에 남길 수 없다.

> **`seed` 만으로는 재현되지 않는다.** 추출 알고리즘이 바뀌면 같은 seed 가 다른 배치를 만든다. 재현의 근거는 `objects` 이며 seed 는 사람이 실행을 식별하는 용도다.

이 토픽도 rosbag2 에 기록된다. `reset_scene` 은 에피소드 **밖**에서 일어나 어느 에피소드에도 속하지 않지만, 세션 전체를 훑으면 "어떤 배치로 시작했는지"를 되짚을 수 있다.

### 5.2 `SceneCommandResult` — 명령 하나당 1건

백엔드 발행 노드가 `SceneCommand` 를 처리하고 그 결과를 낸다. **트윈은 명령을 토픽으로 흘린 뒤 이 값이 갱신될 때까지 기다렸다가 연산을 끝낸다** — 그래야 `reset_scene` 이 완료를 보고할 수 있다(§7).

```
std_msgs/Header  header
bool             success
string           message
int32            applied_count
```

| 필드 | 의미 |
|---|---|
| `header` | `stamp` 는 발행 시각, `frame_id` 는 `''` |
| `success` | 처리 성공 여부 |
| `message` | 실패 사유. 성공 시 `''` |
| `applied_count` | 적용된 물체 수. **성공 시 요청한 개수와 같아야 한다** |

#### 왜 별도 토픽인가

두 가지를 피하려고 이 형태다.

- **서비스 응답으로 받으면 rosbag2 에 기록되지 않아** 나중에 "리셋이 실패했었는지"를 알
  수 없다 (`gripper_control_node` 와 같은 구조다).
- **`/scene/objects` 를 완료 신호로 쓸 수 없다.** 그쪽은 주기 발행이라 명령과 무관하게
  계속 갱신되므로, '갱신됨'을 완료로 삼으면 **리셋 이전 상태를 완료로 오인한다.**

---

## 6. 발행 노드 — `mock_scene_state_node`

[src/robot_control/robot_control/scene/mock_scene_state_node.py](../../src/robot_control/robot_control/scene/mock_scene_state_node.py)

### 6.1 서비스 폴링이 아니라 diff 누적이다

처음에는 `/get_planning_scene` 서비스를 폴링했으나 **간헐적으로 빈 world 를 반환**했다. 물체가 있는 상태에서 1초 간격 10회 조회에 `[3,0,0,0,0,0,3,3,3,0]` 이 나왔다. 요청 컴포넌트 마스크를 바꿔도, 다른 조회자를 모두 없애도, RViz 를 종료해도 같았다. move_group 의 planning scene monitor 가 diff scene 과 parent 를 오가는 것으로 보인다.

그래서 `/monitored_planning_scene` 토픽을 구독한다. 토픽은 **diff** 를 실어 나르므로 (변경된 물체만, `is_diff=True`) 노드가 상태를 누적한다. 그 대가로 서비스의 불안정과 무관해진다.

### 6.2 구독은 이벤트, 발행은 주기 (기본 2 Hz)

1. **백엔드마다 성격이 다르다** — 물리 백엔드는 물체가 계속 움직여 주기 발행이 자연스럽다. mock 만 이벤트성이면 트윈의 `staleness_ms` 를 백엔드별로 달리 잡아야 한다.
2. **발행자가 죽은 것을 감지할 수 있다.** 이벤트 발행이면 씬이 조용히 얼어붙은 것과 정상인 것이 구분되지 않는다.

### 6.3 QoS 두 개가 서로 다르다

| 대상 | durability | 이유 |
|---|---|---|
| `/scene/objects` (발행) | TRANSIENT_LOCAL | 늦게 뜬 트윈·recorder 가 현재 씬을 즉시 본다 |
| `/monitored_planning_scene` (구독) | **VOLATILE** | move_group 이 그렇게 발행한다. TRANSIENT_LOCAL 로 구독하면 QoS 불일치로 **한 건도 받지 못한다** (경고만 뜬다) |

### 6.4 좌표 변환은 합성이 두 번이다

```
base ← 오브젝트 프레임 (TF)  ∘  오브젝트 ← primitive (CollisionObject 내부)
```

**뒤쪽을 빼먹으면 오프셋을 가진 물체가 어긋난다.** 이 합성은 [pose_math.py](../../src/robot_control/robot_control/scene/pose_math.py) 가 담당하며, `tf2_geometry_msgs.do_transform_pose` 를 쓰지 않는다 — 배포판마다 시그니처가 달라 (Pose vs PoseStamped) 조용히 어긋날 수 있고, **ROS 없이 단위 테스트할 수 있어야** 하기 때문이다. 쿼터니언 순서 오류는 크래시가 아니라 '그럴듯하게 틀린 자세'로 나타나므로 테스트로 고정해 두는 값이 크다.

### 6.5 기하가 하나가 아닐 때

- `CollisionObject` 에 primitive 가 **여러 개**면 첫 기하만 발행하고 경고를 남긴다 (계약은 물체 하나에 기하 하나다). 조용히 넘기지 않는 이유는 크기가 실제와 달라지기 때문.
- primitive 가 없고 **mesh 만** 있으면 `type: 'mesh'`, `dimensions: []` 로 나간다.
- 둘 다 없거나 좌표 변환이 불가능하면 **그 물체만 건너뛰고 경고**한다 — 물체 하나 때문에 나머지 씬 전체를 발행하지 못하는 편이 더 나쁘다.

> ⚠️ **읽기와 쓰기가 비대칭이다.** 발행 방향은 `mesh` 를 통과시키지만, `reset_scene` 방향은 `box`/`sphere`/`cylinder`/`cone`/`prism` 만 받고 `mesh` 는 `unsupported type 'mesh' for '<name>'` 로 거부한다. 또한 **메시지에 메시 기하를 실을 필드가 없다** (경로·URI·정점 어느 것도 없다) — 즉 현재 `type: 'mesh'` 는 "primitive 가 아니다" 이상의 정보를 담지 못한다.

### 6.6 mock 의 pose 는 ground truth 가 아니다

planning scene 의 물체는 **물리를 갖지 않는다.** 누가 넣은 값 그대로 있으며, 파지에 실패해도 굴러떨어져도 변하지 않는다. 따라서 **mock 에서 수집한 에피소드로는 성패를 관측할 수 없고**, 이 노드의 값은 변수·연산·기록 경로가 도는지 확인하는 **배관 검증용**이다.

실제 파지 판정은 Gazebo 백엔드 연동 이후다.

---

## 7. 트윈에서 보기

### 변수 `scene_objects` — 배열이 아니라 맵

`projection: scene_object_map` 이 적용되어 REST 응답은 이름 키의 맵이다.

```jsonc
{ "header": { "stamp": { "sec": 1786944345, "nanosec": 868439364 },
              "frame_id": "panda_link0" },
  "objects": {
    "cube_0": { "type": "box",    "dimensions": [0.05, 0.05, 0.05],
                "pose": { "position":    { "x": 0.382, "y": -0.135, "z": 0.025 },
                          "orientation": { "x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0 } } },
    "ball_0": { "type": "sphere", "dimensions": [0.02], "pose": { "...": "..." } } } }
```

`staleness_ms: 1000` — 2 Hz 발행이므로 이보다 오래 끊기면 `STALE` 이다. **물체를 집기 전 파지 좌표를 여기서 얻는다.** 고정 좌표를 쓰면 물체가 그 자리에 없다.

### 변수 `scene_last_command_result`

마지막 `reset_scene` 의 결과다. **이벤트성**이라 명령이 없는 동안 갱신되지 않으므로 `staleness_ms: null` 이다. `reset_scene` 이 이미 이 값을 기다렸다가 반환하므로 직접 읽을 일은 거의 없고, 실패 원인 확인용이다.

### 연산 `reset_scene`

| 항목 | 값 |
|---|---|
| `kind` | `sync` (`sync_timeout_sec: 10.0`) |
| `resource` | **`[scene, arm]`** — 팔이 움직이는 중이면 `409 RESOURCE_BUSY` |
| 입력 | `scene`(필수, 레시피 이름) · `seed`(선택) |
| 출력 | `scene`, `seed`, **`objects`(실제 배치)**, `success`, `applied_count` |

무작위 추출은 [backends.py `_sample_scene`](../../src/robot_twin/robot_twin/backends.py) 이 한다. 레시피의 각 축은 **숫자(고정)** 또는 **`[최소, 최대]`(균등 추출)** 이며 `random.Random(seed)` 를 쓴다. 레시피에 자세가 없으므로 **orientation 은 회전 없음 `(0,0,0,1)`** 으로 채워 보낸다.

```yaml
scenes:
  one_cube:
    objects:
      - name: cube_0
        type: box
        size: [0.05, 0.05, 0.05]   # → dimensions
        x: [0.35, 0.55]            # 범위 → 균등 추출
        y: [-0.15, 0.15]
        z: 0.025                   # 숫자 → 고정
```

**`resource` 에 `arm` 이 함께 들어간 이유**는, `scene` 만 점유하면 "리셋 중에 팔이 진입"하는 방향이 열려 있기 때문이다. 사전조건 검사로 막는 안은 검사·점유 사이에 **TOCTOU**(Time-Of-Check to Time-Of-Use — 확인한 뒤 실제로 쓰기까지의 틈에 상태가 바뀌어 확인 결과가 무의미해지는 경쟁 조건) 창이 남아 기각됐다.

### 파지 좌표를 만드는 쪽 (MCP / `pick_n_place`)

`grasp_pose_of_object` 는 `reset_scene` 의 `outputs.objects` 원소를 그대로 받아 **물체
중심을 위에서 잡는** 자세를 만든다. 돌려주는 좌표는 **손끝(TCP) 기준**이며, 팔에 명령할
때 `to_arm_command()` 로 `panda_link8` 기준으로 바꾼다 — 손끝이 거기서 약 10.3 cm 더
뻗어 있어, 보정을 빠뜨리면 5 cm 큐브(중심 z=0.025)를 집을 때 손끝이 z = -0.078 로
**바닥을 파고든다.**

---

## 8. launch 와 데이터셋

### launch 인자 둘 (YAML 을 거치지 않는다)

| 인자 | 기본 | 설명 |
|---|---|---|
| `enable_scene_node` | `true` | 백엔드 scene 상태 노드 기동 여부 |
| `scene_publish_rate` | `2.0` | `/scene/objects` 발행 Hz |

- **기본 on 은 의도적이다.** 노드는 2 Hz 타이머 하나를 쓰고 `/scene/commands` 가 오기 전까지 아무것도 바꾸지 않는 반면, 꺼 두면 트윈의 `reset_scene` 이 **결과 토픽 타임아웃**으로 죽으면서 로그에 원인이 남지 않는다.
- **`publish_rate` 라는 이름을 쓸 수 없다** — EE pose 헬퍼가 이미 그 이름을 50 Hz 기본값으로 선언하고 있어 충돌한다.
- **`base_frame` 은 이 헬퍼가 선언하지 않는다.** 이미 선언되어 있다고 가정한다 — 물체 pose 의 기준 프레임은 EE pose 의 기준과 같은 로봇 base 여야 하므로, 별도 인자로 열면 두 값이 어긋날 때 조용히 틀린 좌표가 나간다.
- **기동 순서 제약이 없다.** 생성자가 `wait_for_service` 를 부르지 않으므로 `move_group` 과 동시에 spawn 해도 안전하다. 초기 몇 초는 빈 목록을 발행하다가 채워진다.
- **replay 스택에는 넣지 않는다.** 재생 중 물체 상태는 데이터셋에서 나와야 하고, 라이브 노드를 띄우면 `/scene/objects` 에 **두 번째 출처**가 생긴다.

백엔드가 늘면 `create_mock_scene_node()` 를 고치지 말고 `create_gazebo_scene_node()` 같은 **형제 팩토리**를 만든다 — executable 만 다르고 argument 계약은 같다.

### 데이터셋 적재

`/scene/objects` 는 [config/recording_topics.list](../../config/recording_topics.list) 에 등록되어 rosbag2 에 녹화되고, `scene_objects` 테이블에 적재된다.

**한 메시지가 한 행**이며 물체 배열은 `objects JSONB` 에 통째로 들어간다. 물체별 행으로 나누지 않는 이유는 (1) reader 계약이 row 1개 → 메시지 1개이고, (2) §3.3 의 ragged `dimensions` 가 고정 컬럼으로 정규화되지 않기 때문이다.

```sql
SELECT o->>'name' AS name, (o->'position'->>0)::float8 AS x
FROM scene_objects s, jsonb_array_elements(s.objects) o
WHERE s.episode_id = %s AND s.object_count > 0
ORDER BY s.stamp_ts DESC;
```

**학습 입력(observation)으로 쓸지는 별개의 미결 사항**이다 → [scene_objects_observation_decision.md](../rosbag2/scene_objects_observation_decision.md)

---

## 9. 함정 모음

| 증상 | 원인 |
|---|---|
| 토픽은 있는데 값이 한 건도 안 온다 | 구독 durability 불일치. TRANSIENT_LOCAL 발행을 volatile 로 구독하면 매칭 자체가 안 된다 |
| 물체가 데이터셋의 어느 에피소드에도 없다 | `header.stamp` 가 비어 epoch 0 으로 적재됨 |
| 백엔드마다 좌표가 다르다 | `frame_id` 를 로봇 베이스로 변환하지 않음. 변환은 발행 노드 책임 |
| 자세가 '그럴듯하게' 틀리다 | 쿼터니언 wxyz ↔ xyzw 혼동. norm 검사로는 안 잡힌다 — 알려진 비대칭 회전으로 확인 |
| cylinder 크기가 이상하다 | `dimensions` 가 `[높이, 반지름]` 순서다 |
| 오프셋 가진 물체만 어긋난다 | `오브젝트 ← primitive` 합성 누락 (§6.4) |
| `reset_scene` 이 타임아웃으로 실패 | `enable_scene_node:=false` 로 노드가 없음 |
| `reset_scene` 이 409 | `scene` 뿐 아니라 `arm` 도 점유한다 — 팔 이동 중에는 거부 |
| 씬이 조용히 얼어붙었다 | 주기 발행이므로 `staleness_ms` 로 감지된다 (이벤트 발행이면 구분 불가) |
| mock 에서 파지 성패가 안 잡힌다 | 물리가 없어 pose 가 변하지 않는다. 배관 검증용이다 |
| 곰 인형 같은 물체를 배치할 수 없다 | `reset_scene` 은 primitive 만 받는다. `mesh` 는 읽기 전용이며 기하를 실을 필드도 없다 (§6.5) |

---

## 10. 파일 위치

| 구성요소 | 경로 |
|---|---|
| 메시지 정의 (정본) | `src/rdfp_msgs/msg/SceneObject.msg` · `SceneObjects.msg` · `SceneCommand.msg` · `SceneCommandResult.msg` |
| 발행 노드 (mock) | [scene/mock_scene_state_node.py](../../src/robot_control/robot_control/scene/mock_scene_state_node.py) |
| pose 합성 (ROS 무의존) | [scene/pose_math.py](../../src/robot_control/robot_control/scene/pose_math.py) |
| launch 헬퍼 | [launch_helpers/scene.py](../../src/robot_control/robot_control/launch_helpers/scene.py) |
| 트윈 설정 | `src/robot_twin/config/robot_twin_panda01.yaml` (변수 2개 + `reset_scene`) |
| 트윈 핸들러 | [backends.py](../../src/robot_twin/robot_twin/backends.py) `_sample_scene` / `_reset_scene` |
| 녹화 대상 | [config/recording_topics.list](../../config/recording_topics.list) |
| DB 적재 | [writers/scene_objects.py](../../src/rdfp/rdfp/dataset/db/writers/scene_objects.py) · [readers/scene_objects.py](../../src/rdfp/rdfp/dataset/db/readers/scene_objects.py) · [sql/schema.sql](../../src/rdfp/rdfp/dataset/sql/schema.sql) |
| 테스트 | `src/robot_control/robot_control/scene/tests/` (`test_mock_scene_state_node.py` · `test_pose_math.py`) |

---

## 11. 참고

- [robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) — §4.1 변수 응답 형식,
  §4.2 `reset_scene`, §4.4 자원 락
- [auto_episode_collection_draft.md](../robot_twin/auto_episode_collection_draft.md) — §2
  타입 설계 경위(`PoseArray` 를 쓸 수 없던 이유), §3 `reset_scene` 결정, §2.5 좌표 규약
  어댑터, §2.6 diff 누적으로 바꾼 실측
- [scene_objects_observation_decision.md](../rosbag2/scene_objects_observation_decision.md) —
  학습 입력에 넣을 것인가 (미결)
- [multi_simulator_backend_design.md](../simulation/multi_simulator_backend_design.md) — §5
  백엔드 계약. 이 토픽은 그 §5.3 의 "환경 오브젝트"(선택)를 계약으로 승격한 것이다
- [robot_control/launch/README.md](../../src/robot_control/launch/README.md) — §5 헬퍼 인벤토리
- [rdfp/launch/README.md](../../src/rdfp/launch/README.md) — scene 인자가 YAML 밖에 있는 이유

# Scene 물체 상태 — `SceneObject` / `SceneObjects` 계약

작성 2026-08-23 · 현행

scene 안 물체의 종류·크기·자세가 — 시뮬레이터 종류와 상관없이 똑같은 형식으로 — 어디서 나와 어디로 가는지 설명한다. 그 "똑같은 형식"이 `rdfp_msgs/SceneObjects` 메시지와 `/scene/objects` 토픽이며, 이 문서에서 **계약**이라 부르는 것이 그것이다 — 필드 이름부터 단위·좌표계·쿼터니언 순서·QoS 까지 포함한 약속이라, 양쪽이 그것만 지키면 서로의 구현을 몰라도 된다.

계약의 정본은 `src/rdfp_msgs/msg/*.msg` 의 주석이며, 이 문서는 그 내용에 발행 노드·트윈·launch·데이터셋 쪽 사정을 합쳐 한 곳에 모은 것이다.

---

## 1. 왜 있는가 — 백엔드 추상화 경계

mock(MoveIt planning scene) / Gazebo / Isaac Sim 은 물체를 표현하는 방식이 전혀 다르다. 그 차이를 **발행 노드 안에서 흡수**하고, 바깥에는 토픽 하나만 보인다.

```
mock    /monitored_planning_scene diff  ─┐
Isaac   물체 TF (ROS2PublishTransformTree)─┼→  *_scene_state_node  →  /scene/objects
Gazebo  ros_gz_bridge (Pose_V, 미구현)   ─┘                            (rdfp_msgs/SceneObjects)
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

**타입 이름과 토픽 이름은 다르다.** 타입은 단수(`SceneObject`), 토픽은 복수(`/scene/objects`)다 — 메시지 하나가 물체 하나를 담고, 토픽은 그것들이 흐르는 채널이기 때문이다. 여러 개를 한 메시지에 묶는 타입은 복수형(`SceneObjects`)을 쓴다.

**쓰기 경로는 토픽이 아니라 서비스다.** scene 을 바꾸는 것은 `/scene/reset` 서비스이고, 기록·적재되는 것은 `/scene/objects` 뿐이다. 서비스를 고른 이유는 §5 에 있다.

| 토픽 | 타입 | 방향 | QoS |
|---|---|---|---|
| `/scene/objects` | `rdfp_msgs/SceneObjects` | 노드 → 소비자 (주기 발행) | RELIABLE / **TRANSIENT_LOCAL** / depth 1 |
| `/scene/reset` (**서비스**) | `rdfp_msgs/srv/ResetScene` | 트윈 → 노드 | — |

> **QoS 세 값의 뜻**
>
> | 값 | 뜻 | 왜 이 값인가 |
> |---|---|---|
> | **RELIABLE** | 도착을 보장한다 — 유실되면 재전송한다 (반대는 `BEST_EFFORT`, 놓치면 그만) | scene 은 저빈도(2 Hz) 상태 채널이라 재전송 비용이 작고, **물체 하나를 놓치면 그 자리에 없는 것으로 보이는** 오류가 그대로 데이터셋에 남는다 |
> | **TRANSIENT_LOCAL** | 발행자가 마지막 메시지를 들고 있다가 **늦게 붙은 구독자에게 즉시 준다** (반대는 `VOLATILE`, 붙은 뒤 것만 받는다) | 트윈·recorder 는 스택보다 늦게 뜨는 일이 흔한데, 다음 주기(0.5 초)를 기다리지 않고 **붙자마자 현재 scene 을 본다** |
> | **depth 1** | 보관하는 과거 메시지 수 = 1 (`KEEP_LAST` 이력) | 이 토픽은 **최신 상태만 의미가 있다.** 과거 scene 을 몰아서 받으면 오히려 낡은 값을 처리하게 된다 |
>
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

트윈이 이 배열을 **맵으로 투영**(`projection: scene_object_map`, §8)해 `{이름: {...}}` 형태로 내보내는 것도 같은 이유다 — 클라이언트가 인덱스 대신 이름으로 접근하게 한다.

### 3.2 `type` — 문자열이다

값은 `shape_msgs/SolidPrimitive` 의 이름을 **소문자로** 쓰거나 `'mesh'` 다. 흔한 값은 `'box'` / `'sphere'` / `'cylinder'` / `'mesh'` 이며 `'cone'` / `'prism'` 도 올 수 있다.

`type` 을 uint8 상수가 아니라 문자열로 표현하는 이유는 둘이다.

- 트윈의 `enums` 변환기가 **최상위 필드만** 훑으므로, 배열 안에 중첩된 이 필드는 숫자인 채로 노출된다. 변환기를 중첩 지원으로 확장하면 경로 표기가 필요해져 설정만 복잡해진다.
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

> `shape_msgs/SolidPrimitive` 는 기본 입체(box / sphere / cylinder / cone / prism)를 `type` + `dimensions` 로 나타내는 **ROS 표준 메시지**이며, 각 칸의 의미를 상수로 못 박아 둔다.
>
> ⚠️ **`cylinder` 는 높이가 먼저다.** `SolidPrimitive` 의 `CYLINDER_HEIGHT=0`, `CYLINDER_RADIUS=1` 순서이며 직관과 반대다.

자체 순서를 정하지 않고 ROS 표준을 따르는 이유는, mock 백엔드가 planning scene 의 `SolidPrimitive` 를 **재배열 없이 그대로 옮길 수 있게** 하기 위해서다. 재배열이 들어가는 순간 그것이 조용히 틀릴 수 있는 지점이 된다.

**길이가 종류마다 다르다는 점이 이 메시지가 배열-of-구조체인 이유다.** ROS IDL 에 ragged array 가 없어 `JointState` 식 병렬 배열(`string[] name` + `Pose[] pose` + …)로는 표현할 수 없다. 이 성질은 DB 적재(§9)와 학습 입력 인코딩에도 그대로 따라온다.

### 3.4 `pose` — position 은 m, orientation 은 **xyzw**

`geometry_msgs/Pose` 이며 orientation 은 **ROS 규약 xyzw · unit norm** 이다.

> ⚠️ **Isaac Sim 은 wxyz(스칼라 우선)이므로 발행 노드가 반드시 변환해야 한다.** 순서를 틀려도 4개 float 에 unit norm 이라 **타입 검사도 정규화 검사도 전부 통과**하고, 결과는 크래시가 아니라 '그럴듯하게 틀린 자세'다. 예를 들어 Isaac 의 '회전 없음' `(w,x,y,z)=(1,0,0,0)` 을 그대로 옮기면 `(x,y,z,w)=(1,0,0,0)`, 즉 x축 180° 회전이 되는데 이는 '그리퍼가 아래를 본다'는 **지극히 정상적인 자세**라 눈으로는 드러나지 않는다.

**검증은 norm 이 아니라 알려진 자세로 한다** — 회전 없는 물체가 `(0,0,0,1)` 로 나오는지, 그리고 축이 섞이는 비대칭 회전(z축 90° 등)이 맞게 나오는지 확인한다.

---

## 4. `SceneObjects` — 한 시점의 scene 전체

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

물체가 없으면 빈 배열이며, 그것도 **'scene 이 비었다'는 유효한 상태**다. "아직 못 받았다"와 구분된다.

---

## 5. scene 갱신 — 서비스 `/scene/reset`

> **백엔드마다 할 수 있는 일이 다르다.** mock 은 planning scene 을 통째로 교체하지만,
> Isaac 은 물체를 **다시 놓기만** 한다 (스테이지 구성은 `setup_scene.py` 가 정한다) —
> 빈 배열과 모르는 이름은 거부된다. Isaac 쪽 구조·선행 조건·함정은
> [isaac_scene_reset.md](isaac_scene_reset.md) 에 있다.

scene 을 바꾸는 것은 **서비스**다. 타입은 `rdfp_msgs/srv/ResetScene`, 이름은 `/scene/reset` 이며 백엔드별 scene 노드가 서버가 된다.

```
# 요청
SceneObject[] objects     # 새로 놓을 물체 전체. 빈 배열은 'scene 을 비운다'
string        scene       # 레시피 이름 (기록·진단용)
int64         seed        # 추출 seed (기록·진단용)
---
# 응답
bool   success
string message
int32  applied_count      # 성공 시 요청한 개수와 같아야 한다
```

### 왜 토픽이 아니라 서비스인가

**scene 변경은 요청–응답이 자연스러운 연산이다.** 호출자는 "적용됐는가"를 그 자리에서 알아야 하고, 실패했다면 다음 단계로 넘어가면 안 된다. 토픽으로 하면 결과를 받을 채널을 따로 두고, 어느 응답이 내 요청의 것인지 짝지어야 한다.

**기록이 필요한 것은 명령이 아니라 결과 상태다.** 데이터셋에 남아야 하는 것은 "무엇을 요청했나"가 아니라 **실제로 어떻게 놓였나**(`/scene/objects`)다. 물리 시뮬레이터에서는 명령한 배치와 실제 안착 위치가 다르기 때문이다 — 블록이 굴러 기울거나 미끄러진다. 그래서 서비스 응답이 rosbag2 에 남지 않는 것은 문제가 되지 않는다.

**세션 계층과 같은 형태다** — `session_control_node` 도 명령은 서비스, 관측 가능한 상태(`/session`)는 토픽이고, 기록되는 것은 상태 쪽이다.

### 배치의 출처는 어떻게 추적하나

트윈이 `reset_scene` 의 `outputs.objects` 를 돌려주고, 호출자가 그것을
`stop_episode` 의 metadata 로 넘겨 에피소드에 붙인다.

> **`seed` 만으로는 재현되지 않는다.** 추출 알고리즘이 바뀌면 같은 seed 가 다른 배치를 만든다. 재현의 근거는 `objects` 이며 seed 는 사람이 실행을 식별하는 용도다.

**물체 배치는 이미 정해진 상태로 온다.** 무작위 추출은 트윈이 하고 서비스에는 결과만
실린다. 노드가 뽑으면 (1) 같은 seed 로도 백엔드마다 다른 배치가 나오고, (2) 트윈이
실제 배치를 몰라 metadata 에 남길 수 없다.

그리고 물리 시뮬레이터에서는 **명령한 배치와 실제 안착 위치가 다르다** — 블록이 굴러 기울거나 미끄러진다. 그래서 학습 데이터의 근거는 실제 위치, 즉 `/scene/objects` 다.

### 개별 추가/삭제를 두지 않는 이유

그 조합 로직이 클라이언트로 새어 나가고, 그러면 "어떤 배치였는지" 가 데이터셋 밖에 남는다. 통째로 교체하는 요청 하나만 둔다.

---

## 6. 발행 노드 — `실가상 로봇 환경`마다 하나

**노드는 백엔드와 1:1 로 묶인다.** 어느 스택을 기동하느냐가 곧 어느 어댑터가 필요한지를 정하므로, 그 짝을 사용자에게 맡기지 않고 launch 파일이 백엔드별 팩토리를 골라 쓴다.

| 백엔드 | 노드 | 상태 | 원본 → `/scene/objects` |
|---|---|:-:|---|
| **mock** | [`mock_scene_state_node`](../../src/robot_control/robot_control/scene/mock_scene_state_node.py) | ✅ | MoveIt `/monitored_planning_scene` **diff 누적** (§6.2) |
| **Isaac** | [`isaac_scene_state_node`](../../src/robot_control/robot_control/isaac/scene_state_node.py) | ✅ | Isaac 이 내보내는 **물체 TF** (§6.3) |
| **Gazebo** | `gazebo_scene_state_node` | ❌ 미구현 | `ros_gz_bridge` 예정 |

**§6.1 은 모든 발행 노드에 공통**이고, §6.2 는 mock, §6.3 은 Isaac 이다. 이 노드들이 **무엇을 싣고 무엇을 빼는지**는 §7 에 있다.

### 6.1 공통 — 모든 발행 노드에 적용

**구독은 이벤트, 발행은 주기다 (기본 2 Hz).**

1. **백엔드마다 성격이 다르다** — 물리 백엔드는 물체가 계속 움직여 주기 발행이 자연스럽다. mock 만 이벤트성이면 트윈의 `staleness_ms` 를 백엔드별로 달리 잡아야 한다.
2. **발행자가 죽은 것을 감지할 수 있다.** 이벤트 발행이면 scene 이 조용히 얼어붙은 것과 정상인 것이 구분되지 않는다.

**`/scene/objects` 는 TRANSIENT_LOCAL 로 발행한다** — 늦게 뜬 트윈·recorder 가 현재 scene 을 즉시 본다.

**물체 하나가 실패하면 그 물체만 건너뛰고 경고한다.** 물체 하나 때문에 나머지 scene 전체를 발행하지 못하는 편이 더 나쁘다. mock 은 기하가 없거나 좌표 변환이 불가능할 때, Isaac 은 TF 조회에 실패할 때 이 정책을 쓴다.

> ⚠️ **읽기와 쓰기가 비대칭이다.** 발행 방향은 `mesh` 를 통과시키지만, `reset_scene` 방향은 `box`/`sphere`/`cylinder`/`cone`/`prism` 만 받고 `mesh` 는 `unsupported type 'mesh' for '<name>'` 로 거부한다. 또한 **메시지에 메시 기하를 실을 필드가 없다** (경로·URI·정점 어느 것도 없다) — 즉 현재 `type: 'mesh'` 는 "primitive 가 아니다" 이상의 정보를 담지 못한다.

### 6.2 mock — `mock_scene_state_node`

MoveIt planning scene 을 원본으로 삼는다. 아래는 **이 노드에만 해당한다.**

#### 6.2.1 서비스 폴링이 아니라 diff 누적이다

`/monitored_planning_scene` 토픽을 구독한다. 토픽은 **diff** 를 실어 나르므로(변경된 물체만, `is_diff=True`) **전체 상태는 노드가 누적해서 들고 있어야 한다.**

> **`/get_planning_scene` 서비스 폴링으로 되돌리지 않는다.** 처음에는 그 방식이었으나 **간헐적으로 빈 world 를 반환**했다 — 물체가 있는 상태에서 1초 간격 10회 조회에 `[3,0,0,0,0,0,3,3,3,0]`. 요청 컴포넌트 마스크를 바꿔도, 다른 조회자를 모두 없애도, RViz 를 종료해도 같았다. `move_group` 의 planning scene monitor 가 diff scene 과 parent 를 오가는 것으로 보인다. 토픽 구독은 이 불안정과 무관하다.

#### 6.2.2 구독 QoS 는 VOLATILE 이어야 한다

`/monitored_planning_scene` 은 `move_group` 이 **VOLATILE** 로 발행한다. TRANSIENT_LOCAL 로 구독하면 QoS 불일치로 **한 건도 받지 못한다** — 경고만 뜨고 조용히 멈춘다. 발행 쪽(`/scene/objects`, TRANSIENT_LOCAL)과 **서로 다르다는 점**이 함정이다.

#### 6.2.3 좌표 변환은 합성이 두 번이다

```
base ← 오브젝트 프레임 (TF)  ∘  오브젝트 ← primitive (CollisionObject 내부)
```

**뒤쪽을 빼먹으면 오프셋을 가진 물체가 어긋난다.** 이 합성은 [pose_math.py](../../src/robot_control/robot_control/scene/pose_math.py) 가 담당하며, `tf2_geometry_msgs.do_transform_pose` 를 쓰지 않는다 — 배포판마다 시그니처가 달라 (Pose vs PoseStamped) 조용히 어긋날 수 있고, **ROS 없이 단위 테스트할 수 있어야** 하기 때문이다. 쿼터니언 순서 오류는 크래시가 아니라 '그럴듯하게 틀린 자세'로 나타나므로 테스트로 고정해 두는 값이 크다.

> Isaac 에는 이 합성이 없다 — TF 조회 한 번으로 끝난다 (§6.3).

#### 6.2.4 기하가 하나가 아닐 때

- `CollisionObject` 에 primitive 가 **여러 개**면 첫 기하만 발행하고 경고를 남긴다 (계약은 물체 하나에 기하 하나다). 조용히 넘기지 않는 이유는 크기가 실제와 달라지기 때문.
- primitive 가 없고 **mesh 만** 있으면 `type: 'mesh'`, `dimensions: []` 로 나간다.
- 둘 다 없거나 좌표 변환이 불가능하면 그 물체만 건너뛴다 (§6.1 의 공통 정책).

#### 6.2.5 pose 는 ground truth 가 아니다

planning scene 의 물체는 **물리를 갖지 않는다.** 누가 넣은 값 그대로 있으며, 파지에 실패해도 굴러떨어져도 변하지 않는다. 따라서 **mock 에서 수집한 에피소드로는 성패를 관측할 수 없고**, 이 노드의 값은 변수·연산·기록 경로가 도는지 확인하는 **배관 검증용**이다.

실제 파지 판정은 Gazebo 백엔드 연동 이후다.

### 6.3 Isaac — `isaac_scene_state_node`

`panda_isaac.launch.py` 가 `create_scene_node()` 로 띄운다.

**원본이 TF 다.** Isaac 은 물체 pose 를 `ROS2PublishTransformTree` 로 내보내고, 이 노드가
`lookup_transform` 으로 받아 계약으로 바꾼다.

**TF 를 경유하는 것이 이 설계의 핵심이다.** `SceneObjects` 는 pose 를 `panda_link0` 기준으로 요구하는데 Isaac 은 world 기준이고, 게다가 쿼터니언이 **wxyz** 다. TF 를 타면 조회 한 번으로 **좌표 변환과 쿼터니언 규약이 동시에 해결된다.**

```python
# tf2 가 이미 ROS 규약(xyzw)으로 준다. 여기서 뒤집지 않는다.
obj.pose.orientation = transform.transform.rotation
```

§3.4 의 wxyz 함정을 "조심해서 변환한다"가 아니라 **변환 코드를 아예 없애서** 풀었다. 순서를 틀려도 norm 이 1 이라 어떤 검사도 통과하므로, 애초에 그 코드를 안 쓰는 편이 낫다는 판단이다.

**이름·종류·크기는 TF 에 없다.** 그래서 시뮬레이터 쪽 생성 스크립트와 **같은 JSON** (`config/isaac_scene.json`)을 읽는다 — 두 곳에 적으면 조용히 어긋난다.

TF 조회에 실패한 물체는 **그 물체만 빠지고** 나머지는 발행된다 (§6.1 의 공통 정책).

## 7. 무엇이 실리는가 — 조작 대상뿐이다

**`/scene/objects` 에 실리는 것은 전부 조작 대상이다.** 탁자·펜스 같은 환경 물체는 들어오지 않는다.

발행 노드가 걸러낸다. Isaac 은 `isaac_scene.json` 의 **`dynamic: true` 인 것만** 싣고, mock 은 `/scene/reset` 으로 놓인 것만 갖는다(레시피에 환경 물체가 없다). 구분자를 새로 만들지 않고 기존 `dynamic` 플래그를 그대로 쓴다 — 물리적으로 움직이지 않는 물체는 pose 가 변하지 않아 **상태 채널에 실을 값이 없고**, 그래서 "rigid body 인가"와 "조작 대상인가"가 같은 집합이 된다.

> **빠지는 것은 발행뿐이다.** 탁자는 시뮬레이터에 그대로 있다 — `setup_scene.py` 가 같은 JSON 으로 prim 을 만든다. 물리적으로 존재하고 블록을 받쳐 준다.

### MoveIt 은 이 물체들을 모른다

**planning scene 에 아무것도 넣지 않는다.** 2026-09-01 결정이며, 그 전까지 있던 `planning_scene_sync` 노드와 `SceneObject.fixture` 필드는 이때 함께 삭제됐다.

근거는 이렇다. 조작 대상을 planning scene 에 넣으면 **파지 자세가 시작 자세 충돌이 되어** 관절공간 계획이 `INVALID_MOTION_PLAN`(-2)으로 거부된다. 그래서 넣을 수 있는 것은 환경 물체뿐인데, 위처럼 환경 물체를 아예 발행하지 않기로 하면 **넣을 것이 하나도 남지 않는다.** 매 tick 빈 diff 를 내는 노드가 되므로 없앴다.

> ⚠️ **감수하는 위험 — 관절공간 이동 중 팔이 탁자를 통과한다.**
>
> 이건 가설이 아니라 관측이다. 예전에 Isaac 에서 손가락이 상판 안에 박혀 그리퍼가 물리적으로 막혔고, 그것을 "그리퍼 고장"으로 오독한 이력이 있다. **단순성을 위해 감수하기로 한 것**이며(2026-09-01), 되살릴 근거가 생기면 아래 대안을 검토한다.
>
> 다만 노출 범위는 좁다.
>
> - **자기충돌 검사는 그대로 동작한다** — 로봇 링크끼리의 충돌은 world 물체와 무관하게 SRDF 의 ACM 으로 검사된다.
> - **파지 동작은 영향이 0 이다** — 접근·하강·상승이 전부 cartesian 인데 `GetCartesianPath` 는 `avoid_collisions` 기본값이 `False` 라 애초에 planning scene 을 보지 않았다.
>
> 실제로 잃는 것은 **`move_to_named_target` / `move_to_joints` 중의 탁자 회피**뿐이다. 되살릴 때의 후보는 (1) 안전한 경유 자세를 SRDF `group_state` 에 정의, (2) 탁자만 launch 시점에 상자 하나로 planning scene 에 정적 등록, (3) 작업공간 z 하한을 `PositionConstraint` 로 강제. **(2)가 이 설계와 가장 잘 맞는다** — scene 채널을 건드리지 않고 환경 설정으로만 다룬다.

### 자동 라벨링은 지지면을 데이터에서 볼 수 없다

`/scene/objects` 에 탁자가 없으므로, "블록이 탁자 위에 놓였는가"를 판정하려면 지지면의 크기·위치를 **데이터 밖에서** 가져와야 한다. 그 출처가 바뀌면 라벨이 조용히 틀린다. 필요해지면 에피소드 `metadata`(`sessions.metadata` jsonb)에 초기 배치와 함께 한 번 남기는 방법이 있다 — 매 tick 필드를 두지 않으면서 근거를 데이터 안에 보존한다.

## 8. 트윈에서 보기

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

### 연산 `reset_scene`

| 항목 | 값 |
|---|---|
| `kind` | `sync` (`sync_timeout_sec: 10.0`) |
| `resource` | **`[scene, arm]`** — 팔이 움직이는 중이면 `409 RESOURCE_BUSY` |
| 입력 | `scene`(필수, 레시피 이름) · `seed`(선택) |
| 출력 | `scene`, `seed`, **`objects`(실제 배치)**, `applied_count` |
| 백엔드 | `service: /scene/reset` · `service_type: rdfp_msgs/srv/ResetScene` |

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

`grasp_pose_of_object` 는 `reset_scene` 의 `outputs.objects` 원소를 그대로 받아 **물체 중심을 위에서 잡는** 자세를 만든다. 돌려주는 좌표는 **손끝(TCP) 기준**이며, 팔에 명령할 때 `to_arm_command()` 로 `panda_link8` 기준으로 바꾼다 — 손끝이 거기서 약 10.3 cm 더 뻗어 있어, 보정을 빠뜨리면 5 cm 큐브(중심 z=0.025)를 집을 때 손끝이 z = -0.078 로 **바닥을 파고든다.**

---

## 9. launch 와 데이터셋

### launch 인자 (YAML 을 거치지 않는다)

**이름은 백엔드와 무관하게 같다.** `enable_scene` 하나로 어느 스택에서든 scene 노드를 켜고 끈다 — 스택을 바꿔도 명령줄을 고칠 필요가 없다.

| 인자 | 적용 | 기본 | 설명 |
|---|---|---|---|
| `enable_scene` | 전 스택 | 아래 참조 | 백엔드 scene 상태 노드 기동 여부 |
| `scene_publish_rate` | 전 스택 | `2.0` | `/scene/objects` 발행 Hz |

`enable_scene` 의 기본값만 launch 마다 다르다:

| launch | 기본 | 왜 |
|---|---|---|
| `panda_mock` · `panda_jgpc_mock` · `rdfp_panda_*` | `true` | 아래 "기본 on 은 의도적이다" |
| `rdfp_panda_isaac` | `true` | `/scene/objects` 는 데이터셋 채널이므로 수집 스택에서는 켜져 있어야 한다 |
| `panda_isaac` | **`false`** | Isaac 백엔드는 단계별로 올린다 — Phase 3 에 도달하기 전에는 물체 TF 자체가 없어서, 켜 두면 조회 실패 경고만 쌓인다 |

- **기본 on 은 의도적이다.** 노드는 2 Hz 타이머 하나를 쓰고 `/scene/reset` 요청이 오기 전까지 아무것도 바꾸지 않는 반면, 꺼 두면 트윈의 `reset_scene` 이 **서비스를 찾지 못해** 실패하면서 로그에 원인이 남지 않는다.
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

## 10. 함정 모음

| 증상 | 원인 |
|---|---|
| 토픽은 있는데 값이 한 건도 안 온다 | 구독 durability 불일치. TRANSIENT_LOCAL 발행을 volatile 로 구독하면 매칭 자체가 안 된다 |
| 물체가 데이터셋의 어느 에피소드에도 없다 | `header.stamp` 가 비어 epoch 0 으로 적재됨 |
| 백엔드마다 좌표가 다르다 | `frame_id` 를 로봇 베이스로 변환하지 않음. 변환은 발행 노드 책임 |
| 자세가 '그럴듯하게' 틀리다 | 쿼터니언 wxyz ↔ xyzw 혼동. norm 검사로는 안 잡힌다 — 알려진 비대칭 회전으로 확인 |
| cylinder 크기가 이상하다 | `dimensions` 가 `[높이, 반지름]` 순서다 |
| 오프셋 가진 물체만 어긋난다 | `오브젝트 ← primitive` 합성 누락 (§6.2.3) |
| `reset_scene` 이 타임아웃으로 실패 | `enable_scene:=false` 로 노드가 없음 |
| `reset_scene` 이 409 | `scene` 뿐 아니라 `arm` 도 점유한다 — 팔 이동 중에는 거부 |
| scene 이 조용히 얼어붙었다 | 주기 발행이므로 `staleness_ms` 로 감지된다 (이벤트 발행이면 구분 불가) |
| mock 에서 파지 성패가 안 잡힌다 | 물리가 없어 pose 가 변하지 않는다. 배관 검증용이다 |
| Isaac 인데 물체가 안 보인다 | TF 가 안 오거나 `isaac_scene.json` 에 없는 물체다 — 그 물체만 빠진다 (§6.3) |
| MoveIt 이 테이블을 뚫는다 | **설계상 그렇다** (§7). planning scene 에 아무것도 넣지 않기로 했다 — 단순성을 위해 감수한 위험이다 |
| 곰 인형 같은 물체를 배치할 수 없다 | `reset_scene` 은 primitive 만 받는다. `mesh` 는 읽기 전용이며 기하를 실을 필드도 없다 (§6.1) |

---

## 11. 파일 위치

| 구성요소 | 경로 |
|---|---|
| 메시지 정의 (정본) | `src/rdfp_msgs/msg/SceneObject.msg` · `SceneObjects.msg` |
| 서비스 정의 (정본) | `src/rdfp_msgs/srv/ResetScene.srv` |
| 발행 노드 (mock) | [scene/mock_scene_state_node.py](../../src/robot_control/robot_control/scene/mock_scene_state_node.py) |
| 발행 노드 (Isaac) | [isaac/scene_state_node.py](../../src/robot_control/robot_control/isaac/scene_state_node.py) + `config/isaac_scene.json` |
| pose 합성 (ROS 무의존) | [scene/pose_math.py](../../src/robot_control/robot_control/scene/pose_math.py) |
| launch 헬퍼 | [launch_helpers/scene.py](../../src/robot_control/robot_control/launch_helpers/scene.py) |
| 트윈 설정 | `src/robot_twin/config/robot_twin_panda01.yaml` (변수 2개 + `reset_scene`) |
| 트윈 핸들러 | [backends.py](../../src/robot_twin/robot_twin/backends.py) `_sample_scene` / `_reset_scene` |
| 녹화 대상 | [config/recording_topics.list](../../config/recording_topics.list) |
| DB 적재 | [writers/scene_objects.py](../../src/rdfp/rdfp/dataset/db/writers/scene_objects.py) · [readers/scene_objects.py](../../src/rdfp/rdfp/dataset/db/readers/scene_objects.py) · [sql/schema.sql](../../src/rdfp/rdfp/dataset/sql/schema.sql) |
| 테스트 | `src/robot_control/robot_control/scene/tests/` (`test_mock_scene_state_node.py` · `test_pose_math.py`) |

---

## 12. 참고

- [robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) — §4.1 변수 응답 형식, §4.2 `reset_scene`, §4.4 자원 락
- [auto_episode_collection_draft.md](../robot_twin/auto_episode_collection_draft.md) — §2 타입 설계 경위(`PoseArray` 를 쓸 수 없던 이유), §3 `reset_scene` 결정, §2.5 좌표 규약 어댑터, §2.6 diff 누적으로 바꾼 실측
- [scene_objects_observation_decision.md](../rosbag2/scene_objects_observation_decision.md) — 학습 입력에 넣을 것인가 (미결)
- [multi_simulator_backend_design.md](../simulation/multi_simulator_backend_design.md) — §5 백엔드 계약. 이 토픽은 그 §5.3 의 "환경 오브젝트"(선택)를 계약으로 승격한 것이다
- [robot_control/launch/README.md](../../src/robot_control/launch/README.md) — §5 헬퍼 인벤토리
- [rdfp/launch/README.md](../../src/rdfp/launch/README.md) — scene 인자가 YAML 밖에 있는 이유

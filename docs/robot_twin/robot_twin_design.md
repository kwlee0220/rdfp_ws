# 로봇 트윈 (Robot Twin) 설계서

> **상태: 설계안 — 코드 미반영.**
>
> 설계 쟁점은 **A-12(관측성)·A-16(실기 안전) 두 건을 제외하고 모두 결정되어 본문에
> 반영**되었다. 두 부록은 결정 이력과 잔여 항목을 담는다.
>
> - **부록 A** — 설계 쟁점. 잔여 2건 (모두 "지금 정할 필요 없음"으로 분류)
> - **부록 B** — extern_op 연계 쟁점. **전건 결정 완료.** 말미의 확장·미채택 요소
>   요약표는 extern_op 확장 명세의 초안으로 사용할 수 있다.

## 1. 목적과 범위

### 1.1 목적

ROS 2 를 직접 사용하기 어려운 환경에서, **RESTful/HTTP 인터페이스만으로 로봇의
상태를 조회하고 제한된 제어를 수행**할 수 있게 하는 게이트웨이를 정의한다.

제어 대상 로봇 1대에 트윈 1개가 대응하며, 각 트윈은 0~n 개의 **상태 변수**와
0~n 개의 **연산**으로 구성된다.

### 1.2 범위

| 포함 | 제외 |
|---|---|
| 상태 변수의 polling 기반 조회 | 이벤트/메시지 push (SSE, WebSocket) |
| 미리 구현된 연산의 원격 실행 | 임의 ROS 토픽/서비스의 일반 프록시 |
| 장시간 연산의 비동기 세션 관리 | 실시간 제어 (servo, teleop 루프) |
| 트윈 단위 안전 정책 (자원 락, E-stop) | 영상 스트리밍 |
| extern_op 프로토콜 준수 | **AAS Submodel 표현** (아래 참조) |

**AAS Submodel 매핑은 하지 않는다.** 트윈은 extern_op 프로토콜을 따르는 REST 인터페이스만
제공하며, 변수/연산 카탈로그를 AAS Submodel 형태로 노출하지 않는다. MDT Platform 이
AAS 표현을 필요로 한다면 **플랫폼 쪽에서 본 REST 인터페이스를 감싸 변환**한다. 트윈은
AAS 를 모른다.

### 1.3 클라이언트 중립성

트윈은 REST 인터페이스만 제공하므로 **특정 언어에 바인딩되지 않는다.** MDT
Platform(Java)이 첫 소비자이지만, curl / Node-RED / 브라우저 / Python 클라이언트가
동일하게 사용할 수 있다.

이에 따라 다음이 설계 제약이 된다.

- **OpenAPI 스펙을 1급 산출물로 취급한다.** FastAPI 를 사용하면 자동 생성되며,
  `openapi-generator` 로 Java/TypeScript 클라이언트 스텁을 파생할 수 있다.
- JSON 직렬화 규약을 언어 중립적으로 고정한다 (5.5 참조). 특히 `int64` 정밀도와
  `NaN` 처리는 브라우저 클라이언트가 붙는 순간 실제 문제가 된다.
- 인증은 현 단계에서 도입하지 않는다 (8.4). 나중에 도입할 때도 언어 중립적인
  방식(Bearer token 등)을 택해 클라이언트 다양성을 해치지 않는다.

### 1.4 용어

| 용어 | 정의 |
|---|---|
| **트윈 (twin)** | 로봇 1대에 대응하는 게이트웨이 인스턴스 |
| **상태 변수 (variable)** | 로봇 상태의 조회 단위. ROS 소스 + 추출 규칙 + 출력 스키마로 정의 |
| **연산 (operation)** | 트윈이 제공하는 기능 단위. extern_op 의 "연산"과 동일한 개념이다 |
| **스냅샷 (snapshot)** | 특정 시점의 변수 값 + 수신 메타데이터를 담은 불변 객체 |
| **자원 (resource)** | 연산이 배타적으로 점유하는 로봇 부위 (`arm`, `gripper`) |
| **연산 세션 (operation session)** | **비동기 연산 1회 실행**에 대해 트윈이 유지하는 실행 컨텍스트. extern_op 의 "session" 과 같다 |

**⚠️ "세션" 이라는 말이 이 워크스페이스에서 두 가지를 뜻한다.**

| | 의미 | 범위 |
|---|---|---|
| **연산 세션** | 비동기 연산 1회 실행 컨텍스트 | 본 문서 전반 |
| `rdfp` 의 `/session` | `session_control_node` 의 `IDLE → IN_SESSION → IN_EPISODE` — **데이터 수집 세션** | 본 문서와 무관 |

둘은 이름만 같고 아무 관계가 없다. 트윈이 `rdfp` 패키지 안에 있으므로(2.5) 혼동하기
쉽다. 본 문서에서 **"세션"은 언제나 연산 세션**을 뜻하며, 데이터 수집 세션을 상태
변수로 노출할 경우 이름을 `data_session` 처럼 구분되게 둔다.

연산 세션이 담는 것: `status` · `phase` · `progress` · `outputs` · `error` · 시작 시각 ·
`created_by`(현재는 항상 `null` — 8.4 참조).

---

## 2. 아키텍처

### 2.1 배치 — 단일 Python 프로세스

트윈은 **Python 으로 구현하며, rclpy 와 HTTP 서버를 한 프로세스에 둔다.**

```
[REST 클라이언트 — 언어 무관]
        │  HTTP/JSON (extern_op 프로토콜)
┌───────┴─────────────────────────────────┐
│  로봇 트윈 (단일 Python 프로세스)         │
│                                          │
│   FastAPI/uvicorn  ←→  상태 스냅샷 캐시   │
│         │                     ↑          │
│         │              rclpy executor    │
│         └──────────→  rdfp MoveGroupClient│
└───────┬─────────────────────────────────┘
        │  DDS
   ROS 2 Humble (Panda + MoveIt2)
```

**rosbridge / gRPC 사이드카를 두지 않는다.** 트윈 자체가 ROS 노드이므로 중간
브리지가 하려던 일을 직접 수행한다. 이로써 얻는 것:

- MoveIt 액션 goal 을 JSON 으로 왕복시킬 필요가 없다 (브리지 방식의 가장 지저분한 부분).
- QoS 매칭, TF lookup, staleness 판정이 네이티브로 처리된다.
- 취소가 액션 클라이언트의 `cancel_goal_async()` 로 직결된다. 브리지를 거치면
  취소 전파가 가장 까다롭다.
- 오류가 Python 예외 / `MoveItErrorCodes` 로 그대로 들어와 `error.code` 매핑이 단순하다.

### 2.2 스레드 모델

`rclpy.spin()` 은 블로킹이므로 uvicorn 의 asyncio 루프와 같은 스레드에서 돌 수
없다. 다음 구조를 사용한다.

| 스레드 | 역할 |
|---|---|
| **ROS executor 스레드** | `MultiThreadedExecutor` 를 spin. 구독 콜백이 스냅샷을 생성해 캐시에 대입한다 |
| **HTTP 스레드 (asyncio)** | 캐시를 읽어 응답을 만든다. ROS 호출은 비동기 API 만 사용한다 |

**공유 상태 접근 규칙:**

값을 **불변 스냅샷 객체로 만들어 참조 하나만 대입**한다. CPython 에서 속성 참조
대입은 GIL 하에서 원자적이므로, 읽는 쪽은 항상 "이전 스냅샷 또는 새 스냅샷"을
얻고 찢어진 중간 상태를 볼 수 없다. **단일 변수 조회에는 락이 불필요하다.**

```python
# ROS 콜백 (executor 스레드)
def _on_joint_states(self, msg) -> None:
    # 기존 객체를 제자리에서 수정하지 않는다 — 새 스냅샷을 만들어 참조만 교체한다.
    self._vars['joint_states'] = _Snapshot(msg, received_at=time.time(), gen=next(self._gen))
```

**락이 필요한 지점은 하나뿐이다:** 여러 변수를 한 번에 읽어 반환하는 배치 조회.
이 경우 캐시 dict 전체를 통째로 교체하거나 읽기를 락으로 감싼다. 락은 **읽는 순간에
여러 변수가 서로 뒤섞이지 않게** 할 뿐이며, 각 값의 생성 시각을 맞춰 주지는 않는다
(5.4 의 보장 수준 참조).

**변환은 지연 수행한다.** rclpy 는 콜백마다 메시지를 새 객체로 역직렬화해 넘기므로
(rclcpp 의 loaned message 와 다르다) 메시지 참조를 그대로 보관해도 안전하다.
콜백에서 dict 로 변환하면 50 Hz 로 아무도 보지 않는 값을 변환하느라 GIL 을 낭비한다.
JSON 변환은 HTTP 스레드에서 수행하되 8.1 의 메모이즈를 적용한다.

### 2.3 연산 실행 시 데드락 회피 ⚠️

`MoveGroupClient` 의 **동기 메서드를 HTTP 요청 스레드에서 호출하면 안 된다.**
콜백 그룹 구성에 따라 executor 스레드를 기다리다 멈출 수 있다.

- `move_to_named_target_async` / `follow_trajectory_async` / `plan_*_async` 계열만 사용한다.
- 이들 메서드의 **`externally_spun=True` 인자를 반드시 전달한다.** executor 가 다른
  스레드에서 Node 를 spin 중임을 알리는 플래그다
  ([move_group_client.py:396](../../src/robot_control/robot_control/moveit/move_group_client.py#L396)).
- 완료 판정은 트윈의 세션 상태 머신이 `Future` 콜백으로 수행한다. extern_op 이
  비동기 세션 모델이므로 프로토콜과도 자연스럽게 맞는다.

### 2.4 프로세스 경계 — 1 트윈 = 1 프로세스

`ROS_DOMAIN_ID` 는 프로세스 환경변수이므로, 서로 다른 도메인의 로봇 n대를 한
프로세스에서 다루기 어렵다. **트윈 1개 = 프로세스 1개**를 원칙으로 한다.

- 같은 머신에서 트윈 여러 개를 띄울 경우 ROS 노드 이름을 트윈 id 기반으로 고유화한다
  (예: `robot_twin_panda01`).

#### 다중 인스턴스는 포트 분리로 배치한다

**트윈마다 별도 HTTP 포트를 쓴다.** 앞단에 리버스 프록시를 두어 경로로 라우팅하는 방식은
채택하지 않는다.

| | 포트 분리 (채택) | 프록시 라우팅 |
|---|---|---|
| 구성 요소 | 트윈 프로세스만 | 프록시가 추가된다 |
| 장애점 | 트윈별로 독립 | **프록시가 단일 장애점** |
| 배포 | 프로세스 하나 더 띄우면 끝 | 프록시 설정도 함께 갱신 |

트윈이 이미 프로세스 단위로 격리되어 있으므로(위), 포트도 그에 맞추는 편이 구조적으로
일관되고 장애 격리도 유지된다. 프록시는 TLS 종단이나 통합 엔드포인트가 필요해질 때
**앞에 덧붙이면 되며**, 그때도 트윈 쪽 변경은 없다 (부록 A-14).

**URL 의 `{twin}` 세그먼트는 유지한다.** 한 프로세스가 트윈 하나만 서비스하므로 중복처럼
보이지만, 나중에 프록시를 앞에 두어도 클라이언트 URL 이 그대로 유지된다. 다만 결과적으로
**`GET /api/v1/robot_twins` 는 항상 자기 자신 하나만 반환한다** — 이 엔드포인트는 트윈
레지스트리가 아니며, 여러 트윈을 한 번에 조회하는 수단은 제공하지 않는다.

### 2.5 패키징 — `rdfp` 패키지 내부에 둔다

트윈은 **`rdfp` 패키지의 `twin/` 서브패키지**로 구현한다. 별도 ROS 패키지나 순수 pip
패키지로 분리하지 않는다.

| 항목 | 위치 |
|---|---|
| 소스 | `src/robot_twin/robot_twin/` |
| 진입점 | `ros2 run robot_twin robot_twin --config <yaml>` (`setup.py` console_scripts) |
| 트윈 정의 YAML | `src/rdfp/config/` (setup.py 의 `config/*` glob 이 share 로 설치) |
| 테스트 | `src/robot_twin/robot_twin/tests/` |

**근거**

- 트윈은 `rclpy` 를 쓰는 **ROS 노드**다. ROS 환경 없이는 어차피 돌지 않으므로, 순수 pip
  패키지로 두어 얻을 이점이 없다.
- 별도 ROS 패키지로 분리해도 `MoveGroupClient` 때문에 `rdfp` 를 함께 설치해야 한다.
  ament 패키지는 부분 설치가 불가능하므로 `python3-opencv` / `ffmpeg` 도 따라온다 —
  의존성 분리 효과가 크지 않다.
- `dataset/` 이 이미 `pydantic` / `psycopg` 같은 pip 전용 의존성을 `package.xml` 밖에
  두고 README 에 문서화하는 선례를 만들었다. 웹 스택도 같은 방식으로 처리한다.
- **이 결정은 되돌리기가 싸다.** 디렉터리 이동 + `setup.py` 엔트리 이동이면 분리된다.
  지금 패키지를 늘려 빌드·문서·버전을 셋으로 가르는 비용을 미리 치를 이유가 없다.

**분리 가능성을 유지하기 위한 경계 규칙** — 아래를 지키면 나중 분리가 기계적 작업으로
남는다.

| 규칙 | 이유 |
|---|---|
| 트윈은 **`rdfp.moveit` 만 import 한다** | `camera` / `recorder` / `dataset` / `rosbag` 에 의존하는 순간 분리가 어려워진다 |
| `rdfp` 의 기존 코드는 **`rdfp.twin` 을 import 하지 않는다** | 단방향 의존이라야 떼어낼 수 있다 |
| FastAPI / uvicorn 은 `package.xml` 이 아니라 **README 의 pip 의존성 절**에 기재한다 | 기존 관례 준수. `rosdep` 이 잡지 못하는 의존성임을 명시 |
| 테스트는 ROS 불필요 부분과 필요 부분을 분리한다 | 스냅샷 캐시 · 직렬화 · 세션 상태 머신은 ROS 없이 검증 가능하다. ROS 의존 테스트는 `pytest.importorskip` 으로 자체 skip |

**분리 트리거** — 다음 중 하나가 실제로 발생하면 그때 `robot_twin` 으로 옮긴다.

- 트윈을 별도 Docker 이미지로 배포해야 할 때
- 트윈이 `rdfp` 와 다른 릴리스 주기를 가져야 할 때
- `rdfp` 를 설치할 수 없는 환경(다른 벤더 스택)에 트윈을 올려야 할 때

### 2.6 기동 순서와 MoveGroup 클라이언트 생성

#### `mode='auto'` 를 금지한다 ⚠️

`create_move_group_client()` 의 `mode='auto'` 판별은 `/panda_arm_controller/commands`
토픽의 존재 여부를 보는 **토픽 그래프 lookup** 이다. **DDS 디스커버리가 안정되기 전에
호출하면 JGPC 스택을 JTC 로 오판한다.**

트윈은 `move_group` 이나 컨트롤러보다 먼저 뜰 수 있으므로 이 위험에 정면으로 노출된다.
게다가 오판은 예외를 던지지 않고 **조용히 잘못된 클라이언트를 반환**하므로, 증상이
"이동 명령이 아무 효과가 없다"로 나타나 원인 추적이 어렵다.

→ **트윈 정의 YAML 의 `moveit.move_group_mode` 를 필수 항목으로 두고 `'jtc'` / `'jgpc'`
만 허용한다.** `'auto'` 나 미지정은 **설정 로드 시점에 오류**로 처리해 기동을 거부한다.

```python
# 트윈은 항상 mode 를 명시해 생성한다. auto 로 폴백하지 않는다.
client = create_move_group_client(node, mode=config.moveit.move_group_mode)
```

배포 대상 스택을 모르는 경우는 실질적으로 없으며, 자동 판별로 얻는 편의보다 오판의
비용이 크다.

#### HTTP 서버를 ROS 준비보다 먼저 띄운다

ROS 클라이언트 초기화 실패가 **프로세스 기동 실패로 이어져서는 안 된다.** 그러면
`GET /health` 로 "아직 준비되지 않음"을 알릴 수단조차 사라진다.

| 단계 | 동작 |
|---|---|
| 1 | 설정 로드 및 검증 (여기서 `move_group_mode` 미지정이면 즉시 실패) |
| 2 | rclpy 초기화, 노드 생성, 변수 구독 등록, executor 스레드 시작 |
| 3 | **HTTP 서버 기동** — 이 시점부터 `/health` 응답 가능 |
| 4 | MoveGroup 클라이언트 생성 (백그라운드, 실패해도 프로세스는 유지) |

4단계가 완료되기 전의 연산 요청은 **`503` + `PRECONDITION_FAILED`** 로 거부하고(6.8),
`GET /health` 가 그 사실을 드러낸다. 상태 변수 조회는 4단계와 무관하게 동작한다 — 구독은 2단계에서
이미 걸려 있고, 퍼블리셔가 없으면 `SOURCE_UNAVAILABLE` 로 표현된다.

---

## 3. 트윈 정의 (선언적 설정)

변수와 연산을 코드에 하드코딩하면 추가가 곧 코드 수정 + 재배포가 된다. **YAML 로
선언**하고 런타임에 로드한다. 스키마 자동 검증과 OpenAPI 자동 생성이 따라온다.

```yaml
twin:
  id: panda01
  description: Franka Emika Panda (mock)

http:
  host: 0.0.0.0                      # 8.4 참조 — 노출 범위는 망 수준에서 통제한다
  port: 8801                         # 트윈마다 다른 포트 (2.4)
  tls: false

ros:
  domain_id: 31
  rmw: rmw_fastrtps_cpp
  node_name: robot_twin_panda01
  use_sim_time: false

moveit:
  # 필수. 'jtc' 또는 'jgpc' 만 허용하며 'auto' 는 금지한다 (2.6 참조).
  move_group_mode: jtc
  planning_group: panda_arm

variables:
  - name: joint_states
    source:
      type: topic
      topic: /joint_states
      msg: sensor_msgs/msg/JointState
      qos: { reliability: reliable, durability: volatile, depth: 1 }
    projection: joint_state_map       # 배열 → {joint_name: value} 정규화
    staleness_ms: 500                 # 초과 시 quality=STALE
    units: { position: rad, velocity: rad/s, effort: Nm }

  - name: ee_pose
    source:
      type: topic                     # 또는 type: tf (base/tip 지정)
      topic: /ee_pose
      msg: geometry_msgs/msg/PoseStamped
      qos: { reliability: reliable, durability: volatile, depth: 1 }
    staleness_ms: 200
    units: { position: m, orientation: quaternion_xyzw }

  - name: named_targets                # SRDF 정적 데이터 — 5.7 참조
    source:
      type: static
      backend: { type: rdfp, method: get_all_named_targets }
      timeout_sec: 2.0                 # lazy 조회 시 상한
    staleness_ms: null                 # 런타임 불변 — staleness 검사 안 함

operations:
  - name: move_to_named_target
    kind: async                        # 연산별 고정 — 6.2 참조
    resource: arm                      # 배타 점유 자원
    backend: { type: rdfp, method: move_to_named_target_async }
    inputs_schema:                     # JSON Schema — 400 검증에 그대로 사용
      type: object
      required: [target]
      properties:
        target: { type: string }
        velocity_scaling: { type: number, minimum: 0.01, maximum: 1.0, default: 0.1 }
        max_duration_sec: { type: number, minimum: 1, maximum: 120, default: 60 }
      additionalProperties: false
    default_timeout_sec: 60

```

---

## 4. 상태 변수 — 값 획득

### 4.1 스냅샷 캐시

트윈은 정의된 모든 변수의 소스를 **기동 시점부터 상시 구독**하고, 수신할 때마다
스냅샷을 갱신한다. 조회 요청은 ROS 를 건드리지 않고 캐시만 읽는다.

스냅샷은 다음을 포함한다.

| 필드 | 설명 |
|---|---|
| `msg` | 원본 ROS 메시지 참조 (변환하지 않음) |
| `stamp` | `msg.header.stamp` (헤더가 없는 타입이면 `null`) |
| `received_at` | **ROS 콜백에서 찍은 wall clock 수신 시각** |
| `gen` | 단조 증가 세대 번호. ETag / 직렬화 메모이즈 키로 사용 |

### 4.2 시각 처리 ⚠️

**`age_ms` 는 항상 wall clock `received_at` 기준으로만 계산한다.**

`use_sim_time` 환경에서는 `msg.header.stamp` 가 sim clock, 수신 시각은 wall clock
이다. 두 시계를 섞어 age 를 계산하면 값이 무의미해진다. mock 스택과 replay 스택에서
특히 문제가 된다.

- `stamp` 는 원본을 가공 없이 그대로 전달한다.
- 어느 시계 기준인지 변수 메타(`GET /variables`)에 표기한다.
- `stamp` 를 신뢰하려면 로봇과 트윈의 NTP 동기가 전제다.

### 4.3 소스 타입

"상태 변수 = ROS 토픽"으로 고정하면 변수가 몇 개 늘어나는 순간 막힌다. 실제로
초기 변수 중 절반이 이미 이 가정을 벗어난다 (5.7 참조).

| `source.type` | 갱신 방식 | 용도 |
|---|---|---|
| `topic` | push (구독 콜백) | 가장 일반적 |
| `tf` | 주기 lookup | `base` → `tip` 변환을 `PoseStamped` 로 구성 |
| `service` | 주기 호출 | 토픽이 없고 서비스만 있는 상태 (호출 주기 지정 필요) |
| `static` | **lazy 1회 조회 + 캐시** | 런타임에 변하지 않는 데이터 (SRDF named target, planning group 등) |
| `derived` | 다른 변수에서 계산 | 파생 규칙 지정 |

**`static` 소스의 갱신 정책**

- **기동 시점에 즉시 조회하지 않는다.** 트윈이 `move_group` 보다 먼저 뜨면 실패한다.
  **최초 조회 요청 시 lazy 하게 가져와 무기한 캐시**한다.
- **자동 무효화를 하지 않는다.** `move_group` 노드의 그래프 이탈/재등장을 감지하지
  않으므로, 백엔드가 재시작되어 SRDF 가 바뀌어도 캐시된 옛 값이 계속 반환된다.
- 갱신 수단은 두 가지다: **`?refresh=true` 강제 갱신**, 또는 **트윈 재시작**.

자동 무효화를 넣지 않는 이유는 SRDF 가 런타임에 바뀌는 일이 사실상 없고, 그래프 이벤트
감시가 그 희소한 경우 대비로는 과하기 때문이다. 필요해지면 확장한다 (10.3).

---

## 5. 상태 변수 — 조회

### 5.1 품질 (quality)

모든 조회 응답은 값과 함께 **품질 판정**을 반환한다.

| quality | 의미 | 판정 근거 |
|---|---|---|
| `OK` | 값이 신선하다 | `age_ms <= staleness_ms` (또는 staleness 검사 없음) |
| `STALE` | 값은 있으나 오래됐다 | `age_ms > staleness_ms` |
| `NO_DATA` | 아직 한 번도 값을 얻지 못했다 | 스냅샷 없음 |
| `SOURCE_UNAVAILABLE` | 소스 자체를 사용할 수 없다 | 아래 표 참조 |
| `ERROR` | 값은 얻었으나 변환/추출에 실패했다 | 변환 예외 |

소스 타입이 여럿이므로(4.3) `SOURCE_UNAVAILABLE` 의 구체적 사유는 `reason` 필드로
구분한다. 품질 값 자체는 하나로 유지해 클라이언트 분기를 단순하게 둔다.

| `source.type` | 판정 방법 | `reason` |
|---|---|---|
| `topic` | `count_publishers(topic) == 0` | `NO_PUBLISHER` |
| `tf` | TF lookup 예외 | `TF_LOOKUP_FAILED` |
| `service` | 서비스 서버 미발견 | `SERVICE_UNAVAILABLE` |
| `static` | 조회 대상 노드(`move_group` 등) 미기동 | `SOURCE_NODE_DOWN` |

**세 가지 주의점:**

1. **`SOURCE_UNAVAILABLE` 은 나이로 판정할 수 없다.** 퍼블리셔가 없는 것과 퍼블리셔가
   멈춘 것은 age 만 보면 동일하다. `node.count_publishers()` 등을 주기적으로 확인해야
   구분되며, 이 구분이 있어야 클라이언트가 "로봇이 안 떠 있음"과 "로봇에 문제 있음"을
   나눌 수 있다.

2. **staleness 임계값은 변수별이며 끌 수 있어야 한다.** `/session` 처럼 상태 변화
   시에만 발행되는 TRANSIENT_LOCAL 토픽은 값이 오래된 것이 정상이다. 일괄
   staleness 규칙을 적용하면 정상 값을 STALE 로 오판한다. `staleness_ms: null` 이면
   검사하지 않는다.

3. **`quality: OK` 는 "로봇을 쓸 수 있다"는 뜻이 아니다.** `joint_state_broadcaster`
   만 살아 있으면 `joint_states` 는 정상적으로 흐르지만 컨트롤러가 없어 로봇은
   움직이지 않는다. 변수 품질은 "이 값이 믿을 만한가"만 답하고, "연산을 받을 수
   있는가"는 `GET /health` 가 답한다. **클라이언트는 둘 다 확인해야 한다.**

### 5.2 응답 envelope

값만 벗겨서 주는 옵션(`?raw=true`)은 제공하지 않는다. 클라이언트가 staleness
검사를 건너뛰게 만드는 지름길이기 때문이다.

```jsonc
GET /api/v1/robot_twins/panda01/variables/joint_states

200 OK
ETag: "1043"
Cache-Control: no-cache, must-revalidate
{
  "name": "joint_states",
  "quality": "OK",
  "stamp": { "sec": 1785664613, "nanosec": 206514897 },
  "received_at": "2026-08-08T12:47:03.412Z",
  "age_ms": 18,
  "schema_version": 1,
  "value": {
    "position": { "panda_joint1": 0.0012, "panda_joint2": -0.7841 },
    "velocity": { "panda_joint1": 0.0, "panda_joint2": 0.0 }
  }
}
```

값이 없는 경우에도 **HTTP 는 `200`** 이다.

```jsonc
{ "name": "joint_states", "quality": "NO_DATA", "value": null }
```

`404` 는 **"그런 변수가 정의되어 있지 않다"** 에만 사용한다. 변수는 있고 값만 없는
상황과 구분해야 한다.

### 5.3 오류 처리 — 부분 실패

변수 하나의 변환이 실패했다고 배치 응답 전체를 `500` 으로 죽이면 안 된다.

- HTTP 는 `200` 을 유지하고, **개별 변수의 `quality: "ERROR"` 로 실패를 표현**한다.
- 단일 조회에도 동일하게 적용한다. 메시지 필드가 예상과 다르면 `500` 이 아니라
  `200 + quality: ERROR + error{code,message}` 다.
- 이는 "프로토콜 오류는 4xx/5xx, 데이터 문제는 body 로" 라는 extern_op 원칙과 일치한다.

### 5.4 배치 조회

```
GET /api/v1/robot_twins/panda01/variables?names=joint_states,ee_pose
```

폴링 왕복 수를 줄이고, 클라이언트가 여러 변수를 한 번에 받게 한다.

#### 보장 수준 — "동일 시점 스냅샷"이 아니다 ⚠️

배치 조회가 보장하는 것은 **"동일 시점에 읽은 각 변수의 최신값"** 이지, **각 값이 같은
시각에 생성되었다는 뜻이 아니다.**

각 값은 서로 다른 노드가 서로 다른 주기로 발행한 것이다. `ee_pose_publisher` 는 50 Hz,
`joint_state_broadcaster` 는 컨트롤러 주기로 돌므로 두 값의 `stamp` 는 일치하지 않는다.
**클라이언트가 "이 `ee_pose` 는 이 `joint_states` 에 대응한다"고 가정하면 틀린다.**

- 각 변수의 `stamp` / `age_ms` 를 그대로 유지하므로, 정합이 필요한 클라이언트는 이를
  직접 비교해 판단한다.
- 트윈은 시점을 맞추기 위한 보간·대기·재구성을 하지 않는다.
- 엄밀한 시점 정합이 요구되면 그때 별도 수단(동기화된 파생 변수)을 검토한다. 폴링
  인터페이스로 해결할 문제가 아니다 (5.8 참조).

이 보장 수준을 인터페이스 문서에 명시해, 클라이언트가 잘못된 가정 위에 설계하는 것을
막는다.

### 5.5 JSON 직렬화 규약

언어 중립 인터페이스이므로 다음을 고정한다.

| 대상 | 규약 | 이유 |
|---|---|---|
| `builtin_interfaces/Time` | `{"sec": int, "nanosec": int}` 원본 유지 | ISO8601 변환은 정밀도 손실 |
| 트윈이 찍는 시각 | ISO8601 UTC (`received_at`) | 사람이 읽기 위함 |
| `float` `NaN` / `Inf` | `null` | JSON 에 해당 리터럴이 없다 |
| `int64` / `uint64` | 문자열 | JS `Number` 정밀도 손실 |
| 정수 enum 상수 | **문자열 심볼** (`"SUCCEEDED"`), 필요 시 원본 숫자 병기 | 클라이언트의 매직넘버 하드코딩 방지 |
| 병렬 배열 (`name[]`+`position[]`) | `{name: value}` map | 인덱스 순서 의존은 컨트롤러 설정 변경 시 **조용히** 틀린다 |
| byte array (이미지 등) | **상태 변수로 노출 금지** | JSON base64 는 대역폭·GC 부담이 크다 |
| 좌표계 | `frame_id` 를 envelope 로 끌어올려 항상 포함 | 통합 시점 사고 방지 |
| 단위 | 변수 메타에 선언 (m, rad, quaternion xyzw) | 〃 |

#### 단위 변환은 하지 않는다

rad → deg, m → mm 같은 변환을 트윈이 제공하지 않는다. **ROS 원본 단위를 그대로 노출**하고
`units` 메타로 무엇인지 명시만 한다.

변환을 시작하면 어떤 단위가 기준인지 모호해지고, 클라이언트별 요구가 늘어나며(어떤 쪽은
deg, 어떤 쪽은 rad), 변환 지점이 늘수록 오차와 버그가 끼어든다. 변환은 클라이언트의
표현 계층에서 처리한다.

**"변환하지 않음"을 인터페이스 문서에 명시**해, 클라이언트가 단위를 확인하지 않고
사용하는 것을 막는다.

### 5.6 캐시·조건부 요청

- **`ETag`**: 스냅샷 세대 번호(`gen`)를 그대로 사용한다. 값이 바뀌지 않은 폴링은
  `304 Not Modified` 로 끝나 대역폭이 크게 절감된다.
- **`Cache-Control: no-cache, must-revalidate`**: 중간 프록시가 로봇 상태를 캐시하면
  재앙이다. `no-store` 가 아니라 `no-cache` 여야 ETag 재검증이 동작한다.
- **`HEAD` 지원**: 값 없이 ETag / age 만 확인하려는 클라이언트용. 구현 비용이 거의 없다.

### 5.7 초기 상태 변수

| 변수 | 소스 타입 | 유의사항 |
|---|---|---|
| `joint_states` | `topic` — `/joint_states` (`sensor_msgs/JointState`) | 재생 경로에서 `name` 이 비어 오는 케이스가 존재한다. `name` 이 비면 `ERROR` 로 처리하고 인덱스 순서에 의존하지 않는다 |
| `ee_pose` | `topic` — `/ee_pose` (`geometry_msgs/PoseStamped`) | **ROS 표준 상태가 아니라 TF 파생값**이다. rdfp 는 `ee_pose_publisher` 가 토픽으로 발행하지만, 일반 로봇에는 없으므로 `source.type: tf` 대안을 지원해야 한다 |
| `gripper_state` | `topic` — `/gripper_states` (`rdfp_msgs/GripperState`) | 연속 상태. `staleness_ms: 500`. 아래 참조 |
| `named_targets` | `static` — `get_all_named_targets()` | SRDF 정적 데이터. 아래 참조 |

**그리퍼 변수는 하나다 (2026-09-02).**

예전에는 둘로 나뉘어 있었다 — 연속 상태는 `/joint_states` 의 finger joint 에서 파생한
`gripper_position`, 명령 결과는 이벤트성 `gripper_last_command_result`
(`rdfp_msgs/GripperActionState`) 였다. `GripperNode` 도입으로 둘 다 사라졌다.

| 없앤 것 | 왜 |
|---|---|
| `gripper_position` | `/joint_states` 의 손가락은 **백엔드마다 믿을 수 없다** — 펑션베이는 TF 성립용 고정값을 주입해 "항상 열려 있다"고 거짓말한다. `GripperState.width` 가 개구 폭(m)을 직접 싣는다 |
| `gripper_last_command_result` | `GripperState.at_goal` 이 **판정을 값 안에 담는다.** 이벤트 채널을 따로 둘 이유가 없어졌다 |

**완료 판정이 세대에서 `at_goal` 로 바뀌었다.** 전에는 결과 변수의 세대가 올라가는 것을
완료로 삼았는데, `/gripper_states` 는 **주기 발행**이라 그 규칙이 성립하지 않는다 —
명령과 무관하게 다음 틱에 세대가 올라 그리퍼가 움직이기도 전에 성공을 돌려준다.
지금은 `goal` 이 이번 명령과 같고 `at_goal` 이 참인 스냅샷을 기다린다.

판정식(`open`/`close` 는 목표 자세 도달 AND NOT `stalled`, `grasp` 는 `stalled`)은
`GripperNode` 가 이미 적용했으므로 트윈은 다시 따지지 않는다. 클라이언트도 마찬가지로
`at_goal` 하나만 보면 된다 — 상세는
[GripperNode_Design.md](../gripper/GripperNode_Design.md) §2.3.

> ⚠️ **mock 에서 `grasp` 는 완료되지 않는다.** planning scene 물체에 물리가 없어
> `stalled` 를 관측할 수단이 없고, 따라서 `at_goal` 이 서지 않아 `sync_timeout_sec`
> 까지 기다린 뒤 타임아웃한다. 정직한 판정의 대가이며 처리 방침은 미결이다.

**`named_targets` 는 연산이 아니라 상태 변수다.**

SRDF 의 named target 목록은 인자 없이 현재 값을 돌려주는 성격이고 런타임에 변하지
않으므로, 연산(`POST`)이 아니라 상태 변수(`GET`)로 모델링한다. 이로써 ETag·`304`·배치
조회·`HEAD` 가 그대로 적용되고, 부작용 없는 조회에 `POST` 를 쓰는 어색함이 사라진다.

- **값 형태**: `get_all_named_targets()` 가 `dict[str, list[str]]` 을 반환하므로
  그룹별 목록을 한 번에 노출한다. `group` 인자가 필요 없어진다.

  ```jsonc
  { "panda_arm": ["ready", "extended"], "hand": ["open", "close"] }
  ```

- **갱신**: `source.type: static` 의 lazy 조회 + 캐시 정책을 따른다 (4.3 참조).
- **`move_group` 미기동 시**: `SOURCE_UNAVAILABLE` + `reason: SOURCE_NODE_DOWN`.
- **판정 기준** — 조회성 기능을 연산과 변수 중 어디에 둘지는 다음으로 나눈다.

  > 인자 없이 "현재 값"을 돌려주면 **상태 변수**, 인자를 받아 계산·수행하면 **연산**.

  같은 기준으로 `planning_groups`(`get_planning_groups()`)도 상태 변수가 된다.

### 5.8 조회 방식의 한계 (명시)

**폴링으로는 궤적을 복원할 수 없다.** 1 Hz 폴링은 50 Hz 토픽의 50 샘플 중 1개만
본다. 모니터링에는 충분하지만 "이동 중 관절 궤적을 받고 싶다"는 요구는 이 구조로
충족할 수 없다. 이 한계를 인터페이스 문서에 명시해, 이 위에 잘못된 가정으로
설계하는 것을 막는다.

---

## 6. 연산 (operation)

> extern_op 연계 쟁점(구 부록 B)은 **모두 결정되어 본문에 반영되었다.** 확장·미채택
> 요소의 전체 목록은 부록 B 말미의 요약표를 참조한다.

### 6.1 extern_op 매핑 개요

| extern_op 개념 | 로봇 트윈에서의 대응 |
|---|---|
| op-endpoint | `POST /api/v1/robot_twins/{twin}/operations/{op}` |
| session_endpoint | `/api/v1/robot_twins/{twin}/operations/sessions/{sid}` |
| 동기 연산 | 즉시 완료되는 연산 (예: gripper open/close) |
| 비동기 연산 | 이동 연산 전반 (수 초 ~ 수십 초) |
| `RUNNING → COMPLETED/FAILED/CANCELLED` | 액션 goal 상태에 대응 |

#### 모든 비동기 연산은 다중 세션 연산이다

extern_op 의 **단일 세션 비동기 연산**은 배제 범위가 op-endpoint 자기 자신인데, 본
설계의 배제 범위는 **자원(arm/gripper)** 이라 더 넓다 (6.3). 단일 세션 모델을 쓰면
다음 모순이 생긴다.

```
1. move_to_joints 실행 중 (arm 점유)
2. GET  /operations/move_to_pose   → "IDLE"   (자기 자신은 안 돌고 있다)
3. POST /operations/move_to_pose   → 409      (arm 이 점유 중)
```

→ **모든 비동기 연산을 다중 세션으로 선언**한다. 실행 요청마다 세션이 생성되고
`session_endpoint` 가 **항상 발급**된다.

- extern_op 4.2.1 에 따라 **`GET <op-endpoint>` 는 개별 실행 조회에 사용하지 않는다.**
  따라서 **`IDLE` 상태는 본 설계에 등장하지 않으며**, 위 모순이 원천 제거된다.
- 이는 형식상의 회피가 아니라 실질과도 맞다 — `arm` 세션과 `gripper` 세션이 동시에
  살아있을 수 있고, 종료된 세션이 결과 조회를 위해 일정 기간 공존한다.
- 자원 배타성은 세션 모델이 아니라 **세션 생성 시점의 admission control** 로 보장한다 (6.3).

| extern_op 상태 | 본 설계에서의 사용 |
|---|---|
| `IDLE` | **사용하지 않는다** (다중 세션이므로 op-endpoint 상태 조회가 없다) |
| `RUNNING` / `COMPLETED` / `FAILED` / `CANCELLED` | 세션 단위로 사용한다 |

### 6.2 동기/비동기는 연산 종류별로 고정한다

extern_op 은 동기/비동기 여부를 "서버 구현 정책"으로 두고, 클라이언트가 응답의 HTTP
상태 코드(`200` / `202`)로 런타임에 판별하도록 규정한다. 본 설계는 여기서 한 걸음 더
나아가 **연산마다 동기/비동기를 정적으로 고정**한다.

- `move_to_pose` 는 항상 비동기다. 소요 시간이 짧게 끝나더라도 `202 + RUNNING` 을 반환한다.
- `move_gripper_to_target` 은 항상 동기다. `200 + COMPLETED` 로 끝난다.

트윈 정의의 `kind: sync | async` 가 이를 선언하며, `GET .../operations` 카탈로그로
노출한다. extern_op 과의 호환은 유지된다 — 클라이언트는 여전히 HTTP 코드로 판별할 수
있고, 고정 선언은 그 위에 얹히는 **추가 보장**이다.

얻는 것:

- 클라이언트가 호출 전에 폴링 루프 필요 여부를 안다. 런타임 분기가 사라진다.
- OpenAPI 스펙에 연산별 응답 스키마를 정확히 기술할 수 있다.
- 동일 연산이 상황에 따라 다른 형태로 응답하는 비결정성이 없어진다.

**대신 다음을 확정해야 한다.**

#### 동기 연산의 계약

| 항목 | 규칙 |
|---|---|
| **시간 상한** | 서버 측 상한(권장 2초)을 두고, 초과 시 `200 + FAILED + TIMEOUT` 으로 끝낸다. 무한정 매달리게 두지 않는다 |
| **취소 불가** | 세션이 생성되지 않으므로 `session_endpoint` 가 없고 취소할 수 없다. 따라서 동기 연산은 반드시 짧아야 한다 |
| **자원 락** | 자원을 점유하지 않는 동기 연산은 `resource: null` 로 선언한다. 팔이 이동 중에도 실행 가능해야 하는 연산이 여기 해당한다 |
| **구현** | HTTP 핸들러 스레드에서 rclpy 동기 API 를 호출하지 않는다. async API 가 반환한 `Future` 를 상한 시간만큼 `await` 하고, executor 스레드가 완료시킨다 (2.3 참조) |

**동기 연산이라고 즉시 끝나는 것이 아니다.** 백엔드가 준비되지 않은 상태는 시작 전
검사에서 걸러 **`503` + `PRECONDITION_FAILED`** 로 거부한다 (6.8). 검사를 통과한 뒤
상한을 넘긴 경우만 `200` + `FAILED` + `TIMEOUT` 이다.

#### 신규 연산의 동기/비동기 판정 기준

고정 정책이므로 연산 추가 시점에 판단이 필요하다. 다음을 기준으로 한다.

> **p99 소요 시간이 1초 미만이고, 중간에 취소할 이유가 없으면 동기. 그 외는 비동기.**

로봇을 **물리적으로 움직이는 연산은 소요 시간과 무관하게 비동기**로 둔다. 취소 가능성이
안전 요구사항이기 때문이다. `move_gripper_to_target` 처럼 짧은 동작도 이 기준을 적용하면 비동기가
되지만, 그리퍼는 취소 실익이 없고 실패해도 위험이 낮아 예외로 동기 처리한다 (6.14 참조).

### 6.3 자원 락 — 세션 생성 시점의 admission control ⚠️

`move_to_pose` 와 `move_to_joints` 는 서로 다른 op-endpoint 지만 **동시에 실행되면 안
된다.** 배제 단위를 **자원(resource)** 으로 잡고, 연산 정의의 `resource` 필드가 점유
대상을 지정한다.

| 자원 | 해당 연산 |
|---|---|
| `arm` | `move_to_pose`, `move_to_joints`, `move_to_named_target` |
| `gripper` | `move_gripper_to_target`, `move_gripper` |
| `null` | 자원을 점유하지 않는 연산 (팔이 이동 중에도 실행 가능) |

`arm` 과 `gripper` 는 독립 자원이므로 병렬 실행을 허용한다.

**락은 세션 생성 시점에만 판정한다.** 6.1 에 따라 모든 비동기 연산이 다중 세션이므로,
자원 배타성은 세션 모델이 아니라 **admission control** 이 담당한다.

- 통과하면 세션을 생성하고 `202 + session_endpoint` 를 반환한다.
- 통과하지 못하면 **세션을 만들지 않고** `409` 로 거부한다. 이 거부가 사실상 중복 실행
  방지 역할을 하므로 별도 멱등성 키를 두지 않는다 (6.11).
- 자원당 **`RUNNING` 세션은 최대 1개**다. 종료된 세션은 보존 기간 동안 여럿 공존한다.

#### 자원 점유 상태를 별도 자원으로 노출한다

`IDLE` 을 쓰지 않으므로(6.1) 클라이언트가 "지금 시작할 수 있는가"를 알 수단이 필요하다.

```jsonc
GET /api/v1/robot_twins/panda01/resources
{
  "arm":     { "state": "BUSY", "operation": "move_to_joints",
               "session": "/api/v1/robot_twins/panda01/operations/sessions/6f1f2a09",
               "estimated_remaining_ms": 11800 },
  "gripper": { "state": "FREE" }
}
```

조회와 시작 사이의 경합은 남으므로, **클라이언트는 `409` 처리를 생략할 수 없다.**
이 엔드포인트는 사전 확인 편의를 위한 것이지 배타성 보장 수단이 아니다.

#### 경합 시 큐잉하지 않고 거부한다

같은 자원이 점유 중인 상태의 새 요청은 **대기시키지 않고 `409 Conflict` 로 거부**한다.

**큐잉을 채택하지 않는 이유**

- **extern_op 상태 모델에 "대기"가 없다.** 상태 5종에 자리가 없어 `status: RUNNING` +
  `phase: QUEUED` 로 우회하게 되는데, 이는 `CANCELING`(6.6) 보다 훨씬 무리한 확장이다.
  `CANCELING` 은 로봇이 실제로 감속 중이지만 `QUEUED` 는 아무 일도 일어나지 않는 상태를
  `RUNNING` 이라 부르는 것이다. `progress` 가 무의미해지고, `max_duration_sec` 워치독을
  큐 진입 시점부터 재는지 실행 시작부터 재는지도 새로 정해야 한다.
- **지연 작동(delayed actuation)은 로봇에서 위험하다.** 오래 전에 제출한 연산 때문에
  아무도 보고 있지 않을 때 로봇이 움직인다. "누가 다음에 언제 움직이는가"는 명시적이어야
  한다.
- **부수 설계가 딸려온다.** 대기 중 취소는 `RUNNING` 을 거치지 않은 `CANCELLED` 가 되어
  extern_op 상태 전이도에 없는 경로가 생긴다. 앞 연산이 실패했을 때 다음을 실행할지,
  큐 길이 상한·대기 타임아웃·우선순위도 모두 정의해야 한다.
- **처리량 문제는 큐가 아닌 다른 수단으로 푼다.** 큐잉의 실질적 이득은 "클라이언트가
  `COMPLETED` 를 늦게 인지하는 만큼 로봇이 노는" 시간을 없애는 것인데, 이는 **10.1 의
  조건부 롱폴링**으로 해결된다. 프로토콜을 건드리지 않고 같은 이득을 얻는다.

#### `409` 응답 — 점유자와 예상 잔여 시간을 함께 준다

계획 완료 시 궤적 총 duration 을 알 수 있으므로 잔여 시간을 계산할 수 있다.
`Retry-After` 헤더를 함께 주어 표준 클라이언트가 자동 처리하게 한다.

```jsonc
409 Conflict
Retry-After: 12
{
  "error": {
    "code": "RESOURCE_BUSY",
    "message": "resource 'arm' is occupied by move_to_joints",
    "resource": "arm",
    "operation": "move_to_joints",
    "occupied_by": "/api/v1/robot_twins/panda01/operations/sessions/6f1f2a09",
    "estimated_remaining_ms": 11800
  }
}
```

**재시도 폭주 방지** — 여러 클라이언트가 동시에 `409` 를 받고 같은 시점에 재시도하면
thundering herd 가 된다. `Retry-After` 를 존중하되 **클라이언트 측에서 jitter 를 더할
것**을 인터페이스 문서에 권고로 명시한다.

#### 순차 실행이 필요해지면 — 복합 연산으로 확장한다

일반 큐를 도입하지 않고, 여러 단계를 **하나의 세션으로 묶는 복합 연산**(예:
`pick_and_place`)을 추가한다. 중간 상태·실패 처리·취소가 그 연산의 내부 책임이 되므로
프로토콜이 그대로 유지되고, `progress` 와 `phase` 도 자연스럽게 의미를 갖는다.

### 6.4 취소와 비상정지

**취소(`DELETE <session_endpoint>`)의 의미를 명시한다.**

- 취소는 액션 goal 의 `cancel_goal_async()` 로 전달되며, **감속 정지**를 의미한다.
- 취소 후 로봇은 **경로 중간의 불확정 자세**에 있다. 이를 인터페이스 문서에 명시하고,
  `outputs` 에 취소 시점의 pose 를 담아 반환한다 (6.7 의 `final_pose` 와 동일 형식).

#### 취소는 즉시 확정되지 않는다 — `202` + `phase: CANCELING`

로봇은 감속 정지에 시간이 걸리므로 **취소 요청 시점에 최종 상태를 확정할 수 없다.**
extern_op 4.3.2 는 이 경우 `202` 확장을 허용하면서 "구체적인 확장 규약은 별도로
정의되어야 한다"고 위임했다. 본 설계는 다음과 같이 정의한다.

`CANCELING` 을 **6번째 상태로 만들지 않는다.** extern_op 의 상태 5종 집합을 유지하고,
이미 도입한 `phase`(6.6)로 표현한다.

```jsonc
DELETE /operations/sessions/6f1f2a09
→ 202 { "status": "RUNNING", "phase": "CANCELING" }      // 취소 접수, 감속 중

GET /operations/sessions/6f1f2a09
→ 200 { "status": "CANCELLED",
        "outputs": { "stopped_at": { "position": {...}, "frame_id": "panda_link0" },
                     "goal_error": { "position_m": 0.083, "orientation_rad": 0.12 } } }
```

**`DELETE` 후에도 세션을 유지한다.** extern_op 4.3.1 은 다중 세션의 `DELETE` 를
"실행 취소 및 session 컨텍스트의 제거"로 규정하지만, 그대로 따르면 클라이언트가 최종
`CANCELLED` 와 정지 지점을 확인할 수 없다. → **보존 기간(6.12) 동안 유지**하며, 이는
extern_op 에 대한 명시적 확장이다.

이미 종료 상태인 세션에 `DELETE` 하면 `409` 다 (extern_op 4.3.3).

**비상정지는 취소와 다른 채널이다.**

`DELETE <session_endpoint>` 는 세션 id 를 알아야 하고, 세션이 만료되었거나 클라이언트가
id 를 잃으면 사용할 수 없다. 세션과 무관한 별도 엔드포인트를 둔다.

```
POST /api/v1/robot_twins/panda01/estop
```

- 진행 중인 모든 goal 을 취소하고, 이후 연산을 거부하는 잠금 상태로 전이한다.
- 진행 중이던 세션은 **`200` + `FAILED` + `ESTOP_ENGAGED`** 로 종료한다.
- 잠금 상태에서 들어온 새 연산 요청은 **`409` + `ESTOP_ENGAGED`** 로 거부한다. 세션을
  만들지 않는다 (6.8).
- 해제는 명시적 요청(`DELETE .../estop`)으로만 가능하다.
- **누구든 호출할 수 있다.** 권한 개념을 두지 않으므로(8.4) 세션 소유 여부와 무관하며,
  권한을 도입하더라도 이 채널은 열어 둔다. 안전이 접근 제어보다 우선한다.
- 호출은 감사 로그에 예외 없이 남긴다 (8.2).

세션 취소(`DELETE <session_endpoint>`) 역시 현재는 소유권을 검사하지 않는다. 세션
식별자를 아는 주체면 취소할 수 있다.

#### `CANCELLED` 는 클라이언트 요청 전용이다 ⚠️

extern_op 3.1 은 `CANCELLED` 를 **"클라이언트의 요청에 의해"** 취소된 상태로 한정한다.
따라서 안전 장치나 트윈 내부 판단으로 중단된 경우는 `CANCELLED` 가 아니라 `FAILED` 다.

| 종료 사유 | `status` | `error.code` |
|---|---|---|
| 클라이언트 `DELETE` | `CANCELLED` | — |
| **E-stop** | `FAILED` | `ESTOP_ENGAGED` |
| **`max_duration_sec` 초과 워치독** (6.5) | `FAILED` | `TIMEOUT` |
| **다른 세션에 의한 선점** | `FAILED` | `PREEMPTED` |

이 구분이 없으면 클라이언트가 "내가 취소한 것"과 "안전 장치가 개입한 것"을 나눌 수
없다. 앞의 셋은 모두 `outputs` 에 정지 지점을 담아 반환한다.

### 6.5 데드맨 — 클라이언트 유실 대비

polling 방식이므로 트윈은 클라이언트의 생존을 알 수 없다. 클라이언트가 죽어도
로봇은 계속 움직인다. 다음 중 하나를 적용한다.

- **`max_duration_sec` 을 모든 이동 연산의 필수 인자로** 두고, 트윈 워치독이 초과 시
  강제 취소한다. (기본 채택)
- heartbeat 기반 유지 (`teleop_retarget` 의 `pedal_timeout` 과 같은 발상). 폴링
  자체를 heartbeat 로 간주하는 확장도 가능하다.

워치독에 의한 중단은 클라이언트 요청이 아니므로 **`FAILED` + `TIMEOUT`** 이다
(`CANCELLED` 가 아니다 — 6.4 참조). 중단 과정은 취소와 동일하게 `phase: CANCELING` 을
거치며, 정지 지점을 `outputs` 에 담는다.

### 6.6 진행률 — `progress` 와 `phase`

MoveIt 액션 feedback 은 진행률을 제공하지 않는다. 다음과 같이 산출한다.

- 계획 완료 시 궤적 총 duration(`points[-1].time_from_start`)을 알 수 있으므로,
  **경과 시간 / 총 시간** 으로 계산한다.
- **`phase` 필드를 추가한다.** 20초 연산이 "계획 3초 + 실행 17초"일 때 구분이 없으면
  클라이언트는 3초간 멈춘 것으로 인식한다. extern_op 에 없는 확장 필드다.

```jsonc
{ "status": "RUNNING", "phase": "EXECUTING", "progress": 42,
  "message": "executing trajectory (7.1s / 17.0s)" }
```

`phase` 값: `PLANNING` | `EXECUTING` | `CANCELING`.

`CANCELING` 은 취소 요청 접수 후 감속 정지가 끝나기 전의 국면이다 (6.4). 클라이언트
취소뿐 아니라 워치독·E-stop 에 의한 중단도 이 국면을 거친다. **`status` 는 그동안
`RUNNING` 을 유지**하며, 정지가 확정되면 `CANCELLED` 또는 `FAILED` 로 넘어간다.

### 6.7 연산 결과(`outputs`) — 도달 보장 수준

**JGPC 스택을 지원 범위에 포함한다** (`moveit.move_group_mode: jgpc` 유효). 대신
`COMPLETED` 가 보장하는 수준이 스택에 따라 다르다는 사실을 결과에 드러낸다.

**배경** — `MoveGroupJgpcClient` 의 실행 경로는 **open loop** 다. 명령 스트리밍이 정상
종료해도 목표 도달을 보장하지 않는다. JTC 라 해도 액션 `SUCCEEDED` 는 "goal tolerance
내 도달"이지 정확한 도달이 아니다. 즉 **어느 스택이든 `COMPLETED` 만으로는 얼마나
정확히 도달했는지 알 수 없다.**

#### 모든 이동 연산은 도달 검증 결과를 `outputs` 에 담는다

```jsonc
{
  "status": "COMPLETED",
  "outputs": {
    "final_pose":   { "position": {...}, "orientation": {...}, "frame_id": "panda_link0" },
    "final_joints": { "panda_joint1": 0.0012, "...": 0 },
    "goal_error":   { "position_m": 0.0012, "orientation_rad": 0.004, "joint_max_rad": 0.002 },
    "closed_loop":  true,
    "measured_age_ms": 21,
    "planned_duration_ms": 17000,
    "actual_duration_ms": 17240
  }
}
```

| 필드 | 의미 |
|---|---|
| `goal_error` | 목표 대비 잔차. 클라이언트가 자신의 기준으로 성패를 판단한다 |
| `closed_loop` | `jtc` 면 `true`, `jgpc` 면 `false` |
| `measured_age_ms` | 측정에 사용한 상태 스냅샷의 나이 (아래 참조) |

#### 트윈은 오차를 근거로 `COMPLETED` 를 뒤집지 않는다

오차가 크더라도 `FAILED` 로 바꾸지 않고 그대로 `COMPLETED` + `goal_error` 로 보고한다.

- **허용 오차 기준은 응용마다 다르다.** 트윈이 임의 임계값을 정하면 어떤 클라이언트에게는
  과하고 어떤 클라이언트에게는 부족하다.
- **판정 주체가 둘이 되면 불일치가 생긴다.** 컨트롤러가 이미 goal tolerance 로 판정한
  결과를 트윈이 2차 판정하면, 두 판정이 어긋날 때 어느 쪽이 진실인지 모호해진다.
- 오차를 그대로 주면 클라이언트가 자신의 기준으로 판단할 수 있다.

#### `closed_loop: false` 의 의미를 클라이언트에게 알린다

JGPC 스택에서는 **정상 종료가 도달을 뜻하지 않으므로 `goal_error` 확인이 필수**다.
이를 다음 두 곳에 명시한다.

- `GET /robot_twins/{twin}` 트윈 메타 — `move_group_mode` 와 `closed_loop` 특성을
  미리 노출해, 클라이언트가 연산 호출 전에 알 수 있게 한다.
- 인터페이스 문서의 각 이동 연산 설명

#### 측정 시점

연산 종료 직후 **상태 변수 캐시의 최신 스냅샷**(`joint_states`, `ee_pose`)을 사용한다.
별도 조회를 하지 않으므로 추가 지연이 없다. 다만 스냅샷은 최신값일 뿐 종료 시각과
정확히 일치하지 않으므로, `measured_age_ms` 를 함께 실어 측정의 신선도를 드러낸다.

취소된 연산에도 같은 형식을 적용한다 — 6.4 의 `stopped_at` 은 `final_pose` 와 동일한
구조이며, `goal_error` 는 "취소 시점에 목표까지 얼마나 남았는지"를 뜻한다.

### 6.8 오류 코드 체계

`error.code` 는 "시스템 간 해석 가능한 안정적 코드"여야 하므로 미리 확정한다.

#### 세션이 생성된 뒤의 실패 — `200` + `status: FAILED`

extern_op 4.4.1 의 "연산 수준 실패"에 해당한다. 요청이 접수되어 **실행이 시작된 뒤**
발생한 것들이다.

| code | 발생 조건 |
|---|---|
| `PLANNING_FAILED` | IK 실패 / 충돌 / 도달 불가 (`MoveItErrorCodes` 매핑) |
| `EXECUTION_ABORTED` | 컨트롤러 goal tolerance 위반, 액션 ABORTED, **실행 도중 컨트롤러 소실** |
| `TIMEOUT` | `max_duration_sec` 초과로 워치독이 중단 (6.5) |
| `ESTOP_ENGAGED` | 실행 중 비상정지로 중단 (6.4) |
| `PREEMPTED` | 다른 세션에 의한 선점 |
| `ROS_UNAVAILABLE` | 실행 중 ROS 그래프 연결 상실 |

#### 세션이 생성되지 않은 거부 — 4xx / 5xx

**실행이 시작조차 되지 않았으므로 `status` 가 없다.** extern_op 4.4.1 의 "프로토콜 수준
오류"에 해당하며, 세션을 만들지 않으므로 `session_endpoint` 도 발급되지 않는다.

| code | HTTP | 발생 조건 |
|---|---|---|
| `RESOURCE_BUSY` | `409` | 대상 자원이 점유 중 (6.3) |
| `ESTOP_ENGAGED` | `409` | 비상정지 잠금 상태에서의 연산 요청 |
| `PRECONDITION_FAILED` | `503` | 백엔드가 준비되지 않음 (아래 참조) |
| (검증 실패) | `400` | `inputs_schema` 위반, 로봇 도메인 제약 위반 (6.9) |

`ESTOP_ENGAGED` 는 두 표에 모두 등장한다 — **실행 중 중단이면 `200 + FAILED`,
잠금 상태에서의 신규 요청이면 `409`** 다.

#### `PRECONDITION_FAILED` 의 검사 항목 — 로컬 조회만 한다

연산 시작 전 검사는 **비용이 0 인 로컬 조회로 한정**한다. 그래야 매 요청마다 검사할 수
있고, 캐시가 없으니 판정이 stale 해질 여지도 없다.

| 검사 | 방법 | 적용 |
|---|---|---|
| 트윈의 MoveGroup 클라이언트 준비 여부 | 내부 플래그 (2.6 의 4단계 완료 여부) | **`backend.method` 를 선언한 연산에만** |
| ~~해당 연산에 필요한 액션 서버 ready 여부~~ | ~~`ActionClient.server_is_ready()`~~ | **미채택 — 아래 참조** |

**검사는 연산이 실제로 쓰는 백엔드만 본다.** 설정의 백엔드 선언으로 갈린다 —
`backend.method` 는 `MoveGroupClient` 의 메서드, `backend.topic` 은 명령 토픽 발행이다.
그리퍼 연산은 MoveGroup 을 거치지 않으므로 팔 스택이 없어도 쓸 수 있어야 한다. 코드에
연산 이름 목록을 두면 연산을 늘릴 때마다 두 곳을 고쳐야 하고, 한쪽을 잊으면 멀쩡한
연산이 `503` 으로 거부된다.

**액션 서버 ready 검사는 채택하지 않았다.** 확인하려면 이 자리에서 액션 클라이언트를
만들어야 하는데, 갓 만든 클라이언트는 아직 매칭 전이라 `server_is_ready()` 가 `False`
를 돌려주어 **멀쩡한 서버를 미준비로 오판한다.** 클라이언트를 상시 보유하면 해결되지만,
그러려면 연산 카탈로그의 모든 액션을 기동 시 열어 두어야 한다. 그 확인은 실행 단계의
`wait_for_server` 가 맡고, 없으면 `EXECUTION_ABORTED` 로 보고한다.

**`/controller_manager/list_controllers` 는 호출하지 않는다.** 서비스 왕복 지연이
생기고, 캐시하면 판정이 stale 해진다. 컨트롤러 상태는 **`GET /health` 가 주기 갱신으로만
제공**하며 연산 경로에서는 조회하지 않는다.

`PRECONDITION_FAILED` 는 **시작 전 판정 전용**이다. 실행 도중 백엔드가 사라지는 경우는
`EXECUTION_ABORTED` 로 처리한다.

#### `CANCELLED` 에는 `error` 를 붙이지 않는다

클라이언트 자신의 요청에 의한 정상 종료이므로 오류가 아니다. 따라서
`CANCELLED_BY_USER` 같은 코드는 두지 않는다 — `status: CANCELLED` 자체가 그 정보다.

`MoveItErrorCodes`(`GOAL_IN_COLLISION`, `INVALID_MOTION_PLAN` 등)는 별도 매핑 테이블을
두고 `error.details` 에 원본 코드를 보존한다.

### 6.9 입력 검증은 로봇에 닿기 전에

joint limit, workspace 범위, quaternion 정규화, `frame_id` 유효성을 **트윈에서 먼저
검증**한다. 검증 없이 MoveIt 까지 보내면 수 초 뒤에 불친절한 메시지로 실패한다.

프로토콜상으로도 이는 **연산 실패(`200 + FAILED`)가 아니라 프로토콜 오류
(`400 Bad Request`)** 다. 연산 정의의 `inputs_schema`(JSON Schema)로 1차 검증하고,
로봇 도메인 제약은 트윈이 추가 검증한다.

### 6.10 공통 인자

모든 이동 연산에 다음을 공통 옵션으로 둔다. 트윈이 안전 상한으로 clamp 한다.

| 인자 | 기본값 | 설명 |
|---|---|---|
| `velocity_scaling` | 0.1 | 속도 스케일링 (0.01 ~ 1.0) |
| `acceleration_scaling` | 0.1 | 가속 스케일링 (0.01 ~ 1.0) |
| `max_duration_sec` | 60 | 워치독 강제 취소 시한 |
| `frame_id` | `panda_link0` | pose 인자의 기준 좌표계 |

계획 방식에 따른 인자는 공통이 아니라 **해당 연산 고유**다 (6.14).

| 연산 | 고유 인자 |
|---|---|
| `move_to_pose` (자유 계획) | `planner_id`, `planning_time` |
| `move_linear` (Cartesian) | `max_step`, `jump_threshold` |

### 6.11 멱등성 — `Idempotency-Key` 를 지원하지 않는다

**`Idempotency-Key` 를 지원하지 않는다.** 중복 실행 위험이 이미 두 겹으로 막혀 있어
실익이 작기 때문이다.

**1. 자원 락이 실행 중 재시도를 막는다.**

```
POST move_to_pose → 세션 생성, arm 점유, 실행 시작
→ (응답 유실)
→ 클라이언트 재시도 → 409 RESOURCE_BUSY   (중복 실행 없음)
```

중복 실행 방지는 실질적으로 키가 아니라 **admission control**(6.3) 이 수행한다.

**2. 모든 연산이 절대 목표를 지정하므로 본질적으로 멱등적이다.**

| 연산 | 목표 지정 |
|---|---|
| `move_to_pose` | 절대 pose |
| `move_to_joints` | 절대 joint 값 |
| `move_to_named_target` | SRDF 이름 (절대) |
| `move_gripper` | 절대 위치 |
| `move_gripper_to_target` | 이름이 가리키는 절대 폭 + 힘 (`open` / `close` / `grasp`) |

같은 목표로 두 번 실행해도 두 번째는 이미 그 자세에 있어 즉시 끝난다. 자원 락이 막지
못하는 경우(연산 종료 후의 재시도)에도 피해가 없다.

#### 설계 원칙 — 연산은 절대 목표만 받는다 ⚠️

**상대 이동(`move_relative` 류)을 연산으로 추가하지 않는다.** 이것이 위 멱등성의 근거이며,
이 원칙이 깨지면 `Idempotency-Key` 가 즉시 필요해진다. 새 연산을 설계할 때 반드시 확인한다.

#### 카탈로그에 `idempotent` 를 노출한다

```jsonc
GET /operations
{ "move_to_pose": { "kind": "async", "resource": "arm", "idempotent": true, ... } }
```

클라이언트는 이 값으로 재시도 안전성을 판단한다. 비멱등 연산이 추가되면 이 필드가
`false` 가 되어 **키 도입 시점이 자연스럽게 드러난다.**

#### 응답을 놓친 클라이언트의 세션 복구

키가 없으므로 응답 유실 시 `session_endpoint` 를 잃는다. 대신 다음으로 복구한다.

```
GET /operations/sessions?status=RUNNING     진행 중 세션 조회
409 응답의 occupied_by                       재시도 시 점유자 세션
```

클라이언트가 하나인 현 단계에서는 이것으로 충분하다.

#### 재도입 트리거

- **상대 이동 연산**이 추가될 때
- **복합 연산**(`pick_and_place` 등)이 추가될 때 — 부분 실행 후 재시도하면 상태가 달라진다
- 클라이언트가 둘 이상이 되어 `409` 의 `occupied_by` 가 자기 세션인지 판별할 수 없을 때

### 6.12 세션 수명주기

extern_op 4.5 는 종료 세션을 "최소 수 분 이상" 유지하라고만 권장하고 구체값을 서버
구현에 위임했다. 상한이 없으면 세션이 무한 누적되므로 다음과 같이 정한다.

| 항목 | 값 | 근거 |
|---|---|---|
| 종료 세션 보존 | **15분** | 클라이언트 재접속 여유 확보. 폴링 주기(수 초)보다 충분히 길어야 한다 |
| 세션 수 상한 | **트윈당 200개, LRU 축출** | 메모리 무한 증가 방지 |
| 만료·축출된 세션 조회 | `404` | 두 경우를 구분하지 않는다 (6.13) |

**보존 기간이 폴링 주기보다 훨씬 길어야 하는 이유** — 이 여유가 있어야 "활성 폴링 중의
`404` 는 재시작"이라는 추론이 성립한다 (6.13). 15분을 줄이면 그 추론이 약해진다.

종료 상태(`COMPLETED` / `FAILED` / `CANCELLED`)에 도달한 세션만 축출 대상이다.
`RUNNING` 세션은 개수 상한과 무관하게 유지한다 — 자원당 최대 1개이므로 누적되지 않는다.

#### 세션 목록 조회

```
GET /operations/{op}/sessions              특정 연산의 세션
GET /operations/sessions?status=RUNNING    트윈 전체 (운영·디버깅용)
```

extern_op 에는 세션 목록 조회 수단이 없다. 다중 세션 모델(6.1)에서 `IDLE` 을 쓰지 않는
이상 "지금 무엇이 돌고 있는가"를 알 방법이 필요하므로 확장한다.

### 6.13 재시작 시 고아 goal ⚠️

트윈이 죽으면 세션 컨텍스트는 사라지지만 **MoveIt 쪽 액션 goal 은 계속 실행 중일 수
있다.** 로봇이 움직이는데 취소권을 가진 주체가 없는 상태가 된다.

→ **기동 시 진행 중인 goal 을 전부 취소**하는 것을 기본 정책으로 한다. 세션 상태를
프로세스 밖에 영속화하는 것은 이 규모에서 과도하다.

#### 재시작 감지 기능은 두지 않는다

재시작 여부를 클라이언트에 알리는 별도 수단(기동 식별자 헤더, `404` 사유 구분)을
구현하지 않는다.

- **안전은 위 goal 취소가 담당한다.** "로봇이 움직이는데 취소권자가 없다"는 상황은
  발생하지 않으므로, 재시작 감지가 막을 것은 위험이 아니라 클라이언트의 오해다.
- **복구 행동이 사유와 무관하게 동일하다.** `404` 를 받은 클라이언트가 할 일은 상태
  변수로 로봇의 실제 상태를 확인하는 것 하나뿐이며, 사유는 로그 메시지만 바꾼다.
- **사실상 추론 가능하다.** 세션 보존 기간이 폴링 주기보다 훨씬 길므로(6.12),
  **활성 폴링 중의 `404` 는 만료로 설명되지 않는다** — 거의 확실히 재시작이다.

#### 클라이언트 지침 (인터페이스 문서에 명시한다)

> **활성 폴링 중 `404` 를 받으면 연산의 최종 결과를 알 수 없는 것으로 간주하고, 상태
> 변수로 로봇의 실제 상태를 확인한다.** 완료로 가정하지 않는다.

기동 식별자(`X-Twin-Boot-Id`)는 프로토콜 기능이 아니라 운영 진단 수단이므로, 관측성을
정할 때 함께 판단한다 (부록 A-12).

### 6.14 초기 연산 목록

`kind` 는 6.2 에 따라 연산별로 고정된 값이며, 트윈 정의 YAML 에 선언되고
`GET .../operations` 카탈로그로 노출된다.

| 연산 | 자원 | `kind` | `idempotent` | rdfp 백엔드 |
|---|---|---|:-:|---|
| `move_to_pose` | arm | **async** | ✔ | **미구현** — 자유 계획 (9장 참조) |
| `move_linear` | arm | **async** | ✔ | `follow_trajectory_async` (Cartesian) |
| `move_to_joints` | arm | **async** | ✔ | `move_to_joints_async` |
| `move_to_named_target` | arm | **async** | ✔ | `move_to_named_target_async` |
| `move_gripper` | gripper | **async** | ✔ | `control_msgs/GripperCommand` 액션 |
| `move_gripper_to_target` | gripper | **sync** | ✔ | `backend.labels` 가 보낼 수 있는 **심볼 목록**이고, `backend.topic`(`rdfp_msgs/GripperCommand`) 으로 발행된다. 숫자는 `GripperNode` 의 `targets` 파라미터가 갖는다 |

**현재 모든 연산이 `idempotent: true`** 다 — 전부 절대 목표를 지정하기 때문이다 (6.11).
새 연산 추가 시 이 값을 반드시 판정한다.

#### `move_to_pose` 와 `move_linear` 를 분리한다

pose 로의 이동은 **계획 방식이 두 가지**이고, 이를 하나의 연산에 `planner` 인자로 합치지
않고 별도 연산으로 나눈다.

| | `move_to_pose` (자유 계획) | `move_linear` (Cartesian) |
|---|---|---|
| 경로 결정 | OMPL 등 플래너가 탐색한다 | waypoint 사이를 직선 보간한다 |
| EE 궤적 | 곡선. 샘플링 기반이라 매번 다를 수 있다 | 예측 가능한 직선 |
| 장애물 | **회피한다** | 회피하지 못한다 — 막히면 거기서 멈춘다 |
| 실패 방식 | 계획 실패 (`PLANNING_FAILED`) | **부분 성공** (`fraction < 1.0`) |
| MoveIt 호출 | `MoveGroup` 액션 | `GetCartesianPath` 서비스 |
| 용도 | "저 위치로 가라" — 일반적인 이동 | "직선으로 내려가서 집어라" |

**분리하는 이유**

- **`outputs` 스키마가 다르다.** Cartesian 은 `fraction`(부분 성공)이라는 고유 개념이
  있어, 같은 연산에 담으면 "이 필드가 언제 있는가"가 애매해진다.
- **인자가 다르다.** Cartesian 은 `max_step` / `jump_threshold`, 자유 계획은
  `planner_id` / `planning_time` 을 받는다.
- **클라이언트가 의도를 명시하게 된다.** "직선이어야 한다"는 대개 안전 요구사항이므로,
  인자 기본값에 묻히면 안 된다.

`move_linear` 의 `outputs` 에는 `fraction` 을 포함한다. `fraction < 1.0` 을 성공으로 볼지
실패로 볼지는 **클라이언트가 판단**한다 — 트윈은 값을 보고할 뿐 뒤집지 않는다 (6.7 의
`goal_error` 와 같은 원칙).

**계획 파이프라인은 이미 준비되어 있다** — `panda_mock` 계열 launch 가 OMPL / PILZ /
CHOMP 세 개를 로드한다 ([launch_helper.py:56](../../src/robot_control/robot_control/launch_helpers/common.py#L56)).
`planner_id` 미지정 시 기본 파이프라인을 사용한다.

**`move_gripper_to_target` 이 동기인 근거** — 6.2 의 기준(물리적으로 움직이는
연산은 비동기)에 대한 명시적 예외다. 그리퍼 동작은 짧게 끝나고, 중간 취소의 실익이
없으며, 실패해도 위험이 낮다.

이 연산은 명령 토픽에 발행하고 **`at_goal` 이 설 때까지 기다린 뒤** 응답한다.
하드웨어 접점을 직접 부르지 않는 이유는 액션 goal 전송이 서비스라 **rosbag2 가 기록하지
못하기** 때문이다 — 명령이 토픽으로 흘러야 학습 데이터의 action 채널이 남는다.
따라서 `sync_timeout_sec` 은 goal 왕복이 아니라 **실제 동작 시간**을 덮어야 하며,
그만큼 HTTP 요청이 블록된다 (기본 5초). 예산을 넘기면 `TIMEOUT` 인데, 이때 명령은
이미 나간 뒤이므로 **"결과를 모른다"는 뜻**이지 "동작하지 않았다"가 아니다.

**세대 변화를 완료로 삼지 않는다** — `/gripper_states` 는 주기 발행이라 명령과 무관하게
세대가 오른다. `goal` 이 이번 명령과 같고 `at_goal` 이 참인 스냅샷만 결과로 인정한다.

파지 여부는 `at_goal` 이 이미 답한다 — `grasp` 의 판정식이 `stalled` 이기 때문이다
(물체에 막혀 멈추는 것이 곧 성공이고, 목표 자세까지 닫히면 오히려 헛닫힘이다).
⚠️ **mock 은 `stalled` 관측 수단이 없어 `grasp` 가 타임아웃한다** (5.7 참조).

**취소·E-stop 은 진행 중인 그리퍼 goal 을 멈추지 않는다.** 동기라 취소 창 자체가
없고, `_stop_backend()` 는 `MoveGroupClient` 만 안다. 위 "실패해도 위험이 낮다"는
전제에 기대는 부분이므로, 파지력을 크게 주는 목표를 설정에 추가할 때 다시 검토한다.

**named target 목록 조회는 연산이 아니라 상태 변수다** (`named_targets`, 5.7 참조).
인자 없이 현재 값을 돌려주는 조회성 기능은 변수 쪽으로 보낸다.

---

## 7. REST API 요약

```
# 디스커버리
GET    /api/v1/robot_twins                                  트윈 목록 (항상 1개 — 2.4 참조)
GET    /api/v1/robot_twins/{twin}                           트윈 메타 + 카탈로그
GET    /api/v1/robot_twins/{twin}/health                    ROS 연결/노드/컨트롤러 상태

# 상태 변수
GET    /api/v1/robot_twins/{twin}/variables                 변수 목록 + 스키마/단위
GET    /api/v1/robot_twins/{twin}/variables?names=a,b       배치 조회 (보장 수준은 5.4)
GET    /api/v1/robot_twins/{twin}/variables/{name}          단일 조회
HEAD   /api/v1/robot_twins/{twin}/variables/{name}          ETag/age 만 확인

# 자원 점유
GET    /api/v1/robot_twins/{twin}/resources                 자원별 점유 상태 (사전 확인용)

# 연산
GET    /api/v1/robot_twins/{twin}/operations                연산 목록 + kind/resource/inputs_schema
POST   /api/v1/robot_twins/{twin}/operations/{op}           실행 (op-endpoint)
GET    /api/v1/robot_twins/{twin}/operations/{op}/sessions  해당 연산의 세션 목록
GET    /api/v1/robot_twins/{twin}/operations/sessions       트윈 전체 세션 (?status=RUNNING)
GET    /api/v1/robot_twins/{twin}/operations/sessions/{sid} 상태 조회 (session_endpoint)
DELETE /api/v1/robot_twins/{twin}/operations/sessions/{sid} 취소

# 안전
POST   /api/v1/robot_twins/{twin}/estop                     비상정지
DELETE /api/v1/robot_twins/{twin}/estop                     비상정지 해제
```

**변수 이름과 트윈 하위 리소스의 경로 공간을 분리**하기 위해 `variables/` 세그먼트를
둔다. 이것이 없으면 `health` 라는 이름의 변수가 `/health` 엔드포인트와 충돌한다.

**`GET <op-endpoint>` 는 제공하지 않는다.** 모든 비동기 연산이 다중 세션이므로
(6.1), 개별 실행 상태는 `session_endpoint` 로만 조회한다 (extern_op 4.2.1).

extern_op 에 대한 확장 사항:

| 확장 | 이유 |
|---|---|
| `GET .../resources` | `IDLE` 을 쓰지 않으므로 "지금 시작 가능한가"를 알 수단이 필요하다 (6.3) |
| `GET .../operations/{op}/sessions`, `.../operations/sessions` | 진행 중 세션 목록 조회 수단이 프로토콜에 없다 |
| `202` 응답에 `Location` 헤더 | body 의 `session_endpoint` 와 중복이나, 표준 HTTP 클라이언트가 자동 추적 가능 |
| `Retry-After` / `poll_interval_ms` | 20초 연산을 100 ms 로 폴링하는 낭비를 막는다 |
| `phase`, `message` 필드 | `progress` 숫자만으로는 상태 설명이 안 된다 |

---

## 8. 성능 · 운영

### 8.1 직렬화 메모이즈

폴링 클라이언트가 N명이면 같은 스냅샷을 N번 JSON 변환하게 된다. **스냅샷 세대
번호(`gen`)를 키로 직렬화 결과를 캐시**하면 변환은 세대당 1회로 끝난다. ETag 값과
동일한 키를 쓰므로 추가 비용이 사실상 없다.

### 8.2 로그 볼륨

폴링은 요청 수가 많다. 상태 조회 GET 을 매 건 액세스 로그로 남기면 로그가 폭발한다.

- 상태 조회는 샘플링하거나 로그 레벨을 낮춘다.
- **감사 로그는 연산 실행에만** 남긴다. 누가·언제·어떤 인자로 실행했는지는 로봇 사고
  조사에 필수이며, 나중에 추가하기 어렵다.
- 인증을 도입하지 않았으므로(8.4) "누가"는 **요청 IP 와 `User-Agent`** 로 기록한다.
  연산 시작·취소·E-stop 은 예외 없이 남긴다.

### 8.3 응답 크기 상한

현재 정의된 변수들은 작지만, `/tf` 나 PointCloud 를 변수로 등록하면 응답이 폭발한다.
정책적 상한(권장 256 KB)과 초과 시 명확한 오류 코드(`RESPONSE_TOO_LARGE`)를 정의한다.

### 8.4 인증 · 인가 — 현 단계에서는 도입하지 않는다

**권한 개념을 두지 않는다.** 세션 소유권 판정도 하지 않으므로, 누구든 실행 중인 세션을
취소할 수 있고 E-stop 을 호출할 수 있다.

**근거**

- 현재 클라이언트는 MDT Platform 하나다. 소유권 판정은 클라이언트가 둘 이상일 때만
  의미가 생긴다.
- 권한을 넣으려면 인증 체계(토큰 발급·검증·주체 식별)가 선행되어야 한다. 복잡도의
  실체는 세션이 아니라 이쪽이다.
- **취소 권한은 안전 측면에서 오히려 열어두는 편이 낫다.** 아무나 멈출 수 있는 것이
  안전 정합적이다. 통제해야 할 대상은 "아무나 **시작**하는 것"이며, 이는 취소 권한이
  아니라 **네트워크 노출 범위**(부록 A-14)로 다룬다.

#### 나중에 도입할 때 깨지지 않도록 지금 해둘 것

| 항목 | 현재 |
|---|---|
| 세션 레코드의 `created_by` | **필드 자리를 두고 `null` 로 채운다.** 나중에 주체를 채워 넣을 때 스키마 변경이 없다 |
| E-stop 을 세션과 분리된 엔드포인트로 | 이미 그렇게 설계했다 (6.4). 권한 도입 후에도 이 채널은 열어 둔다 |
| 읽기/쓰기 경로 분리 | 이미 분리되어 있다 (`GET /variables` vs `POST /operations`). 스코프를 걸 지점이 명확하다 |
| 감사 로그의 요청 출처 | 인증이 없어도 **IP 와 `User-Agent` 는 기록한다** (8.2). 나중에 토큰 주체로 대체한다 |

`session_endpoint` 는 인증이 없는 만큼 **추측이 어려운 식별자**를 사용한다
(extern_op 4.5). 이것이 현재 유일한 접근 통제 수단이다.

#### 도입 트리거

- 클라이언트가 둘 이상이 될 때
- 트윈이 신뢰 경계 밖(사내망 너머)에 노출될 때

#### 네트워크 노출 범위와 TLS

| 항목 | 결정 |
|---|---|
| 바인딩 주소 | **`0.0.0.0`** — 모든 인터페이스에서 listen 한다 |
| TLS | **사용하지 않는다** (평문 HTTP) |
| 포트 | 트윈마다 분리 (2.4). `http.port` 로 지정 |

**결과적으로 애플리케이션 수준의 접근 통제가 없다.** 인증도 권한도 TLS 도 없으므로,
트윈에 라우팅이 닿는 주체는 누구든 다음이 가능하다.

- 상태 변수 조회
- 연산 실행 — **로봇을 물리적으로 움직인다**
- 실행 중 세션 취소, E-stop 호출

```bash
# 이 한 줄로 로봇이 움직인다
curl -X POST http://<host>:8801/api/v1/robot_twins/panda01/operations/move_to_pose \
     -H 'Content-Type: application/json' -d '{"inputs": {...}}'
```

또한 평문이므로 같은 망의 주체가 요청·응답을 열람하고 변조할 수 있다.

**따라서 통제는 망 수준에서 이루어져야 한다** — 방화벽 규칙, 네트워크 분리(로봇 전용
VLAN 등), 물리적 접근 통제. 트윈은 **신뢰된 폐쇄망 안에 있다는 전제**로 운용한다.

이 전제가 성립하지 않는 환경(사내망 너머 노출, 게스트 네트워크 공유, 클라이언트 다수)
으로 가면 위 "도입 트리거"가 발동한 것이며, 인증·TLS·바인딩 제한을 함께 도입해야 한다
(10.4).

### 8.5 설정 리로드 — 재시작으로만 반영한다

트윈 정의 YAML 이 바뀌면 **프로세스를 재시작**한다. 무중단 리로드를 지원하지 않는다.

무중단 리로드는 변수 구독 재구성(추가·삭제·QoS 변경), 진행 중 세션 유지, 연산 카탈로그
교체가 얽혀 복잡도가 이득을 크게 웃돈다. 설정 변경은 드물게 일어나는 일이다.

**운영 절차** — 재시작은 세션 소실(6.13)과 진행 중 goal 취소를 유발하므로, 다음을 지킨다.

1. `GET /operations/sessions?status=RUNNING` 으로 실행 중 세션이 없음을 확인한다.
2. 재시작한다.
3. `GET /health` 로 준비 완료를 확인한다.

실행 중 세션이 있는 상태에서 재시작하면 로봇이 경로 중간에 정지하고, 클라이언트는
`404` 를 받는다 (6.13 의 클라이언트 지침 참조).

### 8.6 기타

- **Rate limiting**: 폴링 폭주가 곧 트윈 부하다.
- **CORS**: 브라우저 클라이언트를 허용할 경우 명시적으로 설정한다.
- **NTP 동기**: `stamp` 신뢰의 전제 (4.2 참조).

---

## 9. rdfp 구현 격차

현재 코드 기준으로 새로 만들어야 하는 부분이다.

| 항목 | 현황 |
|---|---|
| `move_to_named_target` | `move_to_named_target_async()` — **사용 가능** |
| `named_targets` 변수 | `get_all_named_targets()` (그룹별 `dict[str, list[str]]`) — **사용 가능** |
| `move_gripper_to_target` | `rdfp_msgs/GripperCommand` 토픽 발행 → `GripperNode` 가 자기 백엔드 방식으로 실행 — **사용 가능**. 하드웨어 직접 호출은 rosbag2 가 기록하지 못해 채택하지 않았다 |
| `move_gripper` (임의 폭) | **의도적 미개방.** 명령은 심볼만 싣는다 — 숫자는 그리퍼에 종속이라 다른 기구로 옮기면 틀린 값이 된다. 쓸 수 있는 심볼은 설정(`backend.labels`)이 정한다 |
| `move_linear` (Cartesian) | `follow_trajectory_async()` — **사용 가능**. `fraction` 을 `outputs` 로 노출하도록 반환값 확인 필요 |
| **`move_to_pose`** (자유 계획) | **공개 API 없음.** `_build_move_group_goal()` 이 `JointConstraint` 만 만들므로([move_group_client.py:953](../../src/robot_control/robot_control/moveit/move_group_client.py#L953)), pose 목표는 `PositionConstraint` + `OrientationConstraint` 를 추가하거나 IK(`GetPositionIK`)로 joint 값을 구해 기존 경로에 태워야 한다 |
| ~~`move_to_joints`~~ | **해소됨.** `MoveGroupClient` 에 `plan_joints()` / `plan_joints_async()` 를 두고, 실행은 구현별로 `move_to_joints()` / `move_to_joints_async()` (JTC=MoveGroup 액션, JGPC=command 스트리밍). 관절 이름이 planning group 에 속하는지는 검사하지 않으므로 그룹 밖 관절은 계획 단계에서 실패한다 |
| ~~`gripper_position` 연속 상태~~ | **해소됨.** `/joint_states` 파생을 버리고 `GripperState.width`(개구 폭 m)를 직접 받는다 (5.7) |
| 상태 스냅샷 캐시 계층 | 신규 구현 |
| 도달 검증 (`goal_error`) | 신규 구현. 상태 캐시의 `joint_states` / `ee_pose` 와 목표값을 비교한다 (6.7) |
| 진행 중 동작의 정지 | `MoveGroupClient.cancel()` — **구현 완료**. 아래 참조 |

#### 진행 중 동작의 정지 — `MoveGroupClient.cancel()` (해소됨)

구현 중 실측으로 경로를 확정했다. **취소는 컨트롤러 계층에서 끊어야 한다.**

| 계층 | CancelGoal 응답 | 로봇 |
|---|---|---|
| `MoveGroup` 액션 (`/move_action`) | **응답 없음(타임아웃)** | 궤적을 끝까지 실행 |
| 컨트롤러 `FollowJointTrajectory` | `return_code=0`, `goals_canceling=1` | **정지** |

`move_group` 이 goal 실행 중에는 `CancelGoal` 요청에 응답하지 못하는 것으로 보인다.
반면 궤적을 실제로 실행하는 주체인 컨트롤러는 정상적으로 취소를 처리한다.
**`mock_components` 특유의 현상이 아니다** — 컨트롤러 계층은 mock 에서도 취소를
정상 처리하므로, 원인은 하드웨어 백엔드가 아니라 `move_group` 계층이다.

**구현**

- `MoveGroupClient.cancel()` / `has_active_goal()` — 공개 API. 네 개의 goal 전송
  경로가 수락된 핸들을 추적하고 결과 수령 시 해제한다.
- `MoveGroupJtcClient.cancel()` — 컨트롤러의 `FollowJointTrajectory` 액션에
  `CancelGoal` 을 보낸다. `goal_id` 와 `stamp` 가 모두 0 인 요청은 ROS 2 action
  규격상 **해당 서버의 모든 goal 취소**를 뜻하므로, `moveit_simple_controller_manager`
  가 소유한 goal 도 취소된다. 한 팔을 여러 주체가 동시에 지휘하지 않는다는 전제가 필요하다.
- `MoveGroupJgpcClient.cancel()` — `stop_streaming()` + 상위 goal 취소.
- 액션 클라이언트는 `ReentrantCallbackGroup` 을 쓴다. 기본 그룹에서는 결과 대기 중
  cancel 응답이 같은 그룹에 갇힌다.

**검증** — mock 스택에서 취소·E-stop 모두 로봇이 실제로 정지함을 확인했다(정착 후
2초간 변화 0.000000). 취소된 세션은 `CANCELLED`(error 없음), E-stop 은
`FAILED` + `ESTOP_ENGAGED` 로 종료된다.

---

## 10. 향후 확장 여지

### 10.1 조건부 롱폴링

"polling 만 고려한다"는 현재 방침이나, 실무에서는 "변화를 더 빨리 알고 싶다"는
요구가 거의 반드시 발생한다. 그때 SSE/WebSocket 으로 가면 REST 모델이 깨진다.

**조건부 롱폴링으로 흡수할 수 있다.**

```
GET /api/v1/robot_twins/panda01/variables/joint_states?wait_ms=2000
If-None-Match: "1043"

→ 값이 바뀌면 즉시 200, 안 바뀌면 2초 후 304
```

**지금 구현할 필요는 없다.** 다만 5.6 의 ETag 를 지금 도입해두면 이 확장이 나중에
사실상 공짜가 되고, 클라이언트 코드도 거의 그대로 유지된다. 폴링 전용으로 시작하되
이 문을 닫지 않는 것으로 충분하다.

### 10.2 스키마 진화

변수의 `value` 스키마가 바뀌면 클라이언트가 조용히 깨진다. 경로 버저닝(`/api/v1/`)에
더해 응답의 `schema_version` 과 변수 메타의 스키마 버전으로 호환성을 판단할 수 있게 한다.

### 10.3 `static` 소스의 자동 무효화

현재 `static` 소스는 lazy 조회 후 무기한 캐시하며, 백엔드 노드의 그래프 이탈/재등장을
감지하지 않는다 (4.3). 따라서 `move_group` 이 재시작되어 SRDF 가 바뀌면 **옛 값이 계속
반환된다.** 갱신하려면 `?refresh=true` 를 호출하거나 트윈을 재시작해야 한다.

**확장 시 검토할 것**

- rclpy 그래프 이벤트(`Node.get_node_names()` 폴링 또는 graph guard condition)로 노드
  이탈/재등장을 감지할 수 있는지, 감지 지연은 얼마인지
- 감지 후 캐시를 즉시 버릴지, 다음 조회 시 lazy 재조회할지
- SRDF 외에 `static` 으로 분류될 다른 데이터가 생겼을 때도 같은 정책이 맞는지

**도입 트리거** — 로봇 구성(SRDF)이 런타임에 바뀌는 운용이 생기거나, 백엔드 재시작이
잦아 수동 갱신이 부담이 될 때.

### 10.4 접근 통제 — 인증 · TLS · 바인딩 제한

현재는 애플리케이션 수준의 접근 통제가 전혀 없다 (8.4). 바인딩은 `0.0.0.0`, TLS 미사용,
인증·권한 미도입이며, **신뢰된 폐쇄망 전제**로 운용한다. 이 전제가 깨지면 아래를 함께
도입한다 — 셋은 개별이 아니라 한 묶음으로 판단해야 한다.

**1) 바인딩 제한**

`0.0.0.0` 을 내부망 인터페이스나 `127.0.0.1` 로 좁힌다. 가장 싸고 효과가 큰 조치이므로
가장 먼저 검토한다. `http.host` 설정값이므로 코드 변경이 없다.

**2) TLS**

A-13 에서 리버스 프록시를 두지 않기로 했으므로(2.4) 선택지는 둘이다.

| 방식 | 비용 |
|---|---|
| 트윈이 직접 종단 (uvicorn `--ssl-*`) | **트윈 수만큼 인증서** 관리·갱신 |
| 이 시점에 프록시를 도입해 종단 | 인증서 한 곳. 단 단일 장애점이 생긴다 (2.4 의 트레이드오프 재검토) |

트윈이 소수면 전자, 여러 개로 늘면 후자가 유리해진다.

**3) 인증 · 권한**

8.4 의 "나중에 도입할 때 깨지지 않도록 지금 해둘 것" 네 가지(세션의 `created_by`,
E-stop 채널 분리, 읽기/쓰기 경로 분리, 감사 로그 요청 출처)가 이미 준비되어 있다.
토큰 발급·회수 주체를 정하는 것이 남은 일이다.

**도입 트리거** — 클라이언트가 둘 이상이 되거나, 트윈이 신뢰 경계 밖에 노출될 때.
실기(물리 로봇) 적용 시에는 A-16 과 함께 반드시 재검토한다.

---

## 부록 A. 설계 쟁점 — extern_op 외 (잔여 2건)

> **사용법**은 부록 B 와 같다. 결정된 내용은 본문에 반영하고 해당 항목을 삭제한다.
> **ID 는 재사용·재번호하지 않는다** — 삭제된 번호는 비워 두어 과거 논의를 참조할 때
> 혼선이 없도록 한다. extern_op 프로토콜 자체와 얽힌 쟁점은 **부록 B** 에 따로 있다.

| ID | 분류 | 쟁점 | 중요도 | 상태 |
|---|---|---|:-:|:-:|
| ~~A-1~~ | 구조 | ~~트윈을 `rdfp` 안에 둘 것인가 별도 패키지로 뺄 것인가~~ | — | **결정 → 2.5** |
| ~~A-2~~ | ROS 연동 | ~~`create_move_group_client(mode='auto')` 오판과 기동 순서~~ | — | **결정 → 2.6** |
| ~~A-3~~ | ROS 연동 | ~~JGPC 스택 지원 여부와 `COMPLETED` 의 의미~~ | — | **결정 → 6.7** |
| ~~A-4~~ | 연산 정책 | ~~자원 경합 시 큐잉할 것인가 거부할 것인가~~ | — | **결정 → 6.3** |
| ~~A-5~~ | 연산 정책 | ~~동시 클라이언트의 취소·E-stop 권한~~ | — | **결정 → 8.4** |
| ~~A-6~~ | ROS 연동 | ~~`PRECONDITION_FAILED` 의 검사 항목 확정~~ | — | **결정 → 6.8** |
| ~~A-7~~ | 상태 변수 | ~~`static` 소스 캐시 무효화 방식~~ | — | **결정 → 4.3 (감지 미도입), 10.3** |
| ~~A-8~~ | 상태 변수 | ~~배치 조회의 시점 정합성 보장 수준~~ | — | **결정 → 5.4** |
| ~~A-9~~ | 상태 변수 | ~~단위 변환 정책~~ | — | **결정 → 5.5** |
| ~~A-10~~ | 연산 정책 | ~~`move_to_pose` 의 계획 방식~~ | — | **결정 → 6.14 (두 연산으로 분리)** |
| ~~A-11~~ | 운영 | ~~설정 리로드 정책~~ | — | **결정 → 8.5** |
| A-12 | 운영 | 관측성 — 메트릭 노출 | 중간 | 미결 |
| ~~A-13~~ | 운영 | ~~트윈 다중 인스턴스 배치~~ | — | **결정 → 2.4 (포트 분리)** |
| ~~A-14~~ | 보안 | ~~네트워크 노출 범위와 TLS 종단~~ | — | **결정 → 8.4, 10.4** |
| ~~A-15~~ | 범위 | ~~MDT AAS Submodel 매핑~~ | — | **결정 → 1.2 (범위 제외)** |
| A-16 | 범위 | 실기(물리 로봇) 적용 시 추가 안전 요구 | 보류 | 미결 |

---

### A-12. 관측성 — 메트릭 노출

운영 중 문제 진단을 위해 최소한 다음이 필요하다. 노출 방식(Prometheus `/metrics`
엔드포인트 등)을 결정한다.

- 변수별 수신율(Hz), `quality` 분포, 마지막 수신 경과 시간
- 연산별 실행 횟수 / 성공·실패율 / 소요 시간 분포
- 활성 세션 수, 자원 점유 시간 비율
- HTTP 요청 수·지연 (상태 조회와 연산 실행 분리 집계)
- **기동 식별자 / 기동 시각 / 재시작 횟수** — B-5 에서 넘어온 항목이다. 재시작 감지는
  프로토콜 기능으로 두지 않기로 했으므로(6.13), 필요하면 여기서 `X-Twin-Boot-Id`
  응답 헤더나 `/health` 필드로 노출할지 함께 판단한다

이것이 없으면 "가끔 STALE 이 뜬다" 같은 신고를 재현할 방법이 없다.

**결정 시 반영 위치**: 8장

---

### A-16. 실기(물리 로봇) 적용 시 추가 안전 요구 — 보류

현재 설계는 mock 스택을 전제로 한다. 물리 로봇에 적용하려면 최소한 다음이 추가로
필요하며, 본 설계의 범위를 넘는다.

- 워크스페이스 경계 제한 (소프트 리밋)
- 속도·힘 상한의 하드웨어 수준 강제
- 물리 비상정지 회로와의 연동 (`POST /estop` 은 소프트웨어 정지일 뿐이다)
- 사람 감지·협동로봇 안전 표준(ISO/TS 15066 등) 대응

**이 항목은 실기 적용이 결정될 때 별도 문서로 분리한다.**

---

## 부록 B. extern_op 연계 쟁점 (결정 완료)

> **전건 결정 완료.** 아래 표는 결정 이력이며, 각 항목의 상세는 본문으로 옮겨졌다.
> 확장·미채택 요소의 최종 목록은 이 부록 말미의 요약표에 있다.

| ID | 쟁점 | 제안 | 상태 |
|---|---|---|:-:|
| ~~B-1~~ | ~~자원 락이 extern_op 세션 모델과 어긋난다~~ | ~~모든 비동기 연산을 다중 세션으로 선언~~ | **결정 → 6.1, 6.3** |
| ~~B-2~~ | ~~취소가 즉시 확정되지 않는다~~ | ~~`202` + `phase: CANCELING`, 세션 유지~~ | **결정 → 6.4** |
| ~~B-3~~ | ~~`CANCELLED` 와 `FAILED` 의 경계~~ | ~~E-stop·워치독은 `FAILED`~~ | **결정 → 6.4, 6.8** |
| ~~B-4~~ | ~~`Idempotency-Key` 와 세션의 보존 기간 불일치~~ | ~~키 보존 ≥ 세션 보존~~ | **결정 → 6.11 (키 미지원)** |
| ~~B-5~~ | ~~재시작 후의 `404` 가 만료와 구분되지 않는다~~ | ~~`X-Twin-Boot-Id` + `reason`~~ | **결정 → 6.13 (감지 기능 미도입)** |
| ~~B-6~~ | ~~세션 보존 정책 미정~~ | ~~15분 + 200개 LRU~~ | **결정 → 6.12** |

---

### extern_op 대비 확장 목록 (확정)

부록 B 의 쟁점이 모두 결정되어, extern_op 프로토콜에 대한 확장은 다음으로 확정되었다.
이 표는 extern_op 확장 명세의 초안으로 그대로 사용할 수 있다.

| 확장 | 근거 | 본문 |
|---|---|---|
| 자원(resource) 개념과 `GET /resources` | 배제 단위가 op-endpoint 보다 넓다 | 6.3 |
| 동기/비동기를 연산별로 고정 선언 (`kind`) | 클라이언트의 런타임 분기를 없앤다 | 6.2 |
| `phase` (`PLANNING`/`EXECUTING`/`CANCELING`) | 상태 5종을 늘리지 않고 진행 국면을 표현 | 6.6 |
| `DELETE` → `202` + 세션 유지 | 4.3.2 가 명시적으로 위임한 확장 | 6.4 |
| `FAILED` 세부 코드로 E-stop·워치독 구분 | `CANCELLED` 는 클라이언트 요청 전용 | 6.4, 6.8 |
| `POST`/`DELETE /estop` (세션 독립) | 세션 id 없이도 정지 가능해야 한다 | 6.4 |
| 세션 목록 조회 (`.../sessions`) | 진행 중 세션을 알 수단이 프로토콜에 없다 | 6.12 |
| `outputs` 의 도달 검증 결과 (`goal_error`, `closed_loop`, `fraction`) | `COMPLETED` 의 보장 수준이 스택·계획 방식마다 다르다 | 6.7, 6.14 |
| `Retry-After` / `estimated_remaining_ms` | 폴링·재시도 낭비를 막는다 | 6.3 |

**채택하지 않은 extern_op 요소**

| 요소 | 사유 | 본문 |
|---|---|---|
| `IDLE` 상태 | 다중 세션 모델이라 op-endpoint 상태 조회가 없다 | 6.1 |
| `GET <op-endpoint>` (개별 실행 조회) | 위와 같다 (extern_op 4.2.1 에 부합) | 7장 |

**도입하지 않은 일반 수단**

extern_op 이 규정하지 않지만 REST API 에서 흔히 쓰이는 것 중 의도적으로 두지 않은 것이다.

| 요소 | 사유 | 본문 |
|---|---|---|
| `Idempotency-Key` | 자원 락 + 절대 목표 원칙으로 중복 실행이 이미 방지된다 | 6.11 |

---

## 참고 문서

- [extern_op 프로토콜](file:///home/kwlee/mdt/share/extern_op/extern_op_revised.md) — RESTful 기반 외부 시스템 연동 프로토콜
- [moveit/MoveGroupClient_UserGuide.md](../moveit/MoveGroupClient_UserGuide.md) — `MoveGroupClient` API 및 Threading 주의사항
- [gripper/GripperNode_Design.md](../gripper/GripperNode_Design.md) — 그리퍼 서비스/토픽 인터페이스
- [gripper/gripper_action_server_notes.md](../gripper/gripper_action_server_notes.md) — `gripper_cmd` 액션 계층과 mock 환경 제약
- [session/session_control_guide.md](../session/session_control_guide.md) — TRANSIENT_LOCAL QoS 사례

# robot_twin

**로봇 트윈** — 제어 계층을 변수(variable)와 연산(operation)의 REST API 로 노출한다.
LLM 에이전트가 로봇을 조작하는 진입점이며, 무엇을 노출할지는 코드가 아니라 **YAML
선언**으로 정한다 (`config/robot_twin_panda01.yaml`).

```bash
ros2 run robot_twin robot_twin --config <설정파일>
```

## 백엔드마다 설정 파일이 다르다

| 설정 | 백엔드 | 포트 |
|---|---|:-:|
| `config/robot_twin_panda01.yaml` | mock 스택 (`panda_mock`) | 8801 |
| `config/robot_twin_panda_isaac.yaml` | Isaac Sim (`panda_isaac`) | 8802 |

**연산·변수 목록은 거의 같다.** 상위(에이전트)가 백엔드를 모르게 하는 것이 이 계층의
목적이므로, 갈리는 것은 `moveit` 블록과 `use_sim_time` 뿐이다.

### `moveit.arm_command_*` — 명령 채널

`move_group_mode: jgpc` 는 계획만 MoveIt 이 하고 실행은 명령 스트리밍으로 바꾼다.
그 명령이 나가는 **토픽과 메시지 형식**을 백엔드가 정한다.

```yaml
moveit:
  move_group_mode: jgpc
  arm_command_topic: /isaac/arm_command      # 기본: /panda_arm_controller/commands
  arm_command_format: joint_state            # 기본: float64_multi_array
  arm_command_joint_names: [panda_joint1, ..., panda_joint7]
```

**토픽 remap 으로는 못 바꾼다 — 메시지 타입이 다르다.** 기본값으로 둔 채 Isaac 에
붙이면 팔이 **"성공했다고 보고하면서 아무것도 하지 않는다"**: 계획은 정상이고,
스트리밍은 개루프라 발행만 하면 성공이며, 아무도 안 듣는 토픽으로 나갈 뿐이다.

그래서 설정 단계에서 짝을 검사한다.

- `move_group_mode: jtc` 에 이 키를 주면 **거부**한다 — 그 모드는 보지 않으므로
  조용히 무시되는 키가 생긴다.
- `arm_command_format: joint_state` 인데 `arm_command_joint_names` 가 없으면
  **거부**한다 — 조회할 컨트롤러가 없어 첫 스트리밍에서 멈춘다.

현재 값은 `/health` 의 `arm_command` 에 그대로 드러난다.

### Isaac 설정에 `reset_scene` 이 없는 이유

scene 물체가 **USD 스테이지**에 있어 고치려면 시뮬레이터 안에서 스테이지를 써야 한다 —
ROS 쪽 노드는 원리적으로 할 수 없고, `isaac_scene_state_node` 가 `/scene/reset` 서비스를
**열지 않는** 이유다(발행 전용이다). 설정에 연산을 남겨 두면 서비스를 찾지 못해 실패할
뿐이다. scene 초기화는 Isaac 쪽에서 `setup_scene.py` 를 다시 돌린다.

### 트윈은 시작 자세 충돌에서 스스로 빠져나오지 못한다

모든 팔 연산이 MoveIt 계획을 거치는데, 시작 자세가 planning scene 과 충돌하면 계획
자체가 거부된다(`INVALID_MOTION_PLAN`, -2). 계획을 거치지 않는 복구 경로가 따로
필요하다 — `scripts/isaac/is_recover.py`.

## 세션/에피소드 연산은 선택적이다

`start_session` / `stop_session` / `start_episode` / `stop_episode` 는 수집 계층
(`rdfp`)이 제공한다. 트윈은 그 패키지를 **import 하지 않고**
`robot_twin.backends` entry point 그룹을 조회한다.

- `rdfp` 가 설치되어 있으면 → 자동 등록되어 네 연산이 동작한다
- 설치되어 있지 않으면 → 트윈은 정상 기동하고 해당 연산만 `409` 로 거절한다

제어 스택만 쓰는 환경에서는 설정의 `operations` 에서 네 연산과 `session_state`
변수를 빼는 편이 낫다 — `description` 은 주석이 아니라 에이전트가 읽는 데이터라,
부를 수 없는 연산이 카탈로그에 남으면 에이전트가 시도했다가 실패한다.

문서: [../../docs/robot_twin/robot_twin_user_guide.md](../../docs/robot_twin/robot_twin_user_guide.md) ·
[설계](../../docs/robot_twin/robot_twin_design.md) ·
[Isaac 백엔드](../../docs/simulation/isaac_backend_skeleton.md) (§2 Phase 9)

# robot_twin

**로봇 트윈** — 제어 계층을 변수(variable)와 연산(operation)의 REST API 로 노출한다.
LLM 에이전트가 로봇을 조작하는 진입점이며, 무엇을 노출할지는 코드가 아니라 **YAML
선언**으로 정한다 (`config/robot_twin_panda01.yaml`).

```bash
ros2 run robot_twin robot_twin --config <설정파일>
```

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
[설계](../../docs/robot_twin/robot_twin_design.md)

# Isaac 트윈 구동 및 동작 확인 절차서

Isaac 시뮬레이터 · ROS 제어 스택 · 로봇 트윈을 띄우고, robot-twin-client 로 실제 동작(블록을
실린더 위에 쌓기)까지 시험하는 순서다. 학습 데이터는 남기지 않는다 — 수집까지 하려면
[isaac_bringup_runbook.md](isaac_bringup_runbook.md) §2 의 터미널 4 · 5 를 따른다.

| | |
|---|---|
| 트윈 | id `panda01` · 포트 8801 · 백엔드 `panda_isaac` |
| 클라이언트 | `~/development/mdtpy/robot-twin-client` (기본 접속 대상이 이 트윈이라 인자가 필요 없다) |
| 함정 · 상세 | [isaac_bringup_runbook.md](isaac_bringup_runbook.md) §6 |

---

## 0. 선행 조건 확인 — 처음이거나 코드를 고친 뒤

```bash
cd ~/development/ros/rdfp_ws
./scripts/isaac/is_check_prereq.sh        # 실패 0 이어야 한다 — 실패마다 고치는 명령을 함께 보인다
```

빌드 관련 실패가 나오면 `colcon build --packages-select robot_control robot_twin rdfp` 를 실행한다.
**src 에서 지운 파일이 설치본에 남았다는 경고**는 재빌드로 안 치워지므로, 그 패키지의
`install/<pkg>` · `build/<pkg>` 를 먼저 지우고 빌드한다.

## 1. 띄우기 — 터미널 셋

```bash
# 터미널 1 — 시뮬레이터 (2분쯤 걸린다)
cd ~/development/ros/rdfp_ws
./scripts/run_isaac_sim.sh --gui
#   [bringup] PLAY 가 찍힌 뒤 /clock 이 실제로 도는지 확인하고 넘어간다
#   source ./.rdfprc && ros2 topic echo /clock --once   (두 번 실행해 sec 이 늘면 정상)

# 터미널 2 — ROS 제어 스택
cd ~/development/ros/rdfp_ws
source ./.rdfprc && ./scripts/run_panda_isaac.sh

# 터미널 3 — 로봇 트윈
cd ~/development/ros/rdfp_ws
source ./.rdfprc && ./scripts/run_robot_twin.sh --twin panda01
```

## 2. 트윈 확인 — 로봇은 안 움직인다

```bash
curl -s http://127.0.0.1:8801/api/v1/robot_twins/panda01/health; echo
curl -s http://127.0.0.1:8801/api/v1/robot_twins/panda01 | grep -o '"backend":"[^"]*"'
```

`ros: CONNECTED` · `move_group: READY` · `backend: panda_isaac` 면 정상이다.
`session_state: SOURCE_UNAVAILABLE` 은 수집 계층을 안 띄워서이고, `named_targets: NO_DATA` 는
처음 읽을 때 채워진다 — 둘 다 정상이다.

## 3. 씬 확인 · 처음 배치로 리셋 — 로봇은 안 움직인다

```bash
cd ~/development/mdtpy/robot-twin-client
uv run robot-twin-reset --list      # initial, one_block, two_objects
uv run robot-twin-reset             # 처음 배치(initial) — "놓인 자리" 차이가 몇 mm 이내면 정상
```

`initial` 은 지정 위치(`isaac_scene.json` 의 처음 배치)이고, `two_objects` · `one_block` 은 무작위다
([isaac_scene_reset.md](isaac_scene_reset.md) §7).

## 4. 쌓기 시험 — 로봇이 움직인다

```bash
# ① 고정 좌표로 쌓기 — 씬을 읽지 않고 미리 계산한 좌표로 연산 10개를 보낸다
uv run robot-twin-reset
uv run python examples/stack_block_on_cylinder_fixed.py              # 종료 코드 0 = 10단계 성공

# ② 씬을 실측하며 쌓기 — 끝에 쌓였는지 판정까지 한다
uv run robot-twin-reset
uv run python examples/stack_block_on_cylinder.py                    # "쌓였다 — 높이차 55.0 mm …"

# ③ 무작위 배치에서 쌓기 — 리셋 + 쌓기 + 확인을 한 번에
uv run robot-twin-stack --scene two_objects --seed 7
uv run robot-twin-stack --repeat 5                                   # 매번 다른 무작위 배치로 5회
```

- **①·② 앞에는 반드시 `robot-twin-reset` 을 부른다.** 블록이 실린더 위에 얹힌 채로 ① 을 돌리면
  빈손으로 4단계에서 멈춘다(물체를 엉뚱한 곳에 놓지는 않는다).
- ① 은 쌓였는지 확인하지 않는다 — 확인이 필요하면 ② 를 쓰거나 `scene_objects` 를 읽는다.
- 쌓기 결과의 가로 어긋남은 4~5 mm(−y 쪽)가 정상이다 — 블록을 쥔 동안 팔이 처지는 양이며 허용
  20 mm 안이다.
- 트윈으로 작업하는 동안 **`teleop_keyboard` 를 띄워 두지 않는다** — servo 가 팔을 끌고 가 트윈의
  이동과 컨트롤러를 두고 다툰다.

## 5. 정리

```bash
cd ~/development/ros/rdfp_ws
./scripts/kill_stack.sh --ros    # 시뮬레이터는 두고 ROS 스택만
./scripts/kill_stack.sh          # 시뮬레이터까지 전부
```

시뮬레이터 **프로세스**를 다시 띄웠다면 터미널 2 · 3 도 다시 띄운다 — 시계가 0 으로 되감겨
`TF_OLD_DATA` 가 쏟아진다. 타임라인 Stop/Play 는 시계가 이어지므로 그대로 둔다.

# 펑션베이 트윈 구동 및 동작 확인 절차서

펑션베이 시뮬레이터 · ROS 제어 스택 · 로봇 트윈을 띄우고, robot-twin-client 로 실제 동작(peg 를 한
고정물에서 다른 고정물로 옮겨 꽂기)까지 시험하는 순서다. 학습 데이터는 남기지 않는다 — 수집까지 하려면
`./scripts/run_collect_functionbay.sh` 를 쓴다.

| | |
|---|---|
| 트윈 | id `rdfp01` · 포트 8801 · 백엔드 `panda_functionbay` |
| 클라이언트 | `~/development/mdtpy/robot-twin-client` — `robot-twin-peg` 의 기본 접속 대상이 이 트윈이다 |
| 띄우는 순서 · 함정 상세 | [펑션베이_시뮬레이터_사용법.md](펑션베이_시뮬레이터_사용법.md) §2 · §6 |
| Isaac 판 | [../isaac/isaac_twin_bringup_summary.md](../isaac/isaac_twin_bringup_summary.md) |

**Isaac 과 다른 점 셋** — 먼저 알아 둔다.

- **씬 리셋이 없다.** 펑션베이는 `/scene/reset` 을 제공하지 않아 트윈의 `reset_scene` · `robot-twin-reset` 이
  실패한다. 처음 배치로 되돌리는 방법은 시뮬레이션 정지 → 시작(`/scene/stop` → `/scene/start`)이다.
- **탁자에 내려놓을 수 없다.** 씬에 `peg ↔ 탁자` 접촉이 없어 고정물(`peg_hole` · `peg_tray`) 밖에서 놓으면
  peg 가 탁자를 통과해 떨어지고, 복구는 시뮬레이터 재시작뿐이다. 그래서 동작 시험은 **고정물 사이를 오가는
  peg-in-hole** 이다.
- **시작하면 차트가 먼저 돈다.** 차트가 끝나기 전에 팔 · 그리퍼에 명령을 보내면 차트가 끊긴다 — 팔을
  움직이는 작업은 ⑥ 차트 종료 대기 뒤에 한다.

---

## 0. 선행 조건 확인 — 처음이거나 코드 · 빌드 · 씬을 바꾼 뒤

```bash
cd ~/development/ros/rdfp_ws
colcon build --packages-select rdfp_msgs robot_control robot_twin rdfp   # 설치본을 소스와 맞춘다
grep -n '^sim_scene:' src/robot_control/config/backends/panda_functionbay.yaml   # 띄울 시뮬레이터 씬(r2 · t1)과 같아야 한다
```

- **씬과 솔버는 짝이다** — `r2` 는 RecurDyn(`SolverTCPServer.exe`), `t1` 은 Crisp(`CrispSolver.exe`).
  어긋나면 에러 없이 팔만 안 움직인다.
- **새 빌드를 받았으면** 빌드 폴더 경로 · 우리 차트 설치 · 게인 대조가 먼저다 —
  [펑션베이_시뮬레이터_사용법.md](펑션베이_시뮬레이터_사용법.md) §0 · §2 ⑥.
- src 에서 지운 파일이 설치본에 남으면 재빌드로 안 치워진다 — 그 패키지의 `install/<pkg>` · `build/<pkg>` 를
  지우고 빌드한다.

## 1. 띄우기 — Windows 둘 · WSL 넷

**① 솔버 (Windows)** — 띄울 씬의 솔버 **하나만**.

```bat
:: r2 씬
cd /d D:\MMR-Build_260929\solver\RecurDyn
SolverTCPServer.exe
:: t1 씬
cd /d D:\MMR-Build_260929\solver\Crisp
CrispSolver.exe
```

**② 시뮬레이터 (Windows)** — 모델까지 로드하고 기다린다(저절로 시작하지 않는다).

```bat
cd /d D:\MMR-Build_260929\sim\build_0929
MetaMotionRobot.exe -Scene LAB_ETRI -Model r2_ms      :: t1 이면 -Model t1_ms
```

**③ `ros_tcp_endpoint` (WSL 터미널 1)** — 이미 떠 있으면 또 띄우지 않는다. Claude Code 의 샌드박스 안에서
띄우지 않는다(DDS 가 막혀 토픽이 안 보인다).

```bash
export ROS_DOMAIN_ID=31 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ss -ltnp | grep :10000                        # 떠 있는지 먼저 본다
source ~/development/ros/tcp_ws/install/setup.bash
ros2 daemon start
ros2 run ros_tcp_endpoint default_server_endpoint
```

**④ 시뮬레이션 시작 (WSL 터미널 2)**

```bash
cd ~/development/ros/rdfp_ws && source ./.rdfprc
ros2 topic pub --once -w 1 /scene/start std_msgs/msg/String "{data: ''}"
ros2 topic echo /output/panda_joint --once    # 값이 전부 0.0000 이면 솔버가 없다 (①)
./scripts/functionbay/fb_gains.py             # 종료 코드 1 = 우리 kp 와 씬 게인이 다르다 (④ 뒤에 돌린다)
```

**⑤ 로봇 스택 (WSL 터미널 2) · ⑥ 차트 종료 대기 (WSL 터미널 3)**

```bash
# 터미널 2 — 제어 스택. 기동 로그에 "Scene check passed" 와 "convention: solver=… scene=…" 가 찍혀야 한다
./scripts/run_panda_functionbay.sh

# 터미널 3 — 차트가 끝나면 종료 코드 0
cd ~/development/ros/rdfp_ws && source ./.rdfprc
./scripts/functionbay/fb_wait_chart.py
```

로봇 스택은 기동만으로는 `/input/*` 에 아무것도 내지 않으므로 차트 도중에 띄워도 된다 — 팔을 움직이는
작업만 ⑥ 뒤로 미룬다.

**⑦ 로봇 트윈 (WSL 터미널 4)**

```bash
cd ~/development/ros/rdfp_ws && source ./.rdfprc
./scripts/run_robot_twin.sh --twin rdfp01
```

## 2. 트윈 확인 — 로봇은 안 움직인다

```bash
curl -s http://127.0.0.1:8801/api/v1/robot_twins/rdfp01/health; echo
curl -s http://127.0.0.1:8801/api/v1/robot_twins/rdfp01 | grep -o '"backend":"[^"]*"'
```

`ros: CONNECTED` · `move_group: READY` · `backend: panda_functionbay` 면 정상이다.
`session_state: SOURCE_UNAVAILABLE` 은 수집 계층을 안 띄워서라 정상이다.

## 3. peg 위치 확인 — 로봇은 안 움직인다

```bash
cd ~/development/mdtpy/robot-twin-client
uv run robot-twin-peg --dry-run       # peg 가 어느 고정물에 있는지, 어디로 옮길지 판단만 한다
```

우리 차트(`rdfp_ready_<씬>`)로 시작했다면 peg 는 처음 자리 `peg_tray` 에, 팔은 `ready` 에, 그리퍼는 열린
상태다. **peg 가 쓰러져 있거나 고정물 밖에 있으면** 다루지 않는다 — 시뮬레이션을 정지 → 시작해 처음
배치로 돌린다(5절).

## 4. peg-in-hole 시험 — 로봇이 움직인다

```bash
uv run robot-twin-peg                 # 지금 위치에서 반대쪽 고정물로 옮겨 꽂는다
uv run robot-twin-peg                 # 다시 한 번 — 원래 고정물로 돌아온다 (왕복)
uv run robot-twin-peg peg_hole        # 목적지를 명시할 수도 있다 (peg_hole · peg_tray)
```

- 세 판정이 절차의 실체다 — 파지 오프셋 보정(peg **바닥 끝**을 겨냥), 기하 사전 확인(안 되면 **움직이기
  전에** 거부), 앉음 확인. 실측(2026-09-11) 왕복 5회 연속 이탈 0건, 삽입 오차 0.05~0.40 mm.
- **실패하면 peg 를 쥔 채로 멈춘다.** 그리퍼를 열거나 같은 호출을 재시도하지 않는다 — 놓으면 peg 를 잃고,
  원인이 그대로면 결과도 같다. 출력을 보고 원인부터 본다.
- 첫 이동이 "도달하지 못했다"로 끝나는데 그리퍼는 멀쩡하면, ROS 쪽보다 먼저 `Player.log` 의
  `JointStateMsg` 예외를 센다(빈 `velocity` 함정).
- 트윈으로 작업하는 동안 **`teleop_keyboard` 를 띄워 두지 않는다** — servo 가 같은 명령 채널을 두고 다툰다.

## 5. 처음 배치로 되돌리기 · 정리

```bash
# 시뮬레이션만 처음으로 — peg 는 peg_tray, 팔은 차트 t=0 으로 돌아가고 차트가 다시 돈다
ros2 topic pub --once -w 1 /scene/stop  std_msgs/msg/String "{data: ''}"
ros2 topic pub --once -w 1 /scene/start std_msgs/msg/String "{data: ''}"
./scripts/functionbay/fb_wait_chart.py        # 다시 차트 종료를 기다린다

# 내리기
./scripts/kill_stack.sh --ros                 # ROS 스택 (먼저 --dry-run 으로 볼 수 있다)
#   ros_tcp_endpoint 는 그 터미널에서 Ctrl-C, 시뮬레이터 · 솔버는 Windows 에서 창을 닫는다
```

Stop → Start 는 로봇 스택을 다시 띄우지 않아도 됐다(2026-10-21 실측). **시뮬레이터 프로세스를 다시 띄웠다면**
sim 시계가 0 으로 되감기므로 endpoint · 로봇 스택 · 트윈도 다시 띄운다 — 살아남은 노드가 `TF_OLD_DATA` 를 내고
`/ee_pose` 가 멈춘다.

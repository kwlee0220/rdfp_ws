# 물체를 든 채 servo 로 올리면 옆으로 샌다 — 증상·원인·해결

`teleop_keyboard` 로 물체를 집은 뒤 `q`(+z)를 누르면 팔이 위로 가지 않고 **옆으로 돈다.**
물체를 놓으면 같은 키가 깨끗하게 위로 간다. 같은 파지 상태로 트윈의 `move_linear` 을
부르면 mm 단위로 들어올린다.

> **결론부터**
>
> - **원인**: servo 에는 목표가 없다. 매 주기 *측정된* 관절값에 IK 증분을 더하므로, 부하로
>   처진 만큼이 **명령 안으로 적분된다**(래칫). 되읽는 지점이 **둘**이다 — `moveit_servo`
>   자체와, servo 가 34 ms 마다 보내는 단일점 궤적을 받을 때마다 측정 상태에서 보간을
>   시작하는 **JTC**.
> - **해결**: 둘 다 명령 기준으로 바꾼다. `servo_joint_source:=commanded`
>   (`commanded_joint_state_node`) + JTC `open_loop_control: true`
>   (`config/panda_isaac_ros2_controllers.yaml`). 2026-09-07 부터 `panda_isaac.launch.py`
>   의 **기본**이며, 옆샘이 수평/수직 0.83 → **0.01** 로 사라졌다.
> - **부작용**: 되읽기가 먹던 이동량이 돌아와 같은 입력에 servo 가 **2.7배(직선)·2.3배(회전)
>   빨라졌다.** `teleop_keyboard` 의 `linear_step`/`angular_step` 기본을 0.6 → **0.25** 로
>   되맞췄다 (`scale.linear` 은 0.4 그대로).
>
> 재는 절차와 원자료: 부록 A.

---

## 1. 증상

`teleop_keyboard` 에서 `\`(grasp)로 물체를 쥔 뒤 `q` 를 누르면, 손이 위로 오르면서
동시에 옆으로 밀린다. 눈으로는 "손목이 돌아간다"로 보인다. 물체를 놓으면 사라진다.

**servo 의 자기 진단으로는 안 잡힌다.** `/servo_node/status` 는 내내 `NO_WARNING` 이다 —
특이점도 충돌도 관절 한계도 아니다. servo 는 자기가 낸 해를 정상이라고 보고한다.

> `ros2 topic echo /servo_node/status --once` 로 확인하면 안 된다. status 는 **명령을
> 넣는 동안에만** 갱신되므로, 쉬고 있을 때 찍으면 직전 값이나 `0` 을 보게 된다.
> `teleop_keyboard` 는 status 를 상시 구독해 전이 시점에 경고한다 — 그 로그를 본다.

## 2. 실측 — 수정 전 (Isaac, 2026-09-07)

같은 자세(`panda_joint6` ≈ +1.3~1.7, 손목 특이점·관절 한계에서 멀다)에서 `panda_link0`
기준 **순수 `+z`** 만 넣었다. servo `scale.linear` 0.4, 입력 0.5, 1.5 초. 100 g 블록.

| | 수평 | 수직 | 수평/수직 | `panda_joint5` | status |
|---|---:|---:|---:|---:|---|
| **A · servo, 물체를 잡은 채** | **30.4 mm** | +36.5 mm | **0.83** | **−0.239 rad** | `NO_WARNING` |
| A2 · 같은 조건 반복 | 27.1 mm | +32.5 mm | 0.83 | −0.215 rad | `NO_WARNING` |
| **B · servo, 놓은 뒤 (같은 자세)** | 3.3 mm | +31.5 mm | **0.10** | +0.006 rad | `NO_WARNING` |
| B2 · servo, **빈손으로 닫은 채** | 3.4 mm | +32.5 mm | 0.10 | +0.006 rad | `NO_WARNING` |
| **C · `move_linear`, 잡은 채** | 3축 최대 오차 **1.8 mm** | | | | — |

(2026-09-06 의 첫 관찰은 18.4 / +22.7 / 0.81 / −0.203 과 1.1 / +19.2 / 0.06, C 2.6 mm 로,
같은 그림이다.)

읽는 법:

- **수직 이동량은 A 와 B 가 비슷하다.** "느려진 것"이 아니라 **옆 성분만 9배 늘어난 것**이다.
- `panda_joint5`(팔뚝 롤)가 A 에서 −0.24 rad 돈다. 손이 그 축에서 벗어나 있으므로 이 롤이
  곧 옆으로 스치는 움직임이 된다.
- **B2 가 손가락 상태를 배제한다.** 빈손으로 닫고 밀면 B 와 같다 — 원인은 쥔 **물체**다.
- C 는 A 와 **같은 파지 상태**다. 물체를 드는 것 자체는 문제가 아니다.

## 3. 원인 — servo 는 처짐을 되읽어 명령에 적분한다

### 3.1 명령을 두 몫으로 가르면 보인다

servo 는 매 주기 *측정된* 관절값에 IK 증분을 더해 다음 명령을 만든다. 그래서 명령의
변화는 두 몫으로 나뉜다:

    IK 몫     = 명령(k) − 그 직전 측정값      servo 가 이번 주기에 **더한** 양
    되읽기 몫 = 그 직전 측정값 − 명령(k−1)    팔이 이전 명령보다 **처진** 양을 servo 가 되읽은 양

| joint5 | IK 몫 | 되읽기 몫 | 합(명령 Δ) | 실제 Δ | 처짐 / 주기 |
|---|---:|---:|---:|---:|---:|
| A · 잡은 채 | **+0.117** | **−0.337** | −0.220 | −0.239 | **−7.7 mrad** |
| A2 | +0.095 | −0.291 | −0.196 | −0.215 | −6.5 |
| B · 놓은 뒤 | +0.053 | −0.047 | +0.006 | +0.006 | −1.0 |
| B2 · 빈손 닫음 | +0.054 | −0.048 | +0.006 | +0.006 | −1.0 |

물체를 쥐면 joint5 는 **매 주기(34 ms) 7.7 mrad 씩 명령보다 처진다.** servo 는 그 처진
값을 다음 명령의 기준으로 삼으니 44 주기 동안 −0.34 rad 이 적분되고, IK 는 오히려
**+0.12 rad 를 반대로** 밀었지만 다 못 이겼다. 빈손은 처짐이 1 mrad/주기라 IK 가 정확히
상쇄해 합이 0 이다.

**servo 의 IK 가 다른 해를 고른 것이 아니다.** 시계열에서도 **실제가 명령보다 항상 조금
앞서** 내려간다 (0.2 s: −0.022 / −0.019 … 1.4 s: −0.218 / −0.214). 명령을 따라가는
거라면 실제가 뒤처져야 한다 — 부하가 먼저 끌고 명령이 따라 내려가는 모양이다.

### 3.2 왜 `move_linear` 은 괜찮은가

`move_linear` 은 **끝점이 있는 관절 궤적**이다. 같은 처짐이 생겨도 그것은 컨트롤러의
추종 오차로 남고, 컨트롤러가 목표를 향해 메운다. servo 는 그 오차를 매 주기 **새 목표로
승인**한다. 목표의 유무가 갈림이다.

같은 성질이 servo 텔레오퍼레이션 전반에 있다 — 속도 개루프라 시간이 갈수록 어긋나고,
왕복을 "현재 + 델타"로 쌓으면 오차가 누적된다.

### 3.3 되읽는 지점은 둘이다

| 지점 | 무엇을 되읽나 | 단독으로 끊었을 때 |
|---|---|---|
| ① `moveit_servo` | 매 주기 측정 관절값(`joint_topic`, 기본 `/joint_states`)에 IK 증분을 더한다 | 0.83 → 0.50 |
| ② JTC (`panda_arm_controller`) | 새 궤적을 받을 때마다 보간 시작점을 **측정 상태**로 잡는다(`open_loop_control: false`). servo 는 34 ms 마다 단일점 궤적을 보내므로 매 주기다 | ① 과 함께 → **0.01** |

①만 끊으면 절반이 남는다 — servo 가 명령 기준으로 적분해도, JTC 가 그 명령을 측정 상태에서
다시 출발시키기 때문이다.

### 3.4 부하가 왜 이렇게 크게 처지는가

블록은 **100 g** 이다. 그 무게로 주기당 7.7 mrad, 정착 20 mrad(잡은 채 같은 관절값으로
복귀했을 때 joint5 의 오차)이 처지는 것은 Isaac 의 손목 드라이브가 무르다는 뜻이다
(`tune_drive.py` 는 joint1 의 τ 를 기준으로 맞췄다). 위에서 잡는 자세에서 100 g 의 중력
토크는 joint5 축(팔뚝 롤)에 거의 안 걸리므로, 질량보다 **파지 접촉력의 비대칭**이 원인일
수 있다 — 가르지 않았다. 실기에서는 Franka 의 페이로드 설정(중력 보상)이 처짐을 줄인다.
어느 쪽이든 §4 의 해결은 처짐의 크기와 무관하게 적분을 끊는다.

## 4. 해결 — 둘 다 명령 기준으로

| 지점 | 수정 | 구현 |
|---|---|---|
| ① servo | `servo_joint_source:=commanded` (**기본**) | `commanded_joint_state_node` 가 JTC 의 명령 위치(`/panda_arm_controller/controller_state` 의 `reference`)에 컨트롤러 밖 관절(손가락)의 측정값을 합쳐 `/joint_states_commanded` 로 내고, servo 의 `joint_topic` 이 그것을 본다 |
| ② JTC | `open_loop_control: true` (**기본**) | `config/panda_isaac_ros2_controllers.yaml` — moveit_resources 의 `ros2_controllers.yaml` 복사본에 이 한 줄을 더한 것. `panda_isaac.launch.py` 의 `controllers_file` 인자가 가리킨다 |

실측 (같은 조건, 잡은 채 +z 0.5 × 1.5 s):

| 구성 | 수평 | 수직 | 수평/수직 | joint5 |
|---|---:|---:|---:|---:|
| 수정 전 (둘 다 측정 기준) | 30.4 mm | +36.5 | **0.83** | −0.239 |
| ① 만 | 24.7 | +49.9 | 0.50 | −0.185 |
| **① + ②** | **0.9** | +74.4 | **0.01** | +0.024 |
| (참고) 빈손, ① + ② | 1.7 | +76.1 | 0.02 | +0.020 |

`move_linear` 은 수정 후에도 그대로다 — 트윈의 `pick()` 이 세 번 부르는 `move_linear` 이
모두 5 mm 안에 들어왔다. JTC open-loop 는 MoveIt 궤적의 시작점도 마지막 명령으로 잡지만,
정착 오차(0.3 mrad, 부하 시 20 mrad)는 드라이브가 즉시 메운다. `rdfp_panda_isaac` 은 이
launch 를 include 하므로 수집 스택도 같은 수정을 받는다.

### 4.1 부작용 — servo 가 빨라졌다

되읽기는 매 주기 이동량도 먹고 있었다. 같은 `scale.linear` 0.4 / `scale.rotational` 에서
(`ready`, 1.5 초):

| 입력 | 직선 (수정 전 → 후) | 회전, 베이스 z (수정 전 → 후) |
|---|---|---|
| 0.25 | 11.5 → **28 mm/s** | — |
| 0.5 | 24.5 → **63 mm/s** | 2.5 → **5.8 °/s** |
| 1.0 | 47.5 → **130 mm/s** | — → 13.1 °/s |

**2.7배(직선) · 2.3배(회전)** 이고 비례는 유지된다. `scale` 을 낮추는 대신 **teleop 의
`linear_step` / `angular_step` 기본을 0.6 → 0.25** 로 되맞췄다 — 0.25 는 직선 32 mm/s ·
회전 2.9 °/s 로 수정 전 체감(28.5 mm/s · 3.0 °/s)에 가깝다. `scale` 은 0.4 를 그대로
둔다: 비례가 유지되고, 바꾸면 servo 를 재기동해야 하며(런타임 `param set` 이 안 먹는다),
`ee_twist`·`teleop_retarget` 같은 다른 servo 소비자의 보정이 함께 흔들린다.

`panda_joint4` 가 0.4 rad 넘게 펴지면 `DECELERATE_FOR_APPROACHING_SINGULARITY`(1) 가
찍히는데, 같은 시간에 멀리 가서 그런 것이지 불안정이 아니다.

### 4.2 함정 둘 — 왜 yaml 복사본인가

- JTC 의 `open_loop_control` 은 **읽기 전용**이다. `ros2 param set` 이 거부된다.
- Humble 의 `controller_manager` 는 컨트롤러 파라미터를 **params 파일로만** 넘긴다. launch
  의 인라인 dict 는 `/**` 네임스페이스로 임시 파일에 쓰여 CM 노드에는 실리지만 컨트롤러
  노드에는 닿지 않는다 (실측: CM 은 `True`, 컨트롤러는 `False`). 그래서
  `panda_jgpc_ros2_controllers.yaml` 과 같은 선례로 파일이다.

### 4.3 원래 현상을 다시 보려면

```bash
ros2 launch robot_control panda_isaac.launch.py \
    controllers_file:=/opt/ros/humble/share/moveit_resources_panda_moveit_config/config/ros2_controllers.yaml \
    servo_joint_source:=measured
```

이 구성에서 같은 절차를 돌리면 35 mm / 0.03(빈손), 잡으면 0.83 이 다시 나온다.

### 4.4 다른 백엔드

수정은 **JTC 가 있는 스택**에만 적용된다. 펑션베이는 ros2_control 없이 `/input/panda_joint`
로 곧장 나가므로 ② 가 없고, ① 만 붙이려면 `controller_state` 를 대신할 명령 위치 소스가
필요하다 — 미착수. mock 은 물리가 없어 처짐 자체가 없다.

## 5. 그래서 어떻게 하나

| 하려는 일 | 쓸 것 |
|---|---|
| 물체를 들고 정해진 자리로 옮긴다 | 트윈 `move_linear` (좌표를 `scene_objects` 에서 읽는다) — 목표가 있어 처짐이 추종 오차로만 남는다 |
| 물체를 놓을 높이를 정확히 맞춘다 | 트윈 `move_linear` + **쥔 뒤 물린 위치 실측** |
| 물체를 든 채 손으로 조금 옮긴다 | Isaac: `teleop_keyboard` (servo) — 수정 후 옆샘 1 mm 안. **펑션베이·mock 은 여전히 servo 로 옮기지 않는다** |
| 자유 공간에서 눈대중으로 자세를 잡는다 | `teleop_keyboard` (servo) |
| 정해진 자세로 복귀한다 | `move_to_named_target` (`teleop_keyboard` 의 `/`) |

트윈으로 집고 옮기는 전 과정은 [../robot_twin/robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md)
를 본다. 외부 저장소 `mdtpy/robot-twin` 의 `robot-twin-stack` 이 그 흐름을 그대로
구현한 예다 — 거기에는 **쥔 뒤 손끝과 물체 중심의 실제 차이를 재서** 놓을 좌표를
보정하는 부분이 있다. 파지 미끄러짐이 실측에서 12 mm / 1.0 mm / −3.2 mm 로 매번
달랐으므로 상수로는 잡을 수 없다.

## 6. 함께 겪는 것 — "servo 가 명령 없이도 팔을 끌고 간다"

이 현상과 **별개**지만 같은 증상("팔이 저절로 흐른다")으로 보인다.

2026-09-06 에 "`start_servo` 만 하고 twist 를 한 번도 안 보내도 servo 가 10 초에 293 건
발행하며 분당 10° 씩 중력 처짐 방향으로 흐른다"고 기록했다. **2026-09-07 재현 시도에서는
나오지 않았다** — `start_servo` 만 한 경우, 0 twist 를 한 번 준 경우, 1 초 민 뒤 0 을 준
경우, `teleop_keyboard` 를 띄운 채(키 안 누름) 모두 유휴 10 초에 **0 건 / 0 mrad** 였다
(다른 발행자 0~1). 293 건 / 10 초 = 29.3 Hz 는 servo 의 `publish_period` 그대로라 servo 가
**활성으로 발행 중**이었다는 뜻이고, 그러려면 무언가가 `incoming_command_timeout` 안에
계속 명령을 넣고 있었어야 한다. 가장 그럴듯한 후보는 **2026-09-06 이전의 `teleop_keyboard`**
다 — 데드맨 만료 후 **매 틱 0 twist 를 보냈고**, 그것이 servo 에게는 명령이라 29 Hz 발행이
이어지며 매 주기 처진 자세를 되읽어 흘렀다(§3 과 같은 기작). 지금은 0 을 **한 번**만 보내
servo 가 유휴로 들어간다.

실무 결론은 그래도 유효하다: **`teleop_keyboard` 는 기동 시 servo 를 자동 시작하므로,
트윈으로 작업하는 동안 띄워 두면 servo 와 트윈의 MoveIt 실행이 같은 컨트롤러 토픽을
두고 싸운다.** 트윈으로 작업할 때는 teleop 을 내리거나 `stop_servo` 를 부른다.

```bash
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger
ros2 service call /servo_node/start_servo std_srvs/srv/Trigger   # 다시 쓸 때
```

> **미착수**: `teleop_keyboard` 가 유휴 상태(모션 키가 1~2초 없음)에서 servo 를 스스로
> 정지시키고 다음 모션 키에서 재개하는 방안. **`pause`/`unpause` 쌍은 쓸 수 없다** —
> `scripts/isaac/is_recover.py` 의 실측에서 `unpause_servo` 가 성공을 돌려주면서도 발행을
> 되살리지 못했고 `start_servo` 만이 다시 내보내게 했다. 진행한다면 `stop`/`start` 로
> 간다. 대가는 유휴 뒤 첫 키의 서비스 호출 지연이며, 조용히 실패하면 "키가 죽었다"로
> 보이므로 로그가 필요하다.

## 7. 이력

- **2026-09-06** — 증상과 A/B/C 를 처음 쟀다. 원인으로 "servo 에 목표가 없어 처진 상태를
  되읽어 누적한다"를 가설로 세우고, joint5 의 회전이 servo 의 IK 선택인지 물체에 끌린
  것인지를 "명령에 들어 있으면 IK 선택, 명령이 0 이면 끌림"으로 가르자고 적었다.
- **2026-09-07 오전** — 재현하고 명령값을 봤더니 −0.220 rad 가 들어 있어 "IK 선택"으로
  판정했다. **틀렸다.** 그 이분법은 "처짐이 되읽혀 명령 *안으로* 들어오는" 세 번째 경우를
  놓친 것이었고, 명령을 IK 몫과 되읽기 몫으로 가르자 09-06 의 가설이 맞았음이 드러났다
  (§3.1).
- **2026-09-07 오후** — servo 의 기준을 명령 위치로 바꿔 절반이 줄었고(0.50), JTC 의
  되읽기까지 끊어 0.01 이 됐다(§4). 같은 날 "servo 가 명령 없이도 흐른다"(§6)는 재현되지
  않았다.

## 8. 관련

- [../moveit/servo_client_programmers_guide.md](../moveit/servo_client_programmers_guide.md) — `/servo_node` 시작·정지·상태
- [../moveit/MoveGroupClient_UserGuide.md](../moveit/MoveGroupClient_UserGuide.md) — 카테시안 계획·실행
- [../robot_twin/robot_twin_user_guide.md](../robot_twin/robot_twin_user_guide.md) — `move_linear` / `move_gripper_to_target`
- [../simulation/isaac_backend_skeleton.md](../simulation/isaac_backend_skeleton.md) — Isaac 백엔드 구성과 실측
- [README.md](README.md) — teleop 문서 진입점

---

## 부록 A. 재현·측정 절차

본문의 숫자는 전부 이 절차로 나왔다. 드라이브 게인을 바꾸거나 다른 백엔드에 붙일 때
같은 절차로 다시 잰다. 도구는 [scripts/isaac/is_servo_push.py](../../scripts/isaac/is_servo_push.py)
(A/B 행 + status + joint5 분해) 와 [scripts/isaac/is_servo_idle.py](../../scripts/isaac/is_servo_idle.py)
(§6 무명령 흐름) 이며, 둘 다 값을 **판정하지 않는다** — 표를 채우는 도구다. 스택이 없으면
종료 코드 2 로 멈춘다.

소요 시간: 준비 10 분 + 측정 15 분. Isaac 이 대상이다. 펑션베이·mock 에서도 돌지만 mock 은
물리가 없어 A 가 성립하지 않는다.

### A-0. 준비

**터미널 셋:**

```bash
# T1 — 시뮬레이터
./scripts/run_isaac_sim.sh --gui

# T2 — 로봇 스택 (servo · gripper · scene 이 기본으로 켜진다)
ros2 launch robot_control panda_isaac.launch.py

# T3 — 트윈 (:8802, id panda_isaac)
ros2 run robot_twin robot_twin --config src/robot_twin/config/robot_twin_panda_isaac.yaml
```

기동이 이상하면 [../simulation/isaac_bringup_runbook.md](../simulation/isaac_bringup_runbook.md)
§4 의 확인 순서를 먼저 탄다.

**어느 스택을 재는가.** 기본 스택은 §4 의 수정이 들어 있어 A 가 재현되지 않는다(비율 0.01).
**원래 현상**을 보려면 T2 를 이렇게 띄운다:

```bash
ros2 launch robot_control panda_isaac.launch.py \
    controllers_file:=/opt/ros/humble/share/moveit_resources_panda_moveit_config/config/ros2_controllers.yaml \
    servo_joint_source:=measured
```

수정된 스택(기본)에서 재면 `is_servo_push.py` 에 `--base-topic /joint_states_commanded`
를 준다 — 분해(IK 몫/되읽기 몫)가 servo 가 실제로 되읽는 상태로 계산되게.

**`teleop_keyboard` 는 띄우지 않는다 — 절차 내내.** 기동 시 `start_servo` 를 자동으로
부르기 때문에 §6 의 "명령 0 건" 조건이 깨지고, A/B 를 재는 스크립트와 같은 twist 토픽에
자기 0 을 섞어 넣으며, 트윈의 `move_linear` 과 컨트롤러 토픽을 두고 싸운다. servo 의
시작·정지는 측정 스크립트가 직접 한다. 이미 떠 있다면 내리고
`ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger` 로 세운다.

**트윈 클라이언트.** 파지는 `mdtpy/robot-twin` 의 `pick_n_place` 로 만든다 — 물체 중심에
TCP 오프셋 10.3 cm 와 `panda_link8` 의 45° yaw 를 보정하는 일을 이미 하고 있다. 그 저장소에서
`uv run python` 으로 아래 스니펫을 돌린다.

```bash
cd ~/development/mdtpy/robot-twin
uv run python -c "from robot_twin_client import RobotTwinClient; print(RobotTwinClient(port=8802, twin_id='panda_isaac').health())"
```

### A-1. 파지 만들기 (A 의 시작 상태)

트윈으로 물체를 놓고 집는다. `place` 는 부르지 않는다 — **쥔 채로** 멈춰야 한다.

```bash
# ~/development/mdtpy/robot-twin 에서
uv run python - <<'PY'
from robot_twin_client import RobotTwinClient
from robot_twin_client.pick_n_place import pick, grasp_pose_of, above_of

twin = RobotTwinClient(port=8802, twin_id='panda_isaac')
objs = twin.run('reset_scene', {'scene': 'one_block'})['outputs']['objects']
g = grasp_pose_of(objs[0])
print('grasp at', g['position'])
assert pick(twin, above_of(g), g), 'pick 실패 — 출력을 본다'
print('쥔 채 above 에서 대기 중')
PY
```

`pick()` 은 열기 → 접근 → 하강 → `grasp` → 상승이며, 파지는 `stalled` 로 판정하므로
빈손이면 여기서 멈춘다. `/gripper_states` 로 확인한다:

```bash
ros2 topic echo /gripper_states --once     # goal: grasp, stalled: true, at_goal: true
```

### A-2. A / B / B2 / C 재기

**A · servo, 잡은 채:**

```bash
./scripts/isaac/is_servo_push.py --label A --save-start /tmp/start_A.json --csv /tmp/push_A.csv
```

스크립트가 하는 일: `/ee_pose`·`/joint_states` 스냅샷 → `start_servo` →
`TwistStamped(frame_id=panda_link0, linear.z=0.5)` 를 30 Hz 로 1.5 초 발행 → 0 을 한 번
→ `stop_servo` → 1 초 정착 → 스냅샷. 발행하는 동안 `/servo_node/status` 를 상시 구독해 본
코드를 모두 모으고, `/panda_arm_controller/joint_trajectory` 의 명령값과 기준 상태의
joint5 를 나란히 기록한다(`--csv`). `--input` 은 m/s 가 아니라 [-1, 1] 의 비율이다.
`--save-start` 가 B 를 **같은 자세**에서 재기 위한 것이다.

**B · servo, 놓은 뒤 — 같은 자세.** 물체를 놓고 A 가 시작한 관절값으로 **되돌아간 뒤**
잰다. "현재 위치에서 다시"가 아니다 — A 가 팔을 옮겨 놓았기 때문이다.

```bash
uv run python - <<'PY'
import json
from robot_twin_client import RobotTwinClient
from robot_twin_client.ops import set_gripper

twin = RobotTwinClient(port=8802, twin_id='panda_isaac')
assert set_gripper(twin, 'open')['at_goal']
r = twin.run('move_to_joints', {'joints': json.load(open('/tmp/start_A.json')),
                                'velocity_scaling': 0.2})
print(r['status'])                      # COMPLETED
PY
./scripts/isaac/is_servo_push.py --label B --csv /tmp/push_B.csv
```

놓은 블록이 손 아래에 남아 있어도 +z 로만 가므로 방해되지 않는다. **B2**(빈손으로 닫은
채)는 `set_gripper(twin, 'close')` 로 바꿔 같은 순서를 밟는다.

**C · `move_linear`, 잡은 채.** 다시 집고(A-1), **`ee_pose` 를 한 번 읽어 절대 좌표**로
명령한다. "현재 + 델타"를 반복해서 쌓으면 안 된다 — §3.2 의 누적 오차가 그 함정이다.
수직 이동량을 A 와 맞추려면 A 의 "수직" 값을 `0.020` 자리에 넣는다.

```bash
uv run python - <<'PY'
from robot_twin_client import RobotTwinClient
from robot_twin_client.pick_n_place import pick, grasp_pose_of, above_of

twin = RobotTwinClient(port=8802, twin_id='panda_isaac')
objs = twin.run('reset_scene', {'scene': 'one_block'})['outputs']['objects']
g = grasp_pose_of(objs[0])
assert pick(twin, above_of(g), g)

ee = twin.read('ee_pose')['pose']                       # PoseStamped → pose
target = {'position': {**ee['position'], 'z': ee['position']['z'] + 0.020},
          'orientation': ee['orientation']}
r = twin.run('move_linear', {'pose': target, 'velocity_scaling': 0.2})
got = r['outputs']['final_pose']['pose']['position']
err = {a: (got[a] - target['position'][a]) * 1000 for a in 'xyz'}
print(r['status'], 'error mm:', {a: round(v, 1) for a, v in err.items()},
      'max', round(max(abs(v) for v in err.values()), 1))
PY
```

**판정은 비율로 한다.** A 가 0.5 를 넘고 B 가 0.15 아래면 재현이다. 수직 이동량은 A 와 B 가
비슷해야 한다 — "느려진 것"이 아니라 "옆 성분이 붙은 것"이라는 §2 의 해석이 그 조건 위에
서 있다. 수치가 크게 다르면 먼저 의심할 것: 자세가 다르다(B 의 복귀를 건너뛰었다),
`teleop_keyboard` 가 떠 있었다, Isaac 인스턴스가 둘이다(런북 §5).

### A-3. status 와 joint5 분해 읽는 법

A/B 출력의 `servo status` 줄은 발행 중 상시 구독한 코드의 집합이다. `--once` 는 쓰지
않는다(§1). A 에서도 `0=NO_WARNING` 만 나오면 재현이다. 1·2·5·6 이 섞이면 자세가
특이점·한계에 가까운 것이라 다른 자세(`ready` 근처)에서 다시 한다.

`panda_joint5` 줄은 명령 변화를 §3.1 의 두 몫으로 가른 것이다:

```
  panda_joint5    실제 Δ -0.239 rad / 명령 Δ -0.220 = IK 몫 +0.117 + 되읽기 몫 -0.337 (44 주기, 되읽기 -7.7 mrad/주기)  [기준 /joint_states]
                  → 되읽기(래칫) — 부하로 처진 측정값을 servo 가 매 주기 되읽어 명령에 적분한다 ...
```

| 판정 | 뜻 | 손댈 자리 |
|---|---|---|
| \|되읽기\| ≥ 2·\|IK\|, 부호가 실제와 같다 | **래칫** — 처짐이 명령에 적분된다 | §4 의 두 수정. 굳이 처짐 자체를 줄이려면 추종 강성(`tune_drive.py` 손목 게인) |
| \|IK\| ≥ 2·\|되읽기\| | servo 의 IK 가 그 해를 골랐다 | servo 파라미터 |
| 그 사이 | 섞였다 | 시계열(`--csv`)을 본다 |
| \|실제\| < 30 mrad | 움직임이 작아 판별하지 않는다 | 자세를 바꿔 다시 |

시계열은 `--csv` 파일에 `t_sim, kind(cmd/actual), panda_joint5` 로 있다. 명령이 실제보다
**앞서** 내려가면 servo 가 끌고 간 것이고, 실제가 먼저 내려가고 명령이 **따라** 내려가면
되읽기다(§3.1 의 모양).

### A-4. 무명령 흐름 재기 (§6)

팔을 `ready` 에 두고(자세마다 처짐이 다르다) 세 조건을 돈다. 스크립트는 유휴 구간에 twist
토픽의 **다른 발행자 수**도 찍는다 — 0 이 아니면 "명령 0 건" 조건이 아니다.

```bash
ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger     # 혹시 켜져 있으면
curl -s -X POST http://127.0.0.1:8802/api/v1/robot_twins/panda_isaac/operations/move_to_named_target \
     -H 'Content-Type: application/json' -d '{"inputs":{"target":"ready"}}'
./scripts/isaac/is_servo_idle.py                 # start_servo 만 하고 아무것도 안 보낸다
./scripts/isaac/is_servo_idle.py --prime         # 0 twist 를 한 번 준 뒤 쉰다
./scripts/isaac/is_servo_idle.py --prime 0.3     # +z 0.3 으로 1 초 민 뒤 0 을 한 번 주고 쉰다 (키를 눌렀다 뗀 상황)
```

판정: 활성 구간에 발행이 있고(~29 Hz) 관절이 걸어가는데 추종 오차가 0 이면 §6 의 09-06
관찰이 재현된 것이다. 발행이 0 이면 지금 §6 의 결론이다. `stop_servo` 뒤는 0 건 / 0 mrad
여야 한다. `pause`/`unpause` 를 다시 재려면 `is_servo_idle.py` 의 `stop_cli` 를
`/servo_node/pause_servo` 로, 두 번째 관찰 앞에 `unpause_servo` 를 넣어 발행 건수를 본다.

끝나면 `ros2 service call /servo_node/stop_servo std_srvs/srv/Trigger` 로 세운다.

### A-5. 원자료 (2026-09-07, Isaac 6.0, `panda_isaac.launch.py`)

본문 표에 없는 것만 둔다.

**구성별 A/B 와 분해** (+z 0.5 × 1.5 s):

| 구성 | 행 | 수평 | 수직 | 수평/수직 | joint5 실제 (명령) | 분해 (기준 토픽) |
|---|---|---:|---:|---:|---:|---|
| 수정 전 | A 잡은 채 | 30.4 | +36.5 | 0.83 | −0.239 (−0.220) | IK +0.117 / 되읽기 −0.337 (`/joint_states`) |
| 수정 전 | A2 반복 | 27.1 | +32.5 | 0.83 | −0.215 (−0.196) | IK +0.095 / 되읽기 −0.291 |
| 수정 전 | B 놓은 뒤 | 3.3 | +31.5 | 0.10 | +0.006 | IK +0.053 / 되읽기 −0.047 |
| 수정 전 | B2 빈손 닫음 | 3.4 | +32.5 | 0.10 | +0.006 | IK +0.054 / 되읽기 −0.048 |
| ① servo 만 commanded | A 잡은 채 | 24.7 | +49.9 | 0.50 | −0.185 (−0.187) | (측정 기준 분해라 무의미) |
| ① + ② | B0 빈손 ready | 0.2 | +94.6 | 0.00 | — | — |
| ① + ② | A 잡은 채 | 0.9 | +74.4 | 0.01 | +0.024 (+0.020) | IK +0.069 / 되읽기 −0.049 (`/joint_states_commanded`) |
| ① + ② | B 놓은 뒤 | 1.7 | +76.1 | 0.02 | +0.020 (+0.019) | IK +0.066 / 되읽기 −0.047 |

A 의 시작 관절값으로 복귀했을 때의 잔차(A2 − A): joint5 −20.5 mrad, joint7 −4.2, 나머지
1 mrad 안 — 부하가 정착 자세를 바꾼다(§3.4).

**속도 보정** (`ready`, scale 0.4, 1.5 s, 이동량 → 속도):

| 입력 | 직선, 수정 전 | 직선, ① + ② | 회전(베이스 z), 수정 전 | 회전, ① + ② |
|---|---|---|---|---|
| 0.25 | 17 mm (11.5 mm/s) | 41.5 mm (28 mm/s) | — | — |
| 0.5 | 35.4~38.9 mm (24.5) | 94.6 mm (63) | 3.8° (2.5 °/s) | 8.7° (5.8 °/s) |
| 1.0 | 71 mm (47.5) | 195.5 mm (130) | — | 19.7° (13.1 °/s) |

**무명령 흐름** (유휴 10 초):

| 조건 | 발행 | 관절 변화 | 추종 오차 | 다른 발행자 |
|---|---:|---:|---:|---:|
| 무명령 | 0 건 | 0.0 mrad | 0.32 mrad | 0 |
| 0 twist 한 번 뒤 | 0 건 | 0.0 mrad | 0.30 mrad | 0 |
| +z 0.3 × 1 s 민 뒤 0 한 번 | 0 건 | 0.9 mrad (정착 꼬리, 0.09 mrad/s) | 0.55 mrad | 0 |
| `teleop_keyboard` 를 띄운 채 (키 안 누름) | 0 건 | 0.0 mrad | 0.21 mrad | 1 |

`stop_servo` 뒤도 전부 0 건 / 0 mrad.

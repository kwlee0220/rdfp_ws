# teleop 문서 안내

리더-팔로워 텔레오퍼레이션 관련 문서 7개의 진입점이다. **하려는 일부터 고른다.**

| 하려는 일 | 문서 |
|---|---|
| 지금 로봇을 움직인다 (기동 순서·명령) | [omy_leader_teleop_guide.md](omy_leader_teleop_guide.md) |
| 매핑이 이상하다 / 스케일·정렬·필터를 튜닝한다 | [teleop_retarget_node_guide.md](teleop_retarget_node_guide.md) |
| 새 입력 장치(게임패드·시뮬레이터·다른 리더)를 붙인다 | [external_input_adapters.md](external_input_adapters.md) |
| 왜 이렇게 설계했는지 알고 싶다 | [leader_follower_mirroring_design.md](leader_follower_mirroring_design.md) |
| USB 풋페달로 클러치를 조작한다 | [clutch_pedal_guide.md](clutch_pedal_guide.md) |
| 게임패드를 `/joy` 까지 연결한다 | [joystick_setup.md](joystick_setup.md) |
| **리더 트리거로 그리퍼를 조작하고 싶다** | [leader_gripper_mapping_design.md](leader_gripper_mapping_design.md) (설계안) |

## 전체 그림

```
[입력 어댑터]                  [rdfp]
                                                            ← external_input_adapters
OMY-L100 ─(omy_leader_bridge)─▶ /leader/ee_pose
키보드 / 게임패드 ─────────────▶ /servo_node/delta_twist_cmds (경로 A, 직결)

  /leader/ee_pose ─▶ teleop_retarget ─▶ /follower/target_pose
                          ▲                     │              ← teleop_retarget_node_guide
                     /ee_pose (팔로워)          ▼
                                        ee_twist_publisher
                                                │
                                                ▼
                                   /servo_node/delta_twist_cmds
                                                │
                                                ▼
                                          servo_node ─▶ arm
```

`teleop_mirror.launch.py` 는 가운데 두 노드(`teleop_retarget`, `ee_twist_publisher`)
만 기동한다. 리더 pose 공급자와 팔로워 스택은 **각각 따로 띄워야 한다.**

**그리퍼는 이 체인에 없다.** 리더가 넘기는 것은 EE pose 하나뿐이라, 그리퍼는 키보드
(`=`/`-`)나 트윈 연산으로 따로 조작한다. 리더 트리거로 묶는 방안은
[leader_gripper_mapping_design.md](leader_gripper_mapping_design.md) 에 있다 (설계안).

## 문서별 역할

| 문서 | 성격 | 독자 | 갱신 빈도 |
|---|---|---|---|
| `omy_leader_teleop_guide` | 운영 절차서 | 운용자 (콘솔 앞) | 명령이 바뀔 때 |
| `teleop_retarget_node_guide` | 노드 레퍼런스 | 튜닝·디버깅 | 노드가 바뀔 때 |
| `clutch_pedal_guide` | 장치 연동 | 풋페달 설치·운용 | 장치가 바뀔 때 |
| `external_input_adapters` | **경계 계약** | 외부 어댑터 작성자 | 계약이 바뀔 때 (드묾) |
| `leader_follower_mirroring_design` | 설계 근거·이력 | 구조를 재검토할 때 | 거의 없음 |
| `leader_gripper_mapping_design` | 설계안 (미반영) | 그리퍼 연동을 구현할 때 | 전제 확인 후 |

`external_input_adapters.md` 는 **rdfp 밖의 사람이 보는 문서** 다. rdfp 내부를
몰라도 읽히도록 유지한다.

## 자주 걸리는 함정 — 어디를 볼지

이 네 가지는 **에러 없이 조용히 실패** 한다.

| 증상 | 원인 | 문서 |
|---|---|---|
| 토픽이 아예 안 보인다 | `ROS_DOMAIN_ID` / RMW 불일치 — `rdfp_env` 미실행 | [절차서 §5-3](omy_leader_teleop_guide.md) |
| 토픽은 흐르는데 팔이 안 움직인다 | 클러치 자동 해제 → hold 재발행 중<br>`ros2 topic echo /teleop_retarget/clutch_state --once` 로 사유 확인 | [노드 가이드 §3-2](teleop_retarget_node_guide.md) |
| twist 는 나가는데 컨트롤러 명령이 없다 | `start_servo` 미호출 | [절차서 §5-2](omy_leader_teleop_guide.md) |
| 움직임이 끊기거나 계단식이다 | 실효 주기 < 10 Hz / stale stamp | [계약 §3-2, §3-3](external_input_adapters.md) |

## 관련 (teleop 밖)

- [../moveit/servo_client_programmers_guide.md](../moveit/servo_client_programmers_guide.md) — `servo_node` 서비스·상태 코드
- [../../src/rdfp/launch/README.md](../../src/rdfp/launch/README.md) — `teleop_mirror.launch.py` 인벤토리
- [../INDEX.md](../INDEX.md) — 저장소 전체 문서 인덱스
- 외부 저장소 `omy_leader_bridge` — OMY-L100 어댑터 구현

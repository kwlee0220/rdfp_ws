# data/

검증·재생용 데이터 파일을 둔다. 빌드 산출물이 아니며 패키지에 설치되지 않는다
(`setup.py` 의 `data_files` 는 `src/rdfp/config/` 만 다룬다).

| 항목 | 내용 |
|---|---|
| `ee_pose_bag/` | OMY-L100 리더 암의 EE pose 를 `zenoh_bridge` 로 받아 녹화한 rosbag2 (sqlite3). `/leader/ee_pose_states` (`geometry_msgs/PoseStamped`), 10328건 / 51.6초 |

## `ee_pose_bag` 사용법

`replay_panda_mock` 스택의 `ee_twist` 경로를 검증할 때 쓴다.

```bash
# 터미널 1 — 스택 (replay_arm_path 기본값 = ee_twist)
ros2 launch rdfp replay_panda_mock.launch.py

# 터미널 2 — 토픽 이름을 스택이 기대하는 /ee_pose 로 remap
ros2 bag play data/ee_pose_bag --remap /leader/ee_pose_states:=/ee_pose
```

주의할 점:

- **`-r` 로 배속을 올리면 로봇이 오히려 덜 움직인다.** 속도는 stamp 기준으로
  계산되어 배속과 무관한데 재생 지속시간만 짧아지기 때문이다. 상세는
  [docs/replay/replay_mock_stack_guide.md](../docs/replay/replay_mock_stack_guide.md) §3-4-1.
- 발행은 200 Hz 지만 **실효 주기는 19.3 Hz** 다 (90.4% 가 직전과 완전히 동일한
  메시지라 `dt<=0` 으로 버려진다). 정보 손실은 없다. 같은 문서 §3-4-3.
- 녹화 시점(2026-07-29)의 `header.stamp` 를 그대로 내보내므로, `ee_twist_publisher`
  가 출력 stamp 를 발행 시각으로 갈아끼우지 않으면 servo 가 전 구간을 stale 로
  폐기한다. 같은 문서 §3-4-2.

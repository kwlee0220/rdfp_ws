# 한 시점의 scene 물체 전체. 백엔드별 발행 노드가 `/scene/objects` 로 낸다.
#
# 이 토픽은 rosbag2 에 기록되어 학습 데이터의 일부가 된다. 트윈만 보는 채널(서비스 등)로
# 만들면 물체 위치가 데이터셋에 남지 않는다 — `rdfp_msgs/GripperCommand` 를 만든 것과
# 같은 이유다.
#
# 권장 QoS 는 RELIABLE / TRANSIENT_LOCAL / depth=1 이다. TRANSIENT_LOCAL 이라야 늦게
# 붙은 구독자(트윈·recorder)가 현재 scene 을 즉시 본다. **구독 측도 durability 를 맞춰야
# 한다** — volatile 로 구독하면 매칭 자체가 되지 않아 값이 영영 오지 않는다.

# stamp 는 발행 시각이며 **비워 두면 안 된다.** 적재 코드가 이 값을 그대로 쓰므로
# (`dataset/db/writers/base.py` 의 `extract_stamp`), 비면 epoch 0 에 적재되어 **어느
# 에피소드에도 속하지 못한다.** 로봇은 정상 동작하므로 데이터를 열어보기 전까지
# 드러나지 않는다.
#
# frame_id 는 **로봇 베이스 프레임**이며 현재 스택에서는 `panda_link0` 다. Gazebo 의
# world frame 은 로봇 베이스와 다르므로 변환 책임은 발행 노드가 진다. 여기서 안 맞추면
# 백엔드마다 다른 좌표가 섞여 데이터셋이 조용히 오염된다.
std_msgs/Header header

# 현재 scene 에 있는 물체 전체. **순서는 보장되지 않으므로** 인덱스가 아니라 `name` 으로
# 지목한다. 물체가 없으면 빈 배열이며, 그것도 'scene 이 비었다'는 유효한 상태다.
SceneObject[] objects

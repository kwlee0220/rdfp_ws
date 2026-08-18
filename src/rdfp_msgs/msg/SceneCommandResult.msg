# `SceneCommand` 처리 결과. 백엔드별 발행 노드가 명령 하나당 1건 발행한다.
#
# 트윈은 명령을 토픽으로 흘린 뒤 이 값이 갱신될 때까지 기다렸다가 연산을 끝낸다 —
# 그래야 `reset_scene` 이 완료를 보고할 수 있다. 결과를 서비스 응답으로 받으면
# rosbag2 에 기록되지 않아 나중에 "리셋이 실패했었는지" 를 알 수 없다
# (`gripper_control_node` 와 같은 구조다).
#
# **`/scene/objects` 를 결과 채널로 쓰지 않는 이유**: 그쪽은 상태를 주기적으로
# 발행하므로 명령과 무관하게 계속 갱신된다. 트윈이 '갱신됨'을 완료 신호로 삼으면
# 리셋 이전 상태를 완료로 오인한다.

# header.stamp 는 발행 시각, frame_id 는 ''.
std_msgs/Header header

# 처리 성공 여부.
bool success

# 실패 사유. 성공 시 ''.
string message

# 적용된 물체 수. 성공 시 요청한 개수와 같아야 한다.
int32 applied_count

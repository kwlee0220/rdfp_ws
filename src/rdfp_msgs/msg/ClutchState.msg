# teleop_retarget 의 클러치 상태 메시지.
# 상태가 바뀔 때만 발행하며, QoS 는 TRANSIENT_LOCAL 이라 늦게 구독한 노드도
# 현재 상태를 즉시 받는다 (SessionCommand 와 같은 규약).
# header.stamp 는 상태가 바뀐 시각, frame_id 는 ''.

std_msgs/Header header
# True 이면 클러치가 잡힌(engaged) 상태 — 리더 동작이 팔로워로 전달된다.
bool engaged
# engaged 이면 'engaged'. 아니면 해제 사유
# (예: 'clutch released by operator',
#      'leader pose stream stale for 0.55s',
#      'leader pose jumped 0.312m > 0.200m').
string reason

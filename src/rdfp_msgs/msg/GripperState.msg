# gripper 액션 피드백을 토픽으로 재발행하기 위한 상태 메시지.
# 필드 구성은 control_msgs/action/GripperCommand 의 Feedback 을 따른다.
# header.stamp 는 피드백 수신 시각, frame_id 는 ''.
std_msgs/Header header
# 현재 gripper 위치 (m).
float64 position
# 현재 효과 (힘, N).
float64 effort
# True 이면 stall 상태 (지정 position 에 도달하지 못하고 토크 한계에 걸림).
bool stalled
# True 이면 목표 position 에 도달했다.
bool reached_goal

# gripper 액션의 상태를 토픽으로 재발행하기 위한 상태 메시지.
# position/effort/stalled/reached_goal 은 control_msgs/action/GripperCommand 의
# Feedback·Result 와 동일한 필드 구성이며, 여기에 액션 goal 상태(status) 를 더한다.
# header.stamp 는 메시지 생성(수신) 시각, frame_id 는 ''.

# 액션 goal 상태 상수. 값은 action_msgs/msg/GoalStatus 의 STATUS_* 와 동일하다.
int8 STATUS_UNKNOWN=0
int8 STATUS_ACCEPTED=1
int8 STATUS_EXECUTING=2
int8 STATUS_CANCELING=3
int8 STATUS_SUCCEEDED=4
int8 STATUS_CANCELED=5
int8 STATUS_ABORTED=6

std_msgs/Header header
# 현재 gripper 위치 (m).
float64 position
# 현재 효과 (힘, N).
float64 effort
# True 이면 stall 상태 (지정 position 에 도달하지 못하고 토크 한계에 걸림).
bool stalled
# True 이면 목표 position 에 도달했다.
bool reached_goal
# 이 상태가 만들어진 시점의 goal 상태.
#   - result 기반: 종료 상태 (STATUS_SUCCEEDED / STATUS_CANCELED / STATUS_ABORTED)
#   - feedback 기반: STATUS_EXECUTING
# reached_goal=False 라도 STATUS_CANCELED 면 실패가 아니라 후속 명령에 의한
# 선점(preempt) 이다. 성공 판정은 status 로 한다.
int8 status

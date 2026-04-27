# SessionControlNode 가 'session' 토픽으로 발행하는 세션 상태 메시지.
# 명령 처리 결과로 변경된 세션 상태와 task_label 을 담는다.
# header.stamp 는 메시지 발행 시점의 timestamp, frame_id 는 ''.
std_msgs/Header header
# 세션 상태 ('IDLE' | 'IN_SESSION' | 'IN_EPISODE').
string state
# 현재 세션에 설정된 task label. 설정되지 않은 경우 ''.
string task_label

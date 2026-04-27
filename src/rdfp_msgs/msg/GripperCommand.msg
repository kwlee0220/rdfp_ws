# Teleop 에서 gripper 제어 의도를 기록/전파하기 위한 메시지.
# header.stamp 는 발행 시각, frame_id 는 ''.
std_msgs/Header header
# 'open' 또는 'close' 문자열로 gripper 동작을 지정한다.
string command

# Gripper 제어 명령. 이 토픽이 곧 학습 데이터의 **action 채널**이다.
#
# 액션(`control_msgs/GripperCommand`) 의 goal 전송은 서비스라 rosbag2 가 기록하지
# 못한다. 그래서 명령을 토픽으로 한 번 흘리고 `gripper_control_node` 가 그것을 받아
# 액션으로 전달한다 — 기록 가능성과 실행 경로를 함께 만족시키기 위한 구조다.
#
# 값은 `control_msgs/GripperCommand` goal 에 그대로 실린다. 심볼('open'/'close')이
# 아니라 숫자를 싣는 이유는, 심볼을 쓰면 그 의미(몇 m 인가)가 노드 상수에 남아
# 데이터셋이 자기 완결적이지 않게 되기 때문이다.
#
# `control_msgs/GripperCommand` 를 그대로 쓰지 않는 이유가 이 헤더다. 그쪽은 액션 goal
# 안에 들어가는 값 묶음이라 헤더가 없는데, 이 토픽은 **기록되는 채널**이라 시각이 반드시
# 필요하다. 적재 코드가 `header.stamp` 를 그대로 쓰므로(dataset/db/writers/base.py 의
# `extract_stamp`), 비워 두면 이 명령이 epoch 0 에 적재되어 **어느 에피소드에도 속하지
# 못한다.** 로봇은 정상 동작하므로 데이터를 열어보기 전까지 드러나지 않는다.
#
# header.stamp 는 발행 시각, frame_id 는 ''.
std_msgs/Header header

# 목표 위치 (m). 컨트롤러가 물린 관절 하나의 목표값이며, Panda 는
# `panda_finger_joint1` 기준 0.0 ~ 0.04 이다. **손가락 사이 거리가 아니라 그 절반**
# 이므로 실기 franka 의 move/grasp 가 쓰는 width 와 2배 차이가 난다.
float64 position

# 최대 파지력 (N). 0 이면 드라이버 기본 효과치를 쓴다.
# 물체를 쥐려면 0 보다 큰 값을 준다 — 실기 franka 계열에서 0 은 파지 없는 이동으로
# 해석될 수 있다.
float64 max_effort

# 선택. 사람이 읽기 위한 이름 ('open' / 'close' / 'grasp' 등).
# **제어에 사용되지 않는다.** 비어 있어도 동작에는 영향이 없으며, 데이터셋에서
# 구간을 눈으로 훑거나 필터링할 때만 쓴다.
#
# 값이 아니라 **의도**를 남기는 자리다. 예를 들어 close 와 grasp 는 position 이 같고
# max_effort 만 다른데, label 이 없으면 `max_effort > 0` 인지로 의도를 역추론해야 하고
# 목표가 늘어날수록 그 추론이 불안해진다.
#
# 반대로 이 값으로 **분기하는 제어 코드를 만들면 안 된다** — 그 순간 심볼 시절의 문제
# (명령의 의미가 데이터셋 밖에 있음)로 되돌아간다.
string label

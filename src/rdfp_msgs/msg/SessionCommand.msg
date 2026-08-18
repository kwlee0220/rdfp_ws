# SessionControlNode 가 'session' 토픽으로 발행하는 세션 상태 메시지.
# 명령 처리 결과로 변경된 세션 상태와 task_label 을 담는다.
# header.stamp 는 메시지 발행 시점의 timestamp, frame_id 는 ''.
std_msgs/Header header
# 세션 상태 ('IDLE' | 'IN_SESSION' | 'IN_EPISODE').
string state
# 현재 세션에 설정된 task label. 설정되지 않은 경우 ''.
string task_label

# ---- 아래 둘은 에피소드 종료 전이(IN_EPISODE -> IN_SESSION)에서만 의미를 갖는다 ----
#
# 다른 전이에서는 항상 ''다. 별도 토픽으로 빼지 않은 이유는, 후처리기의 에피소드
# 감지기가 이 토픽 하나만 소비하기 때문이다 — 토픽을 나누면 두 스트림을 stamp 로
# 맞추면서 순서·유실까지 다뤄야 한다.
#
# 이 값들은 rosbag2 에 기록되어 `sessions` 테이블의 success / metadata 컬럼이 된다.
# 서비스 응답으로만 주고받으면 기록되지 않아 데이터셋에 남지 않는다.

# 작업 성패 ('' | 'success' | 'failure').
# **'' 는 실패가 아니라 '판정 없음'이다** — 텔레오퍼레이션처럼 판정 주체가 없었거나,
# stop_session 으로 에피소드가 함께 닫힌 복구 경로다. DB 에는 NULL 로 들어간다.
# bool 을 쓰지 않는 이유가 이것이다: ROS bool 에는 '모름'이 없다.
string outcome

# 에피소드 재현·분석용 부가 정보의 **JSON object 문자열** (seed, scene, 초기 물체 배치,
# 실패 사유 등). 없으면 ''. ROS IDL 에 map/any 가 없어 문자열로 싣는다.
# DB 의 jsonb 컬럼에 그대로 들어가므로 최상위는 반드시 object('{...}')여야 한다.
string metadata

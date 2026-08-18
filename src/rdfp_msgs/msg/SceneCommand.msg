# 씬을 바꾸라는 명령. 백엔드별 발행 노드가 받아 자기 방식으로 적용한다
# (mock 은 planning scene, Gazebo 는 spawn/set_pose, Isaac 은 USD 조작).
#
# **물체 배치는 이미 정해진 상태로 온다.** 무작위 추출은 트윈이 하고 여기에는 그
# 결과만 실린다. 백엔드 노드가 뽑으면 (1) 같은 seed 로도 백엔드마다 다른 배치가
# 나오고, (2) 트윈이 실제 배치를 몰라 에피소드 metadata 에 남길 수 없다.
#
# 이 토픽도 rosbag2 에 기록된다. `reset_scene` 은 에피소드 **밖**에서 일어나므로
# 어느 에피소드에도 속하지 않지만, 세션 전체를 훑으면 "어떤 배치로 시작했는지" 를
# 되짚을 수 있다.

# header.stamp 는 발행 시각, frame_id 는 ''.
std_msgs/Header header

# 수행할 동작.
#   'reset'  objects 로 씬을 **통째로 교체**한다 (기존 물체는 전부 제거).
# 개별 추가/삭제 명령을 두지 않는 이유는 그 조합 로직이 클라이언트로 새어 나가고,
# 그러면 "어떤 배치였는지" 가 데이터셋 밖에 남기 때문이다.
string command

# 씬을 구성할 물체 전체. 빈 배열은 '씬을 비운다'는 뜻이다.
# pose 규약은 SceneObject 와 같다 — 로봇 베이스 프레임, orientation 은 xyzw.
SceneObject[] objects

# 어떤 레시피에서 왔는지 (트윈 설정의 `backend.scenes` 키). 기록용이며 제어에
# 쓰이지 않는다.
string scene

# 무작위 추출에 쓴 seed. 기록용이다.
#
# **이 값만으로는 재현되지 않는다** — 추출 알고리즘이 바뀌면 같은 seed 가 다른
# 배치를 만든다. 재현의 근거는 위의 `objects` 이며 seed 는 사람이 실행을 식별하는
# 용도다.
int64 seed

# 씬 안에 있는 물체 하나의 상태.
#
# mock(MoveIt planning scene) / Gazebo / Isaac Sim 은 물체를 표현하는 방식이 전혀
# 다르지만, 발행 노드가 각자 이 타입으로 변환해 하나의 토픽으로 낸다. 트윈과 후처리기가
# 환경 구현을 모르게 하기 위한 경계다.
#
# **주의: mock 의 물체는 물리를 갖지 않는다.** 파지에 실패해도 굴러떨어져도 pose 가
# 그대로이므로, mock 에서 수집한 에피소드로는 성패를 관측할 수 없다.

# 물체 식별자. 백엔드가 바뀌어도 같은 물체는 같은 이름을 유지해야 한다.
# 배열 인덱스가 아니라 **이 이름으로 물체를 지목한다** — 순서는 보장되지 않는다.
string name

# 기하 종류. `shape_msgs/SolidPrimitive` 의 이름을 **소문자로** 쓰거나 'mesh' 다.
# 흔한 값은 'box' / 'sphere' / 'cylinder' / 'mesh' 이며 'cone' / 'prism' 도 올 수 있다.
#
# uint8 상수가 아니라 문자열인 이유: 트윈의 `enums` 변환이 최상위 필드만 훑으므로
# 배열 안에 중첩된 이 필드는 숫자인 채로 노출된다(변환기를 중첩 지원으로 확장하면
# 경로 표기가 필요해져 설정만 복잡해진다). 기록되는 채널이라 데이터셋 가독성 면에서도
# 문자열이 낫다.
string type

# `type` 에 따른 크기 (m). **순서는 `shape_msgs/SolidPrimitive` 와 동일**하다.
#   box      3개 — x, y, z
#   sphere   1개 — 반지름
#   cylinder 2개 — **높이, 반지름** (SolidPrimitive 의 CYLINDER_HEIGHT=0,
#                  CYLINDER_RADIUS=1 순서다. 직관과 반대이므로 주의한다)
#   cone     2개 — 높이, 반지름
#   mesh     비어 있다
#
# 자체 순서를 정하지 않고 ROS 표준을 그대로 따르는 이유는, mock 백엔드가 planning
# scene 의 SolidPrimitive 를 **재배열 없이 그대로 옮길 수 있게** 하기 위해서다.
# 재배열이 들어가는 순간 그것이 조용히 틀릴 수 있는 지점이 된다.
#
# 길이가 종류마다 다르다는 점이 이 메시지가 **배열-of-구조체인 이유**다. ROS IDL 에
# ragged array 가 없어 JointState 식 병렬 배열(`string[] name` + `Pose[] pose` + …)로는
# 표현할 수 없다.
float64[] dimensions

# 물체의 pose. position 은 m 이고, orientation 은 **ROS 규약 xyzw · unit norm** 이다.
#
# Isaac Sim 은 wxyz(스칼라 우선)이므로 **발행 노드가 반드시 변환해야 한다.** 순서를
# 틀려도 4개 float 에 unit norm 이라 타입 검사도 정규화 검사도 전부 통과하고, 결과는
# 크래시가 아니라 '그럴듯하게 틀린 자세'다. 예를 들어 Isaac 의 '회전 없음'(w,x,y,z)=
# (1,0,0,0)을 그대로 옮기면 (x,y,z,w)=(1,0,0,0), 즉 x축 180° 회전이 되는데 이는
# '그리퍼가 아래를 본다'는 지극히 정상적인 자세라 눈으로 봐서는 드러나지 않는다.
#
# 검증은 노름이 아니라 **알려진 자세**로 한다 — 회전 없는 물체가 (0,0,0,1)로 나오는지,
# 그리고 축이 섞이는 비대칭 회전(z축 90° 등)이 맞게 나오는지 확인한다.
geometry_msgs/Pose pose

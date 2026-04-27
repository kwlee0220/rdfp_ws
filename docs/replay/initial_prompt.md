ROS2 Humble을 사용하는 환경에서
Panda 로봇의 움직이게 하고 이때 joint_states 토픽으로 발송되는 joint 상태 값들을
따로 저장했다가, 나중에 이 값을 읽어서 다시 로봇에게 전송하여 동일한 행동을 replay하는
프로그램 개발할 때 고려해야 할 사항에 대해 말해 줘.

다음은 주요 설계 고려 사항이야.
* 일단 데이터를 생성하는 로봇과 replay하는 로봇 모두 실제 panda 로봇이 아니라 mock components임.
* 상태 기반 replay를 고려하고 있어
* 전체 trajectory로 변환하는 방법을 고려하고 있어.
* 저장시점과 replay 시점의 joint 목록은 동일해.
* 데이터 생성 주기는 50Hz
* replay 목적은 1차는 demo playback이고 2차는 teaching 목적임
* 당장은 gripper는 고려하지 않음.
* joint 값 저장은 rosbag2로 기록하고 후처리 작업으로 replay하려고 함.
* 전체 과정에 MoveIt을 활용

주의할 점과 설계 사항들이 포함된 설계 문서를 작성해죠.
개발 절차도 포함시켜 줘.
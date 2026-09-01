"""scene 물체 상태 발행 서브패키지.

백엔드(mock / Gazebo / Isaac)마다 물체를 표현하는 방식이 다르지만, 각 백엔드의
`*_scene_state_node` 가 `rdfp_msgs/SceneObjects` 로 변환해 **하나의 토픽**
(`/scene/objects`)으로 낸다. 트윈과 후처리기는 그 토픽만 알면 되고 환경 구현을
모른다.

좌표계·단위·쿼터니언 순서를 맞추는 책임은 전부 발행 노드에 있다.
"""
